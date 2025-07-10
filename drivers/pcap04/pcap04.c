// -----------------------------------------------------------------------------
// @file pcap04.c
// @brief PCAP04 device interface implementation for nRF52 SDK projects.
//
// This file provides the implementation for initializing, configuring,
// and interacting with the PCAP04 capacitive sensor device.
// It includes functions for NVRAM operations, measurement logging,
// and result processing.
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "pcap_hal.h"
#include "pcap_types.h"
#include "pcap_registers.h"
#include "pcap_firmware.h"
#include "pcap_calibrations.h"
#include "pcap_configurations.h"
#include "pcap04.h"
#include <math.h>


#define INTN_PG5                 23   /**< DataIsReady from PCAP04. */


bool cdc_complete_flag = false;

bool initialized = false;

  pcap04_version_t pcap_version;

  pcap_measurement_modes_t pcap_measurement_mode;

  pcap_config_t pcap_config;



//pcap_results_t *pcap_results;
struct pcap_results_t  pcap_results;
struct __PCAP_NVRAM_T pcap_nvram;              /**< 1KB NVRAM of the PCAP04 */
struct __PCAP_NVRAM_T pcap_nvram_mirror;       /**< Mirror of the device NVRAM */
struct __PCAP_RESULTS_REGS_T pcap_results_regs;

// ------------------- OPCODE DEFINITIONS -------------------
// wr_mem: Write to NVRAM opcode structure
// rd_mem: Read from NVRAM opcode structure
// wr_config: Write to configuration register opcode structure
// rd_config: Read from configuration register opcode structure
// rd_result: Read result register opcode structure
// por_reset: Power-On Reset command opcode structure
// initialize_op: Initialize operation command opcode structure
// cdc_start: Start capacitive measurement command opcode structure
// rdc_start: Start resistive measurement command opcode structure
// dsp_trig: Trigger DSP function command opcode structure
// nv_store: Store current configuration to NVRAM command opcode structure
// nv_recall: Recall configuration from NVRAM command opcode structure
// nv_erase: Erase NVRAM command opcode structure
// test_read: Test read command opcode structure

pcap_opcode_nvram_t wr_mem = { .nvram = { .data = 0, .addr = 0, .op_code = WR_NVRAM } };
pcap_opcode_nvram_t rd_mem = { .nvram = { .data = 0, .addr = 0, .op_code = RD_NVRAM } };
pcap_opcode_config_t wr_config = { .config = { .data = 0, .addr = 0, .op_code = WR_CONFIG } };
pcap_opcode_config_t rd_config = { .config = { .data = 0, .addr = 0, .op_code = RD_CONFIG } };
pcap_opcode_result_t rd_result = { .result = { .data = 0, .addr = 0, .op_code = RD_RESULT } };
pcap_opcode_command_t por_reset = { .command = { .op_code = POR_RESET } };
pcap_opcode_command_t initialize_op = { .command = { .op_code = INITIALIZE_OP } };
pcap_opcode_command_t cdc_start = { .command = { .op_code = CDC_START } };
pcap_opcode_command_t rdc_start = { .command = { .op_code = RDC_START } };
pcap_opcode_command_t dsp_trig = { .command = { .op_code = DSP_TRIG } };
pcap_opcode_command_t nv_store = { .command = { .op_code = NV_STORE } };
pcap_opcode_command_t nv_recall = { .command = { .op_code = NV_RECALL } };
pcap_opcode_command_t nv_erase = { .command = { .op_code = NV_ERASE } };
pcap_opcode_testread_t test_read = { .testread = { .fixed = TEST_READ_LOW, .op_code = TEST_READ_HIGH } };

// This script provides the implementation for interfacing with the PCAP04 device.
// It includes functions for initialization, configuration, NVRAM operations, measurement logging,
// and result processing. The script is designed for embedded systems using the NRF52 SDK.

/**
 * @brief Sends a command to the PCAP04 device.
 *
 * @param[in] opcode The command opcode to send.
 *
 * @retval true  If the command was successfully sent.
 * @retval false If the command was invalid or failed to send.
 */
bool send_command(const uint8_t opcode)
{

  if (opcode == POR_RESET)
  {
    pcap_hal_transmit_uchar(por_reset.opcode);
  }
  else if (opcode == INITIALIZE_OP)
  {
    pcap_hal_transmit_uchar(initialize_op.opcode);
  }
  else if (opcode == CDC_START)
  {
    pcap_hal_transmit_uchar(cdc_start.opcode);
  }
  else if (opcode == RDC_START)
  {
    pcap_hal_transmit_uchar(rdc_start.opcode);
  }
  else if (opcode == DSP_TRIG)
  {
    pcap_hal_transmit_uchar(dsp_trig.opcode);
  }
  else if (opcode == NV_STORE)
  {
    pcap_hal_transmit_uchar(nv_store.opcode);
  }
  else if (opcode == NV_RECALL)
  {
    pcap_hal_transmit_uchar(nv_recall.opcode);
  }
  else if (opcode == NV_ERASE)
  {
    pcap_hal_transmit_uchar(nv_erase.opcode);
  }
  else
  {
    return false;
  }
  return true;
}

/**
 * @brief Initializes the NVRAM of the PCAP04 device based on version and measurement mode.
 *
 * @param[in] pcap_version          The version of the PCAP04 device.
 * @param[in] pcap_measurement_mode The measurement mode to configure.
 */
void init_nvram(pcap04_version_t pcap_version, pcap_measurement_modes_t pcap_measurement_mode){
  static pcap_data_vector_t firmware;
  static pcap_data_vector_t calibration;
  static pcap_data_vector_t configuration;

  static unsigned short calib_offset;
  static unsigned short calib_index;

   NRF_LOG_INFO("INIT NVRAM");
   NRF_LOG_FLUSH();

  if (pcap_version == PCAP04_V0)
  {
    if (pcap_measurement_mode == STANDARD)
    {
      firmware.data = &PCap04v0_standard_v1[0];
      firmware.size = sizeof(PCap04v0_standard_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_standard_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_standard_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = NULL;
      calibration.size = 0;
      calibration.nvram_loc = 0x320;
    }
    else if (pcap_measurement_mode == HUMIDITY)
    {
      firmware.data = &PCap04v0_linearize_v1_1[0];
      firmware.size = sizeof(PCap04v0_linearize_v1_1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_humidity_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_humidity_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v0_humidity_v1_calibration[0];
      calibration.size = sizeof(PCap04v0_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
    else if (pcap_measurement_mode == PRESSURE)
    {
      firmware.data = &PCap04v0_linearize_v1_1[0];
      firmware.size = sizeof(PCap04v0_linearize_v1_1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_pressure_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_pressure_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v0_pressure_v1_calibration[0];
      calibration.size = sizeof(PCap04v0_pressure_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
  }
  else if (pcap_version == PCAP04_V1)
  {
    if (pcap_measurement_mode == STANDARD)
    {
      firmware.data = &PCap04v1_standard_v1[0];
      firmware.size = sizeof(PCap04v1_standard_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_custom_configuration_v2[0];
      configuration.size = sizeof(PCap04v1_custom_configuration_v2);
      configuration.nvram_loc = 0x03C0;

      calibration.data = NULL;
      calibration.size = 0;
      calibration.nvram_loc = 0x320;
    }
    else if (pcap_measurement_mode == HUMIDITY)
    {
      firmware.data = &PCap04v1_linearize_v1[0];
      firmware.size = sizeof(PCap04v1_linearize_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_humidity_v1_configuration[0];
      configuration.size = sizeof(PCap04v1_humidity_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v1_humidity_v1_calibration[0];
      calibration.size = sizeof(PCap04v1_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
    else if (pcap_measurement_mode == PRESSURE)
    {
      firmware.data = &PCap04v1_linearize_v1[0];
      firmware.size = sizeof(PCap04v1_linearize_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_pressure_v1_configuration[0];
      configuration.size = sizeof(PCap04v1_pressure_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v1_pressure_v1_calibration[0];
      calibration.size = sizeof(PCap04v1_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
  }

  calib_offset = calibration.nvram_loc - PCAP_NVRAM_FW_SIZE;
  calib_index = PCAP_NVRAM_MAX_INDEX_FW_CAL0 - calib_offset;

  // copy firmware to pcap nvram
  if (firmware.size <= PCAP_NVRAM_FW_SIZE){
    memcpy(&pcap_nvram.FW, firmware.data, firmware.size);
  }else {
    memcpy(&pcap_nvram.FW, firmware.data, PCAP_NVRAM_FW_SIZE);
    memcpy(&pcap_nvram.FW_CAL0, firmware.data + PCAP_NVRAM_FW_SIZE, firmware.size-PCAP_NVRAM_FW_SIZE);
  }

  // copy calibration data to pcap nvram if exists
  if (calibration.size > 0){
    if(calibration.size <= calib_index) {
        memcpy(&pcap_nvram.FW_CAL0.data[calib_offset], calibration.data, calibration.size);
    }else {
      memcpy(&pcap_nvram.FW_CAL0.data[calib_offset], calibration.data, calib_index);
      memcpy(&pcap_nvram.FW_CAL1.data[0], calibration.data + calib_index, calibration.size - calib_index);
    }
  }

  memcpy(&pcap_nvram.CFG.CFG0, configuration.data, configuration.size);

}

/**
 * @brief Writes a configuration value to a specific address in the PCAP04 device.
 *
 * @param[in] addr The address to write to.
 * @param[in] data The data to write.
 */
void write_config(const uint8_t addr, const uint8_t data)
{
    wr_config.config.addr = addr;
    wr_config.config.data = data;

    if ((addr >= 0) && (addr < PCAP_NVRAM_CFG_SIZE - 2)) // size-2 to skip the CHARGE_PUMP registers
    {   
        pcap_hal_transmit_uint(wr_config.opcode); //Correct
    }
    else if ((addr == PCAP_NVRAM_CFG_SIZE - 2) || (addr == PCAP_NVRAM_CFG_SIZE - 1))
    {
        NRF_LOG_INFO("write config to address: %d - skipped (CHARGE_PUMP)", addr);
        NRF_LOG_FLUSH();
    }
    else
    {
        NRF_LOG_WARNING("write config address not in range [0-63]: %d", addr);
        NRF_LOG_FLUSH();
    }
}

/**
 * @brief Reads all NVRAM data from the PCAP04 device.
 */
void readall_nvram()
{
  NRF_LOG_INFO("Read all NVRAM");
  for (uint32_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
  {
    read_nvram(addr);
  }
}

/**
 * @brief Reads a specific address from the NVRAM of the PCAP04 device.
 *
 * @param[in] addr The address to read from.
 */
void read_nvram(const uint16_t addr)
{
  static uint32_t s = 0;
  static uint8_t *cfg_p = NULL;


  if ((addr >= 0) && (addr < PCAP_NVRAM_SIZE))
  {
    rd_mem.nvram.addr = 0xFFFF & addr;
    rd_mem.nvram.data = 0;

    rd_mem.nvram.data = pcap_hal_transmit_uint(rd_mem.opcode); //Correct 

    if (rd_mem.nvram.addr >= 0 && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW)
    {
      s = rd_mem.nvram.addr;
      pcap_nvram_mirror.FW.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL0)
    {
      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW;
      pcap_nvram_mirror.FW_CAL0.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL0 && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL1)
    {
      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL0;
      pcap_nvram_mirror.FW_CAL1.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL1 && rd_mem.nvram.addr < PCAP_NVRAM_SIZE)
    {

      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL1;

      cfg_p = (uint8_t *)(&pcap_nvram_mirror.CFG.CFG0 + s);
      *cfg_p = (uint8_t)rd_mem.nvram.data;    
    }
  }
  else
  {
    NRF_LOG_INFO("Read nvram address not in range [0-1023] - 0x%02X", addr);
    NRF_LOG_FLUSH();
  }
}

/**
 * @brief Writes all NVRAM data to the PCAP04 device.
 */
void writeall_nvram(){
  NRF_LOG_INFO("Write nvram ")
  for (uint32_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
  {
    if (addr != PCAP_NVRAM_RUNBIT_INDEX){
      write_nvram(addr);
    }
  }
  write_nvram(PCAP_NVRAM_RUNBIT_INDEX);
}

/**
 * @brief Writes data to a specific address in the NVRAM of the PCAP04 device.
 *
 * @param[in] addr The address to write to.
 */
void write_nvram(const uint16_t addr){
  static uint32_t s = 0;
  static uint8_t * cfg_p = NULL;

  if ((addr >= 0) && (addr < PCAP_NVRAM_SIZE))
  {
    wr_mem.nvram.addr = 0xFFFF & addr;
    wr_mem.nvram.data = 0;

    if (wr_mem.nvram.addr >= 0 && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW)
    {
      s = wr_mem.nvram.addr;
      wr_mem.nvram.data = pcap_nvram.FW.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL0)
    {
      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW;
      wr_mem.nvram.data = pcap_nvram.FW_CAL0.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL0 && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL1)
    {
      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL0;
      wr_mem.nvram.data = pcap_nvram.FW_CAL1.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL1 && wr_mem.nvram.addr < PCAP_NVRAM_SIZE)
    {

      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL1;

      cfg_p = (uint8_t*)(&pcap_nvram.CFG.CFG0 + s);
      wr_mem.nvram.data = *cfg_p;
    }
    pcap_hal_transmit_uint(wr_mem.opcode); // Correct

  }
  else
  {
    NRF_LOG_INFO("Write nvram address not in range [0-1023] - 0x%02X", addr);
    NRF_LOG_FLUSH();
  }
}

/**
 * @brief Validates the NVRAM data by comparing it with the mirror copy.
 */
bool validate_nvram(){
    static uint8_t* nvram_p;
    static uint8_t* nvram_mirror_p;
    bool all_matched = true;

    readall_nvram();

    nvram_p = &pcap_nvram.FW.data[0];
    nvram_mirror_p = &pcap_nvram_mirror.FW.data[0];

    for (size_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
    {
        if (*nvram_p != *nvram_mirror_p){
            NRF_LOG_INFO("Mismatch at address 0x%04X: NVRAM=0x%02X, Mirror=0x%02X", (unsigned int)addr, *nvram_p, *nvram_mirror_p);
            NRF_LOG_FLUSH();
            nrf_delay_ms(10);
            all_matched = false;
        }
        nvram_p++;
        nvram_mirror_p++;
    }
    if (all_matched)
    {
        NRF_LOG_INFO("NVRAM validation passed: all values match.");
        NRF_LOG_FLUSH();
    }
    return all_matched;
}

/**
 * @brief Prints the configuration data of the PCAP04 device.
 */
void print_config(){
  uint32_t s = 0;
  uint32_t offset = 0;

  uint8_t* cfg_p = NULL;
  uint32_t regval = 0x00;

  for (uint32_t i = PCAP_NVRAM_MAX_INDEX_FW_CAL1; i < PCAP_NVRAM_SIZE; i++){
    nrf_delay_ms(50);
    
    s = i - (PCAP_NVRAM_MAX_INDEX_FW_CAL1);

    cfg_p = (uint8_t *)(&pcap_nvram.CFG.CFG0 + s);

    regval = regval | ((uint32_t)(*cfg_p) << 8*offset);
    offset++;
    if ((i+1)%4 == 0){
      NRF_LOG_INFO("0x%08X", regval);
      NRF_LOG_FLUSH();
      offset = 0;
      regval = 0;
    }
  }
  
}

/**
 * @brief Checks if the PCAP04 device has been initialized.
 *
 * @retval true  If the device is initialized.
 * @retval false If the device is not initialized.
 */
bool is_pcap_initialized(void)
{
    return initialized;
}

/**
 * @brief Initializes the PCAP04 device.
 *
 * @retval true  If initialization was successful.
 * @retval false If initialization failed.
 */
bool init_pcap04()
{
    NRF_LOG_INFO("Initializing PCAP04...");

    nrf_delay_ms(10);

    //Power On Reset After Power up reset, ?Write Config.? may be necessary
    send_command(POR_RESET);

    NRF_LOG_INFO("POR has been reset");

    nrf_delay_ms(10);

    write_config(47, 0); // STOP DSP

    NRF_LOG_INFO("Config has been written");
    NRF_LOG_FLUSH();
    nrf_delay_ms(10);

    //readall_nvram();

    //NRF_LOG_INFO("NVRAM has been read");

    //NRF_LOG_INFO("RUNBIT: on PCAP NVRAM - 0x%02X", pcap_nvram_mirror.CFG.CFG47.REGVAL);
    //NRF_LOG_INFO("RUNBIT: on NRF NVRAM - 0x%02X", pcap_nvram.CFG.CFG47.REGVAL);
    //NRF_LOG_FLUSH();

    writeall_nvram();

    NRF_LOG_INFO("NVRAM has been written");
    NRF_LOG_FLUSH();

    bool nvram_ok = validate_nvram();

    if (!nvram_ok) {
        NRF_LOG_ERROR("NVRAM validation failed: mismatches found. Initialization aborted.");
        initialized = false;
        return false;
    }

    NRF_LOG_INFO("RUNBIT: on PCAP NVRAM valid - 0x%02X", pcap_nvram_mirror.CFG.CFG47.REGVAL);

    send_command(INITIALIZE_OP);

    NRF_LOG_INFO("Initialize operation has been sent");
    NRF_LOG_FLUSH();

    //send_command(CDC_START);
    //send_command(POR_RESET);

    initialized = true;

    nrf_delay_ms(10);

    NRF_LOG_INFO("Valid config:");

    //print_config();

    NRF_LOG_INFO("PCAP initialized");
    NRF_LOG_FLUSH();

    //log_dummy_pcap_measurements();

    return initialized;
}


/**
 * @brief Reads a specific result register from the PCAP04 device.
 *
 * @param[in] addr The address of the result register to read.
 */
void read_result(const uint8_t addr)
{
  static unsigned int *res_p = NULL;
  static int byte_offset = 0;
  static uint8_t _addr = 0;

  static unsigned int temp = 0;

  _addr = (uint8_t) addr / 4;

  byte_offset = (int)addr % 4;

  if ((addr >= 0) && (addr < PCAP_RESULTS_SIZE + PCAP_STATUS_SIZE))
  {
    rd_result.result.addr = addr;
    rd_result.result.data = 0;

    rd_result.result.data = pcap_hal_transmit_ushort(rd_result.opcode); // Correct

    if (_addr < 8)
    {
        if ((rd_result.result.addr % 4) == 0)
        {
          temp = 0;
        }

      res_p = (unsigned int *)(&pcap_results_regs.RES0.REGVAL + _addr);

      temp |= (rd_result.result.data << byte_offset * 8);

      *res_p = temp;

    }
    else if (_addr == 8)
    {
      if (byte_offset == 0)
      {
        pcap_results_regs.STATUS0.REGVAL = rd_result.result.data;
      }
      else if (byte_offset == 1)
      {
        pcap_results_regs.STATUS1.REGVAL = rd_result.result.data;
      }
      else if (byte_offset == 2)
      {
        pcap_results_regs.STATUS2.REGVAL = rd_result.result.data;
      }
    }
  
  }
  else
  {
    NRF_LOG_INFO("Read result address not in range [0-34] - 0x%02X", addr);
    NRF_LOG_FLUSH();
  }
}

/**
 * @brief Converts the result registers to floating-point values based on the measurement mode.
 *
 * @param[in] pcap_measurement_mode The measurement mode to use for conversion.
 */
void convert_results_regs_to_float(pcap_measurement_modes_t pcap_measurement_mode){

  uint32_t ireg = 0;
  uint32_t regoffset = PCAP_RESULT_REG_SIZE;

  uint32_t regval = 0;
  uint32_t decimal_part = 0;
  float decimal_part_f = 0;
  uint32_t integer_part = 0;

  for(ireg = 0; ireg < PCAP_RESULTS_SIZE/regoffset; ireg++){
        regval = *(&pcap_results_regs.RES0.REGVAL + ireg);
          float (*scaling_params)[3];
          if(pcap_measurement_mode == HUMIDITY)
          {
             scaling_params = scaling_params_humidity;
          }
          else if(pcap_measurement_mode == PRESSURE)
          {
             scaling_params = scaling_params_pressure;
          }
          else
          {
             scaling_params = scaling_params_standard;
          }

        
          float fpp = scaling_params[ireg][0];
          float factor = scaling_params[ireg][1];
          float offset = scaling_params[ireg][2];

          decimal_part_f = (regval / powf(2, fabsf(fpp))) * factor + offset;

        memcpy((void*)((uintptr_t)(&pcap_results) + ireg*regoffset), &decimal_part_f, regoffset);
  }
}

/**
 * @brief Tests the connection to the PCAP04 device via I2C.
 *
 * @retval true  If the connection test was successful.
 * @retval false If the connection test failed.
 */
bool test_pcap_connection(void)
{
    bool testres = false;
    uint8_t recval = 0;
    NRF_LOG_INFO("TEST PCAP CONNECTION");
    NRF_LOG_FLUSH();  
    recval = pcap_hal_transmit_uchar(((test_read.opcode >> 8) & 0xFF));
    nrf_delay_ms(100);
    // For compatibility, check if the response is as expected (0x11)
    if (recval == 0x11) {
        NRF_LOG_INFO("Device working at address: 0x%x", ADDRESS_PCAP04);
        NRF_LOG_INFO("I2C/SPI write+read test: success");
        NRF_LOG_FLUSH();
        testres = true;
    } else {
        NRF_LOG_INFO("I2C/SPI write+read test: Failed, expected 0x11, got 0x%x", recval);
        NRF_LOG_FLUSH();
        testres = false;
    }
    NRF_LOG_INFO("I2C/SPI read test end");
    NRF_LOG_FLUSH();
    return testres;
}


/**
 * @brief Retrieves the measurement results from the PCAP04 device.
 *
 * @param[in] pcap_measurement_mode The measurement mode to use for retrieving results.
 *
 * @return A pointer to the structure containing the measurement results.
 */
struct pcap_results_t* get_results(pcap_measurement_modes_t pcap_measurement_mode){

  readall_result();

  convert_results_regs_to_float(pcap_measurement_mode);

  return &pcap_results;
}

/**
 * @brief Reads all result registers from the PCAP04 device.
 */
void readall_result()
{
  for (uint32_t addr = 0; addr < PCAP_RESULTS_SIZE; addr++)
  {
    read_result((uint8_t)addr);
  }
}


/**
 * @brief Dummy Logs the measurement results from the PCAP04 device.
 */
void log_dummy_pcap_measurements(void)
{
    if (!initialized) {
        NRF_LOG_WARNING("PCAP04 not initialized. Skipping measurement dummy log.");
        return;
    }
    struct pcap_results_t* pcap0_results;

    // to trigger the measurements
    send_command(CDC_START);
    pcap0_results = get_results(NULL);
    NRF_LOG_INFO("Load Dummy Log Measurements");
    
}


/**
 * @brief Logs the measurement results from the PCAP04 device.
 */
void log_pcap_measurements(void)
{
    if (!initialized) {
        NRF_LOG_WARNING("PCAP04 not initialized. Skipping measurement log.");
        return;
    }
    struct pcap_results_t* pcap1_results;
    float result0, result1, result2;

    // to trigger the measurements
    send_command(CDC_START);

    pcap1_results = get_results(NULL);
    result0 = pcap1_results->C0_over_CREF;
    result1 = pcap1_results->C1_over_CREF;
    result2 = pcap1_results->C2_over_CREF;

    NRF_LOG_INFO("Measurements");
    NRF_LOG_INFO("C0_over_CREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(result0));
    NRF_LOG_INFO("C1_over_CREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(result1));
    NRF_LOG_INFO("C2_over_CREF: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(result2));
    NRF_LOG_FLUSH();
}


