// -----------------------------------------------------------------------------
// @file pcap04.h
// @brief PCAP04 device interface header for nRF52 SDK projects.
//
// This header provides function declarations and type definitions for
// initializing, configuring, and interacting with the PCAP04 capacitive
// sensor device. It is intended for use in embedded firmware on nRF52
// microcontrollers, supporting NVRAM operations, measurement logging,
// and result processing.
// -----------------------------------------------------------------------------

#ifndef PCAP04_H
#define PCAP04_H

#include <stdint.h>
#include <stdbool.h>
#include "pcap_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tests the connection to the PCAP04 device via I2C or SPI.
 *
 * @retval true  If the connection test was successful.
 * @retval false If the connection test failed.
 */
bool test_pcap_connection(void);

/**
 * @brief Initializes the NVRAM with the specified PCAP version and measurement mode.
 *
 * @param[in] pcap_version         The version of the PCAP04 device.
 * @param[in] pcap_measurement_mode The measurement mode to initialize.
 */
void init_nvram(pcap04_version_t pcap_version, pcap_measurement_modes_t pcap_measurement_mode);

/**
 * @brief Converts the result registers to floating-point values based on the measurement mode.
 *
 * @param[in] pcap_measurement_mode The measurement mode to use for conversion.
 */
void convert_results_regs_to_float(pcap_measurement_modes_t pcap_measurement_mode);

/**
 * @brief Retrieves the measurement results.
 *
 * @param[in] pcap_measurement_mode The measurement mode to retrieve results for.
 *
 * @return Pointer to the structure containing the measurement results.
 */
struct pcap_results_t* get_results(pcap_measurement_modes_t pcap_measurement_mode);

/**
 * @brief Reads a specific result register.
 *
 * @param[in] addr The address of the result register to read.
 */
void read_result(uint8_t addr);

/**
 * @brief Reads all result registers.
 */
void readall_result(void);

/**
 * @brief Sends a command to the PCAP04 device.
 *
 * @param[in] opcode The opcode of the command to send.
 *
 * @retval true  If the command was sent successfully.
 * @retval false If the command failed to send.
 */
bool send_command(uint8_t opcode);

/**
 * @brief Writes a configuration value to a specific address.
 *
 * @param[in] addr The address to write the configuration value to.
 * @param[in] data The configuration value to write.
 */
void write_config(uint8_t addr, uint8_t data);

/**
 * @brief Initializes the PCAP04 device.
 *
 * @retval true  If the initialization was successful.
 * @retval false If the initialization failed.
 */
bool init_pcap04(void);

/**
 * @brief Logs the measurement results from the PCAP04 device.
 */
void log_pcap_measurements(void);
void log_dummy_pcap_measurements(void);

/**
 * @brief Reads a specific address from the NVRAM of the PCAP04 device.
 *
 * @param[in] addr The address to read from.
 */
void read_nvram(const uint16_t addr);

/**
 * @brief Writes data to a specific address in the NVRAM of the PCAP04 device.
 *
 * @param[in] addr The address to write to.
 */
void write_nvram(const uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif // PCAP04_H