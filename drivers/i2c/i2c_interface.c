/**
 * @file i2c_interface.c
 * @brief I2C (TWI) interface implementation for nRF52 SDK.
 *
 * Provides initialization, event handler, and data transmission functions for TWI/I2C communication.
 */

#include "i2c_interface.h"
#include <stdio.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <pcap_hal.h>

// Transfer completion flag
volatile bool m_xfer_done = false;

// Buffer for received sample data
uint8_t m_sample = 0;

// TWI instance structure
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);



#define HANDLE_ERROR(err_code) \
    if ((err_code) != NRF_SUCCESS) { \
        NRF_LOG_ERROR("I2C Error: %d", err_code); \
    }

/**
 * @brief Enable internal pull-ups on I2C lines.
 */
void i2c_enable_internal_pullups(void)
{
    nrf_gpio_cfg(
        PCAP04_PIN_I2C_SCL,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        PCAP04_PIN_I2C_SDA,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    NRF_LOG_INFO("Internal pull-ups enabled on I2C lines.");
    NRF_LOG_FLUSH();
}

/**
 * @brief Disable internal pull-ups on I2C lines.
 */
void i2c_disable_internal_pullups(void)
{
    nrf_gpio_cfg(
        PCAP04_PIN_I2C_SCL,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        PCAP04_PIN_I2C_SDA,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    NRF_LOG_INFO("Internal pull-ups disabled (lines in high-Z).");
    NRF_LOG_FLUSH();
}

/**
 * @brief Initialize the TWI (I2C) interface.
 *
 * Configures the TWI peripheral with the specified pins and frequency,
 * registers the event handler, and enables the TWI driver.
 */
void i2c_init(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("I2C INIT");

    const nrf_drv_twi_config_t twi_sensor_config = {
       .scl                = PCAP04_PIN_I2C_SCL,
       .sda                = PCAP04_PIN_I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_sensor_config, NULL, NULL);
    HANDLE_ERROR(err_code);

    nrf_drv_twi_disable(&m_twi);  // Disable TWI after use(&m_twi);
    NRF_LOG_INFO("I2C INIT COMPLETE");
    NRF_LOG_FLUSH();
}

void i2c_lines_high_z(void)
{
    nrf_gpio_cfg_input(PCAP04_PIN_I2C_SCL, NRF_GPIO_PIN_INPUT_DISCONNECT);
    nrf_gpio_cfg_input(PCAP04_PIN_I2C_SDA, NRF_GPIO_PIN_INPUT_DISCONNECT);
}

/**
 * @brief Transmit a 32-bit unsigned integer over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_uint(uint32_t value, uint8_t address)
{
    uint8_t recval = 0;
    ret_code_t err_code;
   
    uint8_t sendData[3] = {
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)
    };

    nrf_drv_twi_enable(&m_twi);  // Disable TWI after use(&m_twi);
    if (sendData[0] > 0x7F)
    {
        err_code = nrf_drv_twi_tx(&m_twi, address, sendData, 3, false);
        HANDLE_ERROR(err_code);

        return recval;
    }
    else
    {
        err_code = nrf_drv_twi_tx(&m_twi, address, sendData, 2, false);
        HANDLE_ERROR(err_code);

    }

    err_code = nrf_drv_twi_rx(&m_twi, address, &recval, 1);
    HANDLE_ERROR(err_code);

    nrf_drv_twi_disable(&m_twi);  // Disable TWI after use(&m_twi);
    i2c_lines_high_z();
    return recval;
}

/**
 * @brief Transmit a 32-bit unsigned long over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_ulong(uint32_t value, uint8_t address)
{
    uint8_t recval = 0;
    ret_code_t err_code;
   
    uint8_t sendData[3] = {
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)
    };
    nrf_drv_twi_enable(&m_twi);  // Disable TWI after use(&m_twi);
    err_code = nrf_drv_twi_tx(&m_twi, address, sendData, 3, false);
    HANDLE_ERROR(err_code);

    nrf_drv_twi_disable(&m_twi);  // Disable TWI after use(&m_twi);
    i2c_lines_high_z();
    return recval;
}

/**
 * @brief Transmit a 16-bit unsigned short over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_ushort(uint16_t value, uint8_t address)
{
    uint8_t recval = 0;
    ret_code_t err_code;
    
    uint8_t sendData[2] = {
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    nrf_drv_twi_enable(&m_twi);  // Disable TWI after use(&m_twi);
    if (sendData[0] > 63)
    {
        err_code = nrf_drv_twi_tx(&m_twi, address, sendData, 1, false);
        HANDLE_ERROR(err_code);
    }
    else
    {
        err_code = nrf_drv_twi_tx(&m_twi, address, sendData, 2, false);
        HANDLE_ERROR(err_code);
    }

    err_code = nrf_drv_twi_rx(&m_twi, address, &recval, 1);
    HANDLE_ERROR(err_code);
    nrf_drv_twi_disable(&m_twi);  // Disable TWI after use(&m_twi);
    i2c_lines_high_z();
    return (uint8_t)(recval & 0xFF);
}

/**
 * @brief Transmit an 8-bit unsigned char over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_uchar(uint8_t value, uint8_t address)
{
    uint8_t recval = 0;
    ret_code_t err_code;
    
    nrf_drv_twi_enable(&m_twi);  // Disable TWI after use(&m_twi);
    err_code = nrf_drv_twi_tx(&m_twi, address, &value, 1, false);
    HANDLE_ERROR(err_code);

    if (value > 0x7F)
    {

        return recval;
    }
    err_code = nrf_drv_twi_rx(&m_twi, address, &recval, 1);
    HANDLE_ERROR(err_code);

    nrf_drv_twi_disable(&m_twi);  // Disable TWI after use(&m_twi);
    i2c_lines_high_z();
    return recval;
}