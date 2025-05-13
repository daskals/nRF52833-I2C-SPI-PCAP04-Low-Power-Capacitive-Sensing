/**
 * @file pcap_hal.h
 * @brief Hardware abstraction layer for PCAP04 communication (I2C/SPI selectable).
 */

#ifndef PCAP_HAL_H
#define PCAP_HAL_H

#include <stdint.h>
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Communication interface selection.
 */
typedef enum {
    PCAP_HAL_IFACE_I2C = 0,
    PCAP_HAL_IFACE_SPI = 1
} pcap_hal_iface_t;

#define PCAP04_PIN_I2C_SCL   27 
#define PCAP04_PIN_I2C_SDA   26 

#define PCAP04_PIN_IIC_EN     23 // Serial interface select pin (0 = SPI, 1 = I2C)

#define PCAP04_PIN_SPI_CS         22 // SPI CS pin
#define PCAP04_PIN_SPI_MOSI       21 // SPI SCK pin
#define PCAP04_PIN_SPI_SCK        20 
#define PCAP04_PIN_SPI_MISO       19 // SPI SCK pin



#define ADDRESS_PCAP04           0x28 /**< I2C address for PCAP04. */

void pcap_hal_select_interface(pcap_hal_iface_t iface);

/**
 * @brief Initialize the selected interface (I2C or SPI).
 * @param iface Interface to initialize.
 */
void pcap_hal_init(pcap_hal_iface_t iface);

/**
 * @brief Transmit a 32-bit unsigned integer and optionally receive a response.
 * @param value   Value to transmit.
 * @param address Address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t pcap_hal_transmit_uint(uint32_t value);

/**
 * @brief Transmit a 16-bit unsigned short and optionally receive a response.
 */
uint8_t pcap_hal_transmit_ushort(uint16_t value);

/**
 * @brief Transmit an 8-bit unsigned char and optionally receive a response.
 */
uint8_t pcap_hal_transmit_uchar(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // PCAP_HAL_H
