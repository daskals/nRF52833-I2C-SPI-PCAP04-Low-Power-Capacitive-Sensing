/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**
 * @file spi_interface.h
 * @brief SPI interface header for nRF52 SDK.
 *
 * Provides initialization and data transmission functions for SPI communication.
 */

#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <stdint.h>
#include "nrf_drv_spi.h"  /* nRF52 SPI driver */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SPI interface.
 *
 * Configures and enables the SPI peripheral for communication.
 */
void spi_init(void);

/**
 * @brief Transmit a 32-bit unsigned integer over SPI and receive a response.
 *
 * @param value 32-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_uint(uint32_t value);

/**
 * @brief Transmit a 16-bit unsigned short over SPI and receive a response.
 *
 * @param value 16-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_ushort(uint16_t value);

/**
 * @brief Transmit an 8-bit unsigned char over SPI and receive a response.
 *
 * @param value 8-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_uchar(uint8_t value);

/**
 * @brief Test SPI communication by sending a command and logging the response.
 */
void spi_test(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_INTERFACE_H */
// End of file

