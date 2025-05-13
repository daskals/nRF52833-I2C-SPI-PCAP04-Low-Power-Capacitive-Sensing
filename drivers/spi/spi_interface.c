/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**
 * @file spi_interface.c
 * @brief SPI interface implementation for nRF52 SDK.
 *
 * Provides initialization, event handler, and data transmission functions for SPI communication.
 */

#include "nrf_drv_spi.h"           /* nRF52 SPI driver */
#include "app_util_platform.h"     /* Utility macros and platform definitions */
#include "nrf_gpio.h"              /* GPIO functions */
#include "nrf_delay.h"             /* Delay functions */
#include "boards.h"                /* Board pin definitions */
#include "app_error.h"             /* Error handling macros */
#include <string.h>                 /* Standard string functions */
#include "nrf_log.h"               /* Logging macros */
#include "nrf_log_ctrl.h"          /* Logging control */
#include "nrf_log_default_backends.h" /* Logging backends */
#include "spi_interface.h"         /* SPI interface header */
#include "pcap_hal.h"              /* PCAP hardware abstraction layer */

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(1);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define SPI_TEST_COMMAND 0x7E   /**< Example: RD_CONFIG command code. */
#define SPI_RX_LEN       4      /**< Number of bytes expected in SPI response. */

#define TEST_STRING "Nordic"   /**< Test string for SPI TX buffer. */
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];  /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief Error handler macro for SPI operations.
 * Logs error if err_code is not NRF_SUCCESS.
 */
#define HANDLE_ERROR(err_code) \
    if ((err_code) != NRF_SUCCESS) { \
        NRF_LOG_ERROR("SPI Error: %d", (err_code)); \
    }

/**
 * @brief Initialize the SPI interface.
 *
 * Configures the SPI peripheral with board-specific pin assignments and settings.
 */
void spi_init(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("SPI INIT");
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = PCAP04_PIN_SPI_CS;   /* Chip select pin */
    spi_config.miso_pin = PCAP04_PIN_SPI_MISO; /* Master In Slave Out pin */
    spi_config.mosi_pin = PCAP04_PIN_SPI_MOSI; /* Master Out Slave In pin */
    spi_config.sck_pin  = PCAP04_PIN_SPI_SCK;  /* Serial clock pin */
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M; /* Set SPI clock speed to 1 MHz */
    /* SCK active low, sample on trailing edge of clock. */
    /* CPOL = 1: Clock idle high */
    /* CPHA = 1: Data sampled on falling edge (second clock edge) */
    spi_config.mode = NRF_DRV_SPI_MODE_1; /* CPOL = 1, CPHA = 1: Mode 3 */

    err_code = nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);
    HANDLE_ERROR(err_code);

    NRF_LOG_INFO("SPI example started.");
    NRF_LOG_INFO("SPI INIT COMPLETE");
    NRF_LOG_FLUSH();
}

/**
 * @brief Test SPI communication by sending a command and logging the response.
 */
void spi_test(void)
{
    ret_code_t err_code;
    uint8_t tx_buf[1] = {SPI_TEST_COMMAND};
    uint8_t rx_buf[SPI_RX_LEN] = {0};

    NRF_LOG_INFO("Sending SPI command 0x%02X", SPI_TEST_COMMAND);
    NRF_LOG_FLUSH();
    // Pull CS low (handled by driver)

    // Transmit command and receive data
    err_code = nrf_drv_spi_transfer(&spi, tx_buf, 1, rx_buf, SPI_RX_LEN);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SPI Response:");
    for (int i = 0; i < SPI_RX_LEN; ++i)
    {
        NRF_LOG_INFO("Byte %d: 0x%02X", i, rx_buf[i]);
    }
    NRF_LOG_FLUSH();
}

/**
 * @brief Transmit a 32-bit unsigned integer over SPI and receive a response.
 *
 * @param value 32-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_uint(uint32_t value)
{
    uint8_t recval = 0;
    uint8_t tx_buf[3] = {
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)
    };
    uint8_t rx_buf[3] = {0};

    // Always send 3 bytes and receive 3 bytes
    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_buf, 3, rx_buf, 3);
    HANDLE_ERROR(err_code);

    // Return the last received byte (typical for command-response SPI)
    recval = rx_buf[2];
    return recval;
}

/**
 * @brief Transmit a 16-bit unsigned short over SPI and receive a response.
 *
 * @param value 16-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_ushort(uint16_t value)
{
    uint8_t recval = 0;
    uint8_t tx_buf[2] = {
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    uint8_t rx_buf[2] = {0};

    // Send 2 bytes, receive 2 bytes
    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2);
    HANDLE_ERROR(err_code);

    recval = rx_buf[1]; // Response is the last received byte
    return recval;
}

/**
 * @brief Transmit an 8-bit unsigned char over SPI and receive a response.
 *
 * @param value 8-bit value to transmit.
 * @return      The received byte from the SPI slave (typically the last byte).
 */
uint8_t spi_transmit_uchar(uint8_t value)
{
    uint8_t recval = 0;
    uint8_t tx_buf[2] = {value, 0}; // Send command + dummy byte
    uint8_t rx_buf[2] = {0};        // Receive: echo + response

    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2);
    HANDLE_ERROR(err_code);

    recval = rx_buf[1];  // Response is in the second byte
    return recval;
}
// End of file