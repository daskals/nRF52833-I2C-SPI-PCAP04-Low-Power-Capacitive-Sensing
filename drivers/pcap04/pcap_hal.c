/**
 * @file pcap_hal.c
 * @brief Hardware abstraction layer for PCAP04 communication (I2C/SPI selectable).
 */

#include "pcap_hal.h"
#include "i2c_interface.h"
#include "nrf_gpio.h"
#include "spi_interface.h" // Include SPI support



static pcap_hal_iface_t s_iface = PCAP_HAL_IFACE_I2C;

void pcap_hal_select_interface(pcap_hal_iface_t iface)
{
    // Configure IIC_EN pin as output
    nrf_gpio_cfg_output(PCAP04_PIN_IIC_EN);
    if (iface == PCAP_HAL_IFACE_I2C) {
        nrf_gpio_pin_set(PCAP04_PIN_IIC_EN); // 1 = I2C
    } else {
        nrf_gpio_pin_clear(PCAP04_PIN_IIC_EN); // 0 = SPI
    }
}

void pcap_hal_init(pcap_hal_iface_t iface)
{
    pcap_hal_select_interface(iface);
    s_iface = iface;
    if (iface == PCAP_HAL_IFACE_I2C) {
        i2c_init();
    } else if (iface == PCAP_HAL_IFACE_SPI) {
        spi_init();
    }
}

uint8_t pcap_hal_transmit_uint(uint32_t value)
{
    if (s_iface == PCAP_HAL_IFACE_I2C) {
        return i2c_transmit_uint(value, ADDRESS_PCAP04);
    } else {
        return spi_transmit_uint(value);
    }
}


uint8_t pcap_hal_transmit_ushort(uint16_t value)
{
    if (s_iface == PCAP_HAL_IFACE_I2C) {
        return i2c_transmit_ushort(value, ADDRESS_PCAP04);
    } else {
        return spi_transmit_ushort(value);
    }
}

uint8_t pcap_hal_transmit_uchar(uint8_t value)
{
    if (s_iface == PCAP_HAL_IFACE_I2C) {
        return i2c_transmit_uchar(value, ADDRESS_PCAP04);
    } else {
        return spi_transmit_uchar(value);
    }
}
