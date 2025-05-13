// -----------------------------------------------------------------------------
// License and File Description
// -----------------------------------------------------------------------------
/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** 
 * Perhipheral: nRF52 SAADC
 * Compatibility: nRF52832 rev 1/nRF52840 Eng A, nRF5 SDK 13.0.0
 * Softdevice used: No softdevice
 *

 * This example enables the RTC timer to periodically trigger SAADC sampling. RTC is chosen here instead of 
 * TIMER because it is low power. The example samples on a single input pin, the AIN0, which maps to physical pin P0.02 on the nRF52832 IC.
 * This SAADC example shows the following features:
 * - Low Power -> Enabled with initializing SAADC when sampling and uninitializing when sampling is complete.
 *                Low power can only be obtained when UART_PRINTING_ENABLED is not defined and
 *                SAADC_SAMPLES_IN_BUFFER is 1
 * - Oversampling -> This reduces SAADC noise level, especially for higher SAADC resolutions, see
 *                   https://devzone.nordicsemi.com/question/83938/nrf52832-saadc-sampling/?comment=84340#comment-84340
 *                   Configured with the SAADC_OVERSAMPLE constant.
 * - BURST mode -> Burst mode can be combined with oversampling, which makes the SAADC sample all oversamples as fast
 *                 as it can with one SAMPLE task trigger. Set the SAADC_BURST_MODE constant to enable BURST mode.
 * - Offset Calibration -> SAADC needs to be occasionally calibrated. The desired calibration interval depends on the
 *                         expected temperature change rate, see the nRF52832 PS for more information. The
 *                         calibration interval can be adjusted with configuring the SAADC_CALIBRATION_INTERVAL
 *                         constant.
 * The SAADC sample result is printed on UART. To see the UART output, a UART terminal (e.g. Realterm) can be configured on 
 * your PC with the UART configuration set in the uart_config function, which is also described in the saadc example documentation -> 
 * http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v11.0.0/nrf_dev_saadc_example.html?cp=5_0_0_4_5_24
 *

 * Indicators on the nRF52-DK board:
 * LED1: SAADC Sampling triggered 
 * LED2: SAADC sampling buffer full and event received
 * LED3: SAADC Offset calibration complete
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"

#include "nrf_drv_spi.h"
#include "i2c_interface.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "pcap_hal.h"
#include "pcap_types.h"
#include "pcap04.h"
//#include "pcap_opcodes.h"
//#include "pcap_registers.h"
//#include "pcap_firmware.h"

//#define NRF_LOG_ENABLED 1
// -----------------------------------------------------------------------------
// Defines and Constants
// -----------------------------------------------------------------------------
#define LED_FUNCTIONALITY_ENABLED 1 // Set to 1 to enable LED functionality, 0 to disable
#define CALIBRATION_FUNCTIONALITY_ENABLED 1 // Set to 1 to enable calibration functionality, 0 to disable
#define SAADC_SAMPLE_INTERVAL_SEC 1 // Interval in seconds at which RTC times out and triggers SAADC sample task
#define SAADC_SAMPLE_INTERVAL_MS (SAADC_SAMPLE_INTERVAL_SEC * 1000) // Convert seconds to milliseconds
#define RTC_FREQUENCY 32                          //Determines the RTC frequency and prescaler
#define RTC_CC_VALUE 8                            //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
#define SAADC_CALIBRATION_INTERVAL 20              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 0                        //Set to 1 to enable BURST mode, otherwise set to 0.
#define SAADC_RESOLUTION_BITS   12                // ADC resolution in bits
#define SAADC_REFERENCE_VOLTAGE 0.6f              // Internal reference voltage in volts
#define SAADC_GAIN              NRF_SAADC_GAIN1_6 // Gain setting for ADC
#define VOLTAGE_DIVIDER_RATIO   6.0f              // Voltage divider ratio corresponding to the gain
#define ADC_IN_PIN NRF_GPIO_PIN_MAP(0, 2) // Use P0.05 for MUX control pin S2

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static uint32_t                rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS*1000, RTC_FREQUENCY);
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;      
static bool                    saadc_initialized = false; // Track if SAADC is initialized
static bool                    pcap_initialized = false; // Track if PCAP04 is initialized

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
static float calculate_voltage_mV(nrf_saadc_value_t sample);
static ret_code_t perform_saadc_sample(nrf_saadc_value_t *sample);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
static void sample_and_print_saadc(void);
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void lfclk_config(void);
static void rtc_config(void);
void gpio_init(void);
void saadc_init(void);
void init_log(void);

// -----------------------------------------------------------------------------
// Function Implementations
// -----------------------------------------------------------------------------

/**
 * @brief Calculate the voltage in millivolts from an SAADC sample.
 * @param[in] sample The raw SAADC sample value.
 * @return float The calculated voltage in millivolts.
 */
static float calculate_voltage_mV(nrf_saadc_value_t sample)
{
    return ((float)sample / ((1 << SAADC_RESOLUTION_BITS) - 1)) * SAADC_REFERENCE_VOLTAGE * 1000 * VOLTAGE_DIVIDER_RATIO;
}

/**
 * @brief Perform a single SAADC sample conversion and return the result.
 * @param[out] sample Pointer to store the sampled value.
 * @return ret_code_t Error code indicating success or failure.
 */
static ret_code_t perform_saadc_sample(nrf_saadc_value_t *sample)
{
    ret_code_t err_code = nrf_drv_saadc_sample_convert(0, sample);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("SAADC sample conversion failed: %d", err_code);
    }
    
    return err_code;
}

/**
 * @brief SAADC event callback handler.
 * Handles calibration complete event.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");
    }
}

/**
 * @brief Print the SAADC sample result.
 */
static void sample_and_print_saadc(void)
{
    if (!saadc_initialized) {
        NRF_LOG_WARNING("SAADC not initialized. Skipping sample.");
        return;
    }
    nrf_saadc_value_t sample = 0;
    ret_code_t err_code = perform_saadc_sample(&sample);
    if (err_code == NRF_SUCCESS) 
    {
        float voltage_mV = calculate_voltage_mV(sample);
        NRF_LOG_INFO("Sample: %d (%d mV)\r\n", sample, (int)voltage_mV);
    }
    #if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_0_MASK); // Toggle LED1 to indicate SAADC sampling start
    #endif
    m_adc_evt_counter++;
}

/**
 * @brief RTC interrupt handler. Triggers ADC sampling and resets RTC compare.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if (saadc_initialized) {
            sample_and_print_saadc();
        } else {
            NRF_LOG_WARNING("SAADC not initialized. Skipping RTC sample.");
        }

        if (pcap_initialized) {
            log_pcap_measurements();
    #if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_1_MASK); // Toggle LED1 to indicate SAADC sampling start
    #endif
        } else {
            NRF_LOG_WARNING("PCAP04 not initialized. Skipping measurements.");
        }

        err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);
    }
}

/**
 * @brief Configure low frequency clock source.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief Configure and initialize RTC instance.
 */
static void rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);                //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);                    //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                                   //Enable RTC
}

/**
 * @brief Initialize GPIOs for ADC input.
 */
void gpio_init(void)
{
    // Configure the SAADC input pin (e.g., AIN0) as an analog input
    nrf_gpio_cfg_default(ADC_IN_PIN); // Example: AIN0 corresponds to P0.02
}

/**
 * @brief Initialize and configure SAADC peripheral.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;

	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    if(SAADC_BURST_MODE)
    {
        channel_config.burst = NRF_SAADC_BURST_ENABLED;                                   //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);

    // nrf_drv_saadc_abort();
    NRF_LOG_INFO("SAADC calibration starting...");    //Print on UART
    while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task

    saadc_initialized = true; // Mark SAADC as initialized
}

/**
 * @brief Initializes the logging module.
 */
void init_log(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT(); // Initialize RTT backend
    NRF_LOG_DEBUG("Logging initialized\r\n");
}



/**
 * @brief Main application entry point.
 */
int main(void)
{


#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK);                       //Configure all leds
    LEDS_OFF(LEDS_MASK);                             //Turn off all leds
#endif

    bool pcap_ok=false;
    NRF_POWER->DCDCEN = 1;                           //Enabling the DCDC converter for lower current consumption
    NRF_LOG_INFO("Main Inits.");	
    // Initialize the logging module
    init_log();
    gpio_init();
    lfclk_config();                                  //Configure low frequency 32kHz clock
    
    pcap_hal_init(PCAP_HAL_IFACE_I2C);
    pcap_ok=test_pcap_connection();
    if (pcap_ok)
    {
    init_nvram(PCAP04_V1,STANDARD);
    pcap_initialized=init_pcap04();
    }
    saadc_init();  //Initialize and start SAADC
    // Only now start RTC, after all inits are done!
    rtc_config();  // Configure RTC and enable it
    
    while (1)
    { 
        while(NRF_LOG_PROCESS() != NRF_SUCCESS); 
        nrf_pwr_mgmt_run();
        
    }
}

// -----------------------------------------------------------------------------
// End of File
// -----------------------------------------------------------------------------