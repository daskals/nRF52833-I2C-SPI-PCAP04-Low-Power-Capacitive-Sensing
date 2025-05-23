#ifndef PCAP_DEFINES_H
#define PCAP_DEFINES_H

#include <stdint.h>
#include <math.h>

// Measurement mode definitions
// Uncomment the desired measurement mode
// #define PCAP_MEASUREMENT_MODE_STANDARD
#define PCAP_MEASUREMENT_MODE_STANDARD_FLOATING
// #define PCAP_MEASUREMENT_MODE_STANDARD_DIFFERENTIAL
// #define PCAP_MEASUREMENT_MODE_HUMIDITY
// #define PCAP_MEASUREMENT_MODE_PRESSURE

// Reference capacitor selection
// Uncomment the desired reference capacitor
//#define PCAP_REFERENCE_CAP_EXTERNAL
#define PCAP_REFERENCE_CAP_INTERNAL

// Command definitions for PCap04
#define WR_NVRAM 0x28       // Write to NVRAM ('b:10 1000)
#define RD_NVRAM 0x08       // Read from NVRAM ('b:00 1000)
#define WR_CONFIG 0x028F    // Write configuration ('b:10 1000 1111)
#define RD_CONFIG 0x008F    // Read configuration ('b:00 1000 1111)
#define RD_RESULT 0x01      // Read result ('b:01)
#define POR_RESET 0x88      // Power-on reset ('b:1000 1000)
#define INITIALIZE_OP 0x8A  // Initialize operation ('b:1000 1010)
#define CDC_START 0x8C      // Start CDC measurement ('b:1000 1100)
#define RDC_START 0x8E      // Start RDC measurement ('b:1000 1110)
#define DSP_TRIG 0x8D       // Trigger DSP ('b:1000 1101)
#define NV_STORE 0x96       // Store to NVRAM ('b:1001 0110)
#define NV_RECALL 0x99      // Recall from NVRAM ('b:1001 1001)
#define NV_ERASE 0x9C       // Erase NVRAM ('b:1001 1100)
#define TEST_READ_LOW 0x11  // Test read low ('b:1001 1100)
#define TEST_READ_HIGH 0x7E // Test read high ('b:1001 1100)

// Utility macros
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0')

#define GET_NUMBER_OF_DIGITS(i) \
    (i > 0 ? (int) log10 ((double) i) + 1 : 1)

// Result and status sizes
#define PCAP_RESULT_REG_SIZE 4  // Each result register contains 4 bytes
#define PCAP_RESULTS_SIZE 32
#define PCAP_STATUS_SIZE 3

// NVRAM configuration
#define PCAP_NVRAM_RUNBIT_INDEX 1007
#define PCAP_NVRAM_FW_SIZE 704
#define PCAP_NVRAM_FW_CAL0_SIZE 128
#define PCAP_NVRAM_FW_CAL1_SIZE 128
#define PCAP_NVRAM_CFG_SIZE 64
#define PCAP_NVRAM_SIZE (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE + PCAP_NVRAM_FW_CAL1_SIZE + PCAP_NVRAM_CFG_SIZE)

#define PCAP_NVRAM_MAX_INDEX_FW PCAP_NVRAM_FW_SIZE
#define PCAP_NVRAM_MAX_INDEX_FW_CAL0 (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE)
#define PCAP_NVRAM_MAX_INDEX_FW_CAL1 (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE + PCAP_NVRAM_FW_CAL1_SIZE)

#endif


