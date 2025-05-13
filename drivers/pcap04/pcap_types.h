#ifndef PCAP_TYPES_H
#define PCAP_TYPES_H

#include <stdint.h>
#include "pcap_defines.h"


typedef enum {
    PCAP04_V0 = 0,
    PCAP04_V1
} pcap04_version_t;

typedef enum {
    CFG_REG = 0,
    READ_REG
} regtype_t;

typedef enum {
    STAND_ALONE = 0,
    PRE_CONFIGURED,
    PURE_SLAVE
} pcap_operation_modes_t;

typedef enum {
    STANDARD = 0,
    HUMIDITY,
    PRESSURE
} pcap_measurement_modes_t;

typedef enum {
    PCAP_I2C_MODE,
    PCAP_SPI_MODE
} pcap_serial_interface_t;

typedef union {
    uint8_t i2c_addr;
    struct {
        uint8_t wr : 1;
        uint8_t addr : 2;
        uint8_t fixed_addr : 5;
    } i2caddr;
} pcap_i2c_addr_t;

typedef union {
    uint32_t opcode;
    struct {
        uint32_t data : 8;
        uint32_t addr : 10;
        uint32_t op_code : 6;
    } nvram;
} pcap_opcode_nvram_t;

typedef union {
    uint32_t opcode;
    struct {
        uint32_t data : 8;
        uint32_t addr : 6;
        uint32_t op_code : 10;
    } config;
} pcap_opcode_config_t;

typedef union {
    uint16_t opcode;
    struct {
        uint16_t data : 8;
        uint16_t addr : 6;
        uint16_t op_code : 2;
    } result;
} pcap_opcode_result_t;

typedef union {
    uint8_t opcode;
    struct {
        uint8_t op_code : 8;
    } command;
} pcap_opcode_command_t;

typedef union {
    uint16_t opcode;
    struct {
        uint16_t fixed : 8;
        uint16_t op_code : 8;
    } testread;
} pcap_opcode_testread_t;

typedef struct {
    regtype_t reg_type;
    uint8_t reg_name[16];
    uint8_t addr;
    void* reg; // Generic pointer to accommodate various types
} pcap_reg_t;

typedef struct {
    uint8_t OLF_CTUNE : 2;
    uint8_t OLF_FTUNE : 4;
    uint8_t I2C_A : 2;
    uint8_t OX_RUN : 3;
    // bool OX_STOP;              // ams internal bits
    // bool OX_AUTOSTOP_DIS;      // ams internal bits
    bool OX_DIV4;
    bool OX_DIS;

    bool RDCHG_EXT_EN;
    bool RDCHG_INT_EN;
    uint8_t RDCHG_INT_SEL0 : 2;
    uint8_t RDCHG_INT_SEL1 : 2;

    bool RCHG_SEL;
    bool RDCHG_EXT_PERM;
    bool RDCHG_PERM_EN;
    uint8_t RDCHG_OPEN : 2;
    bool AUX_CINT;
    bool AUX_PD_DIS;

    bool C_FLOATING;
    bool C_DIFFERENTIAL;
    bool C_COMP_INT;
    bool C_COMP_EXT;
    bool C_REF_INT;

    bool C_DC_BALANCE;
    bool CY_PRE_LONG;
    bool CY_DIV4_DIS;
    bool CY_HFCLK_SEL;
    bool C_PORT_PAT;
    bool CY_PRE_MR1_SHORT;

    uint8_t C_PORT_EN : 6;

    uint16_t C_AVRG : 13;

    uint32_t CONV_TIME : 23;

    uint16_t DISCHARGE_TIME : 10;
    uint8_t C_TRIG_SEL : 3;
    uint8_t C_STARTONPIN : 2;

    uint16_t PRECHARGE_TIME : 10;
    uint8_t C_FAKE : 4;

    uint16_t FULLCHARGE_TIME : 10;
    uint8_t C_REF_SEL : 5;

    uint8_t C_G_EN : 6;
    bool C_G_OP_EXT;
    bool C_G_OP_RUN;

    uint8_t C_G_TIME : 4;
    uint8_t C_G_OP_ATTN : 2;
    uint8_t C_G_OP_VU : 2;

    uint8_t C_G_OP_TR : 3;
    bool R_CY;

    uint16_t R_TRIG_PREDIV : 10;
    uint8_t R_AVRG : 2;
    uint8_t R_TRIG_SEL : 3;

    uint8_t R_STARTONPIN : 2;
    bool R_FAKE;
    bool R_PORT_EN_IREF;
    bool R_PORT_EN_IMES;
    uint8_t R_PORT_EN : 2;

    // unsigned char TDC_MUPU_SPEED:2;      // Mandatory : 3
    // bool TDC_NOISE_DIS:1;                // Mandatory : 0
    // bool TDC_ALUPERMOPEN:1;              // Mandatory : 0
    // unsigned char TDC_CHAN_EN:2;         // Mandatory : 3

    // unsigned char TDC_MUPU_NO:6;         // Mandatory : 1

    // unsigned char TDC_NOISE_CY_DIS:1;    // Mandatory : 1
    // unsigned char TDC_QHA_SEL:6;         // Mandatory : 20

    bool PG0xPG2;
    bool PG1xPG3;
    uint8_t DSP_SPEED : 2;
    uint8_t DSP_MOFLO_EN : 2;

    uint8_t WD_DIS;

    uint8_t DSP_FF_IN : 4;
    uint8_t DSP_STARTONPIN : 4;

    uint8_t DSP_START_EN : 3;
    bool PG4_INTN_EN;
    bool PG5_INTN_EN;

    uint8_t PI0_CLK_SEL : 3;
    bool PI0_PDM_SEL;
    uint8_t PI0_RES : 2;
    bool PI0_TOGGLE_EN;
    bool PI1_TOGGLE_EN;

    uint8_t PI1_CLK_SEL : 3;
    uint8_t PI1_PDM_SEL : 1;
    uint8_t PI1_RES : 2;

    uint8_t PG_PU : 4;
    uint8_t PG_DIR_IN : 4;

    // Mandatory
    bool AUTOSTART;
    bool BG_PERM;
    bool DSP_TRIG_BG;
    bool INT_TRIG_BG;
    uint8_t CDC_GAIN_CORR;
    uint8_t BG_TIME : 8;
    uint8_t PULSE_SEL0 : 4;
    uint8_t PULSE_SEL1 : 4;
    uint8_t C_SENSE_SEL;
    uint8_t R_SENSE_SEL;

    bool C_MEDIAN_EN;
    bool R_MEDIAN_EN;
    bool HS_MODE_SEL;
    bool EN_ASYNC_READ;
    bool ALARM0_SELECT;
    bool ALARM1_SELECT;
    bool RUNBIT;
    uint8_t MEM_LOCK : 4;
    uint16_t SERIAL_NUMBER;
    uint8_t MEM_CTRL;
} pcap_config_t;


struct pcap_results_t
{

    float C0_over_CREF;
    float C1_over_CREF;
    float C2_over_CREF;
    float C3_over_CREF;
    float C4_over_CREF;
    float C5_over_CREF;
    float PT1_over_PTREF;
    float PTInternal_over_PTREF;
};

typedef struct {
    bool RUNBIT;
    bool CDC_ACTIVE;
    bool RDC_READY;
    bool AUTOBOOT_BUSY;
    bool POR_CDC_DSP_COLL;
    bool POR_FLAG_CONFIG;
    bool POR_FLAG_WDOG;

    uint8_t NC : 4;
    bool RDC_ERR;
    bool MUP_ERR;
    bool ERR_OVERFL;
    bool COMB_ERR;

    bool C_PORT_ERR_INT;
    bool C_PORT_ERR0;
    bool C_PORT_ERR1;
    bool C_PORT_ERR2;
    bool C_PORT_ERR3;
    bool C_PORT_ERR4;
    bool C_PORT_ERR5;
} pcap_status_t;

typedef struct {
    uint16_t nvram_loc;
    size_t size;
    uint8_t *data;
} pcap_data_vector_t;

#endif
