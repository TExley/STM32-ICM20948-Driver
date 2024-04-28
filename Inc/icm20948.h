/*
 * icm20948.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Lain (trevynexley@gmail.com)
 */

#pragma once

#include "stm32l4xx_hal.h"


#define BITS_PER_BYTE 8

/* ICM20948 CONSTANTS START */

/* ICM20948 SLAVE ADDRESSES START */
static const uint16_t ICM20948_ADDR_L = 0xD0; // SDO(AD0) pin is low
static const uint16_t ICM20948_ADDR_H = 0xD2; // SDO(AD0) pin is high
/* ICM20948 SLAVE ADDRESSES END */


/* ICM20948 REGISTER DEFINITION START */
typedef struct reg {
	const char* name;
	const uint16_t address;
} reg;
/* ICM20948 REGISTER DEFINITION END */


/* ICM20948 USER UBANK SELECT REGISTER START */
/* BIT 	| NAME 		| DESC
 * 7:6 	| -			| reserved
 * 5:4	| USER_BANK	| value of user bank
 * 3:0	| -			| reserved
 * Reset Value: 0b01000000
 */
static const reg REG_UBANK_SEL = { "UBANK_SEL", 0x7F}; // A RW register but cannot be used in reg_RW or reg_R functions
static const uint8_t REG_UBANK_SEL_RESERVED_MASK = 0b11001111;
typedef enum user_bank {UBANK_0 = 0x0, UBANK_1 = 0x10, UBANK_2 = 0x20, UBANK_3 = 0x30} user_bank;
/* ICM20948 USER UBANK SELECT REGISTER END */


/* ICM20948 REGISTER DEFINITION CONTINUED START */
typedef struct reg_R {
	const char* name;
	const uint16_t address;
	const user_bank bank;
} reg_R;

typedef struct reg_RW {
	const char* name;
	const uint16_t address;
	const user_bank bank;
	const uint8_t reserved_mask;
} reg_RW;
/* ICM20948 REGISTER DEFINITION CONTINUED END */


/* ICM20948 USER UBANK 0 REGISTERS START */
static const reg_R REG_WHO_AM_I = { "WHO_AM_I", 0x00, UBANK_0 };
static const uint8_t REG_WHO_AM_I_VALUE = 0xEA; // Read Only Value

/* BIT 	| NAME 			| DESC
 * 7 	| DMP_EN		| 1 enables DMP features
 * 6	| FIFO_EN		| 1 enables FIFO operation mode
 * 5	| I2C_MST_EN	| 1 enables I2C master I/F module. isolates ES_DA/ES_SCL from SDA/SCL
 * 4	| I2C_IF_DIS	| 1 reset I2C and enter SPI mode only
 * 3	| DMP_RST		| 1 asynchronous reset of DMP module, bit clears after one clock cycle
 * 2	| SRAM_RST		| 1 asynchronous reset of SRAM module, bit clears after one clock cycle
 * 1	| I2C_MST_RST	| 1 asynchronous reset of I2C module, bit clears after one clock cycle
 * 0	| -				| reserved
 * Reset Value: 0
 */
static const reg_RW REG_USER_CTRL = { "USER_CTRL", 0x03, UBANK_0, 0 };
typedef enum user_ctrl_opts {
	I2C_MST_RST = 0b00000010,
	SRAM_RST 	= 0b00000100,
	DMP_RST 	= 0b00001000,
	I2C_IF_DIS 	= 0b00010000,
	I2C_MST_EN 	= 0b00100000,
	FIFO_EN 	= 0b01000000,
	DMP_EN 		= 0b10000000
} user_ctrl_opts;

/* BIT 	| NAME 			| DESC
 * 7 	| -				| reserved
 * 6	| I2C_MST_CYCLE	| (See p68 for I2C_MST_ODR_CONFIG)
 * 5	| ACCEL_CYCLE	| 1 sets duty cycled mode
 * 4	| GYRO_CYCLE	| 1 sets duty cycled mode
 * 3:0	| -				| reserved
 * Reset Value: 0b01000000
 */
static const reg_RW REG_LP_CONFIG = { "LP_CONFIG", 0x05, UBANK_0, 0b10001111 };
typedef enum lp_config_opts {
	GYRO_CYCLE		= 0b00010000,
	ACCEL_CYCLE 	= 0b00100000,
	I2C_MST_CYCLE 	= 0b01000000
} lp_config_opts;

/* BIT 	| NAME 			| DESC
 * 7 	| DEVICE_RESET	| 1 resets all registers
 * 6	| Sleep			| 1 sets sleep mode
 * 5	| LP_EN			| 1 sets low power mode
 * 4	| -				| reserved
 * 3	| TEMP_DIS		| 1 disables temp sensor
 * 2:0	| CLKSEL		| 1-5 auto clock (DS p37)
 * Reset Value: 0b01000001
 */
static const reg_RW REG_PWR_MGMT_1 = { "PWR_MGMT_1", 0x06, UBANK_0, 0b00010000 };
typedef enum pwr_mgmt_1_opts {
	CLKSEL_AUTO 	= 0b00000001,
	CLKSEL_INTERNAL	= 0b00000110,
	CLKSEL_STOP		= 0b00000111,
	TEMP_DIS 		= 0b00001000,
	LP_EN			= 0b00100000,
	SLEEP 			= 0b01000000,
	DEVICE_RESET 	= 0b10000000
} pwr_mgmt_1_opts;

static const reg_R REG_ACCEL_XOUT_H = { "ACCEL_XOUT_H", 0x2D, UBANK_0 };
static const reg_R REG_ACCEL_XOUT_L = { "ACCEL_XOUT_L", 0x2E, UBANK_0 };
static const reg_R REG_ACCEL_YOUT_H = { "ACCEL_YOUT_H", 0x2F, UBANK_0 };
static const reg_R REG_ACCEL_YOUT_L = { "ACCEL_YOUT_L", 0x30, UBANK_0 };
static const reg_R REG_ACCEL_ZOUT_H = { "ACCEL_ZOUT_H", 0x31, UBANK_0 };
static const reg_R REG_ACCEL_ZOUT_L = { "ACCEL_ZOUT_L", 0x32, UBANK_0 };

static const reg_R REG_GYRO_XOUT_H = { "GYRO_XOUT_H", 0x33, UBANK_0 };
static const reg_R REG_GYRO_XOUT_L = { "GYRO_XOUT_L", 0x34, UBANK_0 };
static const reg_R REG_GYRO_YOUT_H = { "GYRO_YOUT_H", 0x35, UBANK_0 };
static const reg_R REG_GYRO_YOUT_L = { "GYRO_YOUT_L", 0x36, UBANK_0 };
static const reg_R REG_GYRO_ZOUT_H = { "GYRO_ZOUT_H", 0x37, UBANK_0 };
static const reg_R REG_GYRO_ZOUT_L = { "GYRO_ZOUT_L", 0x38, UBANK_0 };

static const reg_R REG_TEMP_OUT_H = { "TEMP_OUT_H", 0x33, UBANK_0 };
static const reg_R REG_TEMP_OUT_L = { "TEMP_OUT_L", 0x34, UBANK_0 };

static const reg_R REG_EXT_SLV_SENS_DATA_00 = { "EXT_SLV_SENS_DATA_00", 0x3B, UBANK_0 };

/* BIT 	| NAME 			| DESC
 * 7:4 	| -				| reserved
 * 3 	| SLV_3_FIFO_EN	| write associated EXT_SENS_DATA registers to FIFO at sample rate
 * 2	| SLV_2_FIFO_EN	| write associated EXT_SENS_DATA registers to FIFO at sample rate
 * 1 	| SLV_1_FIFO_EN	| write associated EXT_SENS_DATA registers to FIFO at sample rate
 * 0 	| SLV_0_FIFO_EN	| write associated EXT_SENS_DATA registers to FIFO at sample rate
 */
static const reg_RW REG_FIFO_EN_1 = { "FIFO_EN_1", 0x66, UBANK_0, 0b11110000};
typedef enum fifo_en_1_opts {
	SLV_0_FIFO_EN 	= 0b00000001,
	SLV_1_FIFO_EN	= 0b00000010,
	SLV_2_FIFO_EN	= 0b00000100,
	SLV_3_FIFO_EN 	= 0b00001000
} fifo_en_1_opts;

/* ICM20948 USER UBANK 0 REGISTERS END */


/* ICM20948 USER UBANK 1 REGISTERS START */
/* BIT 	| NAME 			| DESC
 * 7:0 	| XA_OFFS[14:7]	| Upper bits of X accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_XA_OFFS_H = { "XA_OFFS_H", 0x14, UBANK_1, 0 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| XA_OFFS[6:0]	| Lower bits of X accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_XA_OFFS_L = { "XA_OFFS_L", 0x15, UBANK_1, 0b00000001 };

/* BIT 	| NAME 			| DESC
 * 7:0 	| YA_OFFS[14:7]	| Upper bits of Y accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_YA_OFFS_H = { "YA_OFFS_H", 0x17, UBANK_1, 0 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| YA_OFFS[6:0]	| Lower bits of Y accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_YA_OFFS_L = { "YA_OFFS_L", 0x18, UBANK_1, 0b00000001 };

/* BIT 	| NAME 			| DESC
 * 7:0 	| ZA_OFFS[14:7]	| Upper bits of Z accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_ZA_OFFS_H = { "ZA_OFFS_H", 0x1A, UBANK_1, 0 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| ZA_OFFS[6:0]	| Lower bits of Z accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_ZA_OFFS_L = { "ZA_OFFS_L", 0x1B, UBANK_1, 0b00000001 };
/* ICM20948 USER UBANK 1 REGISTERS END */


/* ICM20948 USER UBANK 2 REGISTERS START */
/* BIT 	| NAME 				| DESC
 * 7:0 	| GYRO_SMPLRT_DIV	| sample rate dividor 1.1kHz/(1 + GYRO_SMPLRT_DIV)
 * Reset Value: 0
 */
static const reg_RW REG_GYRO_SMPLRT_DIV = { "GYRO_SMPLRT_DIV", 0x00, UBANK_2, 0 };

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5:3	| GYRO_DLPFCFG	| low pass filter config (DS p60)
 * 2:1	| GYRO_FS_SEL	| full scale select (DS p59)
 * 0	| GYRO_FCHOICE	| 1 enable DLPF
 * Reset Value: 0
 */
static const reg_RW REG_GYRO_CONFIG_1 = { "GYRO_CONFIG_1", 0x01, UBANK_2, 0b11000000 };
typedef enum gyro_config_1_opts {
	GYRO_FCHOICE 		= 0b00000001,
	// GYRO_FS_SEL_250 by default
	GYRO_FS_SEL_500 	= 0b00000010,
	GYRO_FS_SEL_1000 	= 0b00000100,
	GYRO_FS_SEL_2000 	= 0b00000110,
	// GYRO_DLPFCFG_0 by default
	GYRO_DLPFCFG_1 		= 0b00001000,
	GYRO_DLPFCFG_2 		= 0b00010000,
	GYRO_DLPFCFG_3 		= 0b00011000,
	GYRO_DLPFCFG_4 		= 0b00100000,
	GYRO_DLPFCFG_5 		= 0b00101000,
	GYRO_DLPFCFG_6 		= 0b00110000,
	GYRO_DLPFCFG_7 		= 0b00111000
} gyro_config_1_opts;

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5	| XGYRO_CTEN	| 1 enable x self test
 * 4	| YGYRO_CTEN	| 1 enable y self test
 * 3	| ZGYRO_CTEN	| 1 enable z self test
 * 2:0	| GYRO_AVGCFG	| average lp-mode filter config (DS p60)
 * Reset Value: 0
 */
static const reg_RW REG_GYRO_CONFIG_2 = { "GYRO_CONFIG_2", 0x02, UBANK_2, 0b11000000 };
typedef enum gyro_config_2_opts {
	// GYRO_AVGCFG_1x by default
	GYRO_AVGCFG_2x 		= 0b00000001,
	GYRO_AVGCFG_4x 		= 0b00000010,
	GYRO_AVGCFG_8x 		= 0b00000011,
	GYRO_AVGCFG_16x 	= 0b00000100,
	GYRO_AVGCFG_32x 	= 0b00000101,
	GYRO_AVGCFG_64x 	= 0b00000110,
	GYRO_AVGCFG_128x 	= 0b00000111,
	ZGYRO_CTEN 			= 0b00001000,
	YGYRO_CTEN 			= 0b00010000,
	XGYRO_CTEN 			= 0b00100000
} gyro_config_2_opts;

/* BIT 	| NAME 					| DESC
 * 7:0 	| XG_OFFS_USER[15:8]	| Upper bits of X gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_XG_OFFS_USRH = { "XG_OFFS_USRH", 0x03, UBANK_2, 0 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| XG_OFFS_USER[7:0]	| Lower bits of X gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_XG_OFFS_USRL = { "XG_OFFS_USRL", 0x04, UBANK_2, 0 };

/* BIT 	| NAME 					| DESC
 * 7:0 	| YG_OFFS_USER[15:8]	| Upper bits of Y gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_YG_OFFS_USRH = { "YG_OFFS_USRH", 0x05, UBANK_2, 0 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| YG_OFFS_USER[7:0]	| Lower bits of Y gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_YG_OFFS_USRL = { "YG_OFFS_USRL", 0x06, UBANK_2, 0 };

/* BIT 	| NAME 					| DESC
 * 7:0 	| ZG_OFFS_USER[15:8]	| Upper bits of Z gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_ZG_OFFS_USRH = { "ZG_OFFS_USRH", 0x07, UBANK_2, 0 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| ZG_OFFS_USER[7:0]	| Lower bits of Z gyro offset cancellation
 * Reset Value: 0
 */
static const reg_RW REG_ZG_OFFS_USRL = { "ZG_OFFS_USRL", 0x08, UBANK_2, 0 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| -				| reserved
 * 0	| ODR_ALIGN_EN	| sensor start time alignment if and smprt set
 * Reset Value: 0
 */
static const reg_RW REG_ODR_ALIGN_EN = { "ODR_ALIGN_EN", 0x09, UBANK_2, 0b11111110 };
typedef enum odr_align_opts {
	ODR_ALIGN_EN = 0b00000001
} odr_align_opts;

/* BIT 	| NAME 				| DESC
 * 7:4 	| -					| reserved
 * 3:0	| ACCEL_SMPLRT_DIV	| MSB for ACCEL_SMPLRT_DIV
 * Reset Value: 0
 */
static const reg_RW REG_ACCEL_SMPLRT_DIV_1 = { "ACCEL_SMPLRT_DIV_1", 0x10, UBANK_2, 0b11110000 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| ACCEL_SMPLRT_DIV	| LSB for sample rate dividor 1.125kHz/(1 + ACCEL_SMPLRT_DIV[11:0])
 * Reset Value: 0
 */
static const reg_RW REG_ACCEL_SMPLRT_DIV_2 = { "ACCEL_SMPLRT_DIV_2", 0x11, UBANK_2, 0 };

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5:3	| ACCEL_DLPFCFG	| low pass filter config (DS p64)
 * 2:1	| ACCEL_FS_SEL	| full scale select (DS p64)
 * 0	| ACCEL_FCHOICE	| 1 enable DLPF
 * Reset Value: 0b00000001
 */
static const reg_RW REG_ACCEL_CONFIG_1 = { "ACCEL_CONFIG_1", 0x14, UBANK_2, 0b11000000 };
typedef enum accel_config_1_opts {
	ACCEL_FCHOICE 		= 0b00000001,
	// ACCEL_FS_SEL_2g by default
	ACCEL_FS_SEL_4g 	= 0b00000010,
	ACCEL_FS_SEL_8g 	= 0b00000100,
	ACCEL_FS_SEL_16g 	= 0b00000110,
	// ACCEL_DLPFCFG_0 by default
	ACCEL_DLPFCFG_1 	= 0b00001000,
	ACCEL_DLPFCFG_2 	= 0b00010000,
	ACCEL_DLPFCFG_3 	= 0b00011000,
	ACCEL_DLPFCFG_4		= 0b00100000,
	ACCEL_DLPFCFG_5		= 0b00101000,
	ACCEL_DLPFCFG_6		= 0b00110000,
	ACCEL_DLPFCFG_7		= 0b00111000
} accel_config_1_opts;

/* BIT 	| NAME 			| DESC
 * 7:5 	| -				| reserved
 * 4	| AX_ST_EN_REG	| 1 enable x self test
 * 3	| AY_ST_EN_REG	| 1 enable y self test
 * 2	| AZ_ST_EN_REG	| 1 enable z self test
 * 1:0	| DEC3_CFG		| controls samples averaged in devimator (DS p65)
 * Reset Value: 0
 */
static const reg_RW REG_ACCEL_CONFIG_2 = { "ACCEL_CONFIG_2", 0x15, UBANK_2, 0b11100000 };
typedef enum accel_config_2_opts {
	// DEC3_CFG_1 or DEC3_CFG_4 by default (DS p65)
	DEC3_CFG_8		= 0b00000001,
	DEC3_CFG_16		= 0b00000010,
	DEC3_CFG_32 	= 0b00000011,
	AZ_ST_EN_REG 	= 0b00000100,
	AY_ST_EN_REG 	= 0b00001000,
	AX_ST_EN_REG 	= 0b00010000
} accel_config_2_opts;

/* BIT 	| NAME 			| DESC
 * 7:3 	| -				| unused (not labeled reserved)
 * 2:0	| TEMP_DLPFCFG	| low pass filter config (DS p67)
 * Reset Value: 0
 */
static const reg_RW REG_TEMP_CONFIG = { "TEMP_CONFIG", 0x53, UBANK_2, 0 };
typedef enum temp_config_opts {
	// TEMP_DLPFCFG_0 by default
	TEMP_DLPFCFG_1 	= 0b00000001,
	TEMP_DLPFCFG_2 	= 0b00000010,
	TEMP_DLPFCFG_3 	= 0b00000011,
	TEMP_DLPFCFG_4	= 0b00000100,
	TEMP_DLPFCFG_5	= 0b00000101,
	TEMP_DLPFCFG_6	= 0b00000110,
	TEMP_DLPFCFG_7	= 0b00000111
} temp_config_opts;
/* ICM20948 USER UBANK 2 REGISTERS END */


/* ICM20948 USER UBANK 3 REGISTERS START */
/* BIT 	| NAME 			| DESC
 * 7 	| MULT_MST_EN	| enables multi-master capability
 * 6:5	| -				| reserved
 * 4	| I2C_MST_P_NSR	| 1/0 sets transition btwn reads to stop/restart
 * 3:0	| I2C_MST_CLK	| sets I2C master clock frequency (DS p68)
 * Reset Value: 0
 */
static const reg_RW REG_I2C_MST_CTRL = { "I2C_MST_CTRL", 0x01, UBANK_3, 0b01100000 };
typedef enum i2c_mst_ctrl_opts {
	// I2C_MST_CLK_0 by default (DS p81)
	// I2C_MST_CLK_1 is undefined
	// I2C_MST_CLK_2 == I2C_MST_CLK_0
	I2C_MST_CLK_3 	= 0b00000011,
	I2C_MST_CLK_4 	= 0b00000100,
	// I2C_MST_CLK_5 == I2C_MST_CLK_3
	I2C_MST_CLK_6  	= 0b00000110,
	I2C_MST_CLK_7  	= 0b00000111,
	I2C_MST_CLK_8  	= 0b00001000,
	// I2C_MST_CLK_9 == I2C_MST_CLK_3
	I2C_MST_CLK_10  = 0b00001010,
	// I2C_MST_CLK_11 == I2C_MST_CLK_10
	I2C_MST_CLK_12  = 0b00001100,
	// I2C_MST_CLK_13 == I2C_MST_CLK_3
	// I2C_MST_CLK_14 == I2C_MST_CLK_7
	// I2C_MST_CLK_15 == I2C_MST_CLK_7
	I2C_MST_P_NSR 	= 0b00010000,
	MULT_MST_EN 	= 0b10000000
} i2c_mst_ctrl_opts;

/* BIT 	| NAME 			| DESC
 * 7 	| I2C_SLV0_RNW	| 1/0 – Transfer is a read/write
 * 6:0	| I2C_ID_0		| physical address of I2C slave 0
 * Reset Value: 0
 */
static const reg_RW REG_I2C_SLV0_ADDR = { "I2C_SLV0_ADDR", 0x03, UBANK_3, 0 };
typedef enum i2c_slv_addr_opts { // same value for all I2C_SLVX_ADDR
	I2C_SLV_RNW = 0b10000000
} i2c_slv_addr_opts;

/* BIT 	| NAME 			| DESC
 * 7:0 	| I2C_SLV0_REG	| register address from where to begin data transfer
 * Reset Value: 0
 */
static const reg_RW REG_I2C_SLV0_REG = { "I2C_SLV0_REG", 0x04, UBANK_3, 0 };

/* BIT 	| NAME 				| DESC
 * 7 	| I2C_SLV0_EN		| 1 reads data at the sample rate and stores it at the first available EXT_SENS_DATA register
 * 6 	| I2C_SLV0_BYTE_SW	| 1 enables swapping bytes when reading both the low and high byte of a word (DS p70)
 * 5 	| I2C_SLV0_REG_DIS	| 1 the transaction does not write a register value, it will only read data, or write data
 * 4 	| I2C_SLV0_GRP		| sets wether groups of bytes swapped end in odd (0) or even (1) registers
 * 3:0 	| I2C_SLV0_LENG		| number of bytes to be read from I2C slave 0
 * Reset Value: 0
 */
static const reg_RW REG_I2C_SLV0_CTRL = { "I2C_SLV0_CTRL", 0x05, UBANK_3, 0 };
typedef enum i2c_slv_ctrl_opts { // same value for all I2C_SLVX_ADDR
	// I2C_SLV_LENG_0 by default
	I2C_SLV_LENG_1	 	= 0b00000001,
	I2C_SLV_LENG_2 		= 0b00000010,
	I2C_SLV_LENG_3 		= 0b00000011,
	I2C_SLV_LENG_4 		= 0b00000100,
	I2C_SLV_LENG_5 		= 0b00000101,
	I2C_SLV_LENG_6 		= 0b00000110,
	I2C_SLV_LENG_7 		= 0b00000111,
	I2C_SLV_LENG_8 		= 0b00001000,
	I2C_SLV_LENG_9 		= 0b00001001,
	I2C_SLV_LENG_10 	= 0b00001010,
	I2C_SLV_LENG_11 	= 0b00001011,
	I2C_SLV_LENG_12 	= 0b00001100,
	I2C_SLV_LENG_13 	= 0b00001101,
	I2C_SLV_LENG_14 	= 0b00001110,
	I2C_SLV_LENG_15 	= 0b00001111,
	I2C_SLV_GRP 		= 0b00010000,
	I2C_SLV_REG_DIS 	= 0b00100000,
	I2C_SLV_BYTE_SW 	= 0b01000000,
	I2C_SLV_EN 		= 0b10000000
} i2c_slv_ctrl_opts;

/* BIT 	| NAME 			| DESC
 * 7:0 	| I2C_SLV0_DO	| data out when slave 0 is set to write
 * Reset Value: 0
 */
static const reg_RW REG_I2C_SLV0_DO = { "I2C_SLV0_DO", 0x06, UBANK_3, 0 };

/* ICM20948 USER UBANK 3 REGISTERS END */


/* SENSOR SUPPORT STRUCTS START */
typedef struct int16_vector3 {
	int16_t x;
	int16_t y;
	int16_t z;
} int16_vector3;

typedef struct float_vector3 {
	float x;
	float y;
	float z;
} float_vector3;
/* SENSOR SUPPORT STRUCTS END */


/* DELAYS AND TIMEOUTS START */
// Start-up time for register read/write from power-up or reset
static const uint32_t STARTUP_DELAY = 100;

/* Start-up time for sensor output from sleep
 * ACC ~ 30ms
 * GYR ~ 35ms
 * MAG ~ 8ms
 */
static const uint32_t WAKE_DELAY = 45;

// Maximum time-out to wait for any sensor ACK
static const uint32_t MAXIMUM_ICM_TIMEOUT = 20;
/* DELAYS AND TIMEOUTS END */


/* ICM20948 ENUMS START */
typedef enum SDO_Pinouts {SDO_LOW, SDO_HIGH} SDO_Pinouts;
/* ICM20948 ENUMS END */
/* ICM20948 CONSTANTS END */


/* AK09916 CONSTANTS START */
static const uint8_t AK09916_ADDR = 0x18;

typedef enum CNTL2_Mode {
	POWER_DOWN 	   = 0,
	SINGLE_MEASURE = 0b000001,
	CONT_MEASURE_1 = 0b000010,
	CONT_MEASURE_2 = 0b000100,
	CONT_MEASURE_3 = 0b000110,
	CONT_MEASURE_4 = 0b010000,
	SELF_TEST 	   = 0b100000
} CNTL2_Mode;

typedef enum AK09916_Register {
	WIA = 0x01,
	ST1 = 0x10,
	HXL = 0x11,
	HXH = 0x12,
	HYL = 0x13,
	HYH = 0x14,
	HZL = 0x15,
	HZH = 0x16,
	ST2 = 0x18,
	CNTL2 = 0x31,
	CNTL3 = 0x32
} AK09916_Register;
/* AK09916 CONSTANTS END */


/* ICM20948 FUNCTIONS START */
HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef* hi2c, SDO_Pinouts pinout);

HAL_StatusTypeDef ICM20948_ReadUserBank(uint8_t* data);
HAL_StatusTypeDef ICM20948_ChangeUserBank(user_bank ubank);

HAL_StatusTypeDef ICM20948_ReadRegister(const reg_R* regi, uint8_t* data);
HAL_StatusTypeDef ICM20948_ReadRegisters(const reg_R* regi, uint8_t* data, uint16_t size);
HAL_StatusTypeDef ICM20948_WriteRegister(const reg_RW* regi, uint8_t data);
HAL_StatusTypeDef ICM20948_WriteRegisterEnables(const reg_RW* regi, uint8_t data);
HAL_StatusTypeDef ICM20948_WriteRegisterDisables(const reg_RW* regi, uint8_t data);

HAL_StatusTypeDef ICM20948_ReadSensorRegisters(int16_vector3* accel, int16_vector3* gyro);
HAL_StatusTypeDef ICM20948_ReadGyroRegisters(int16_vector3* gyro);
float_vector3 ICM20948_ScaleSensorVectors(int16_vector3* sensor_v, float scale_factor);

HAL_StatusTypeDef ICM20948_MeasureGyroOffset(uint32_t ticks, int16_vector3* gyro, uint32_t gyro_update_period_ms);
HAL_StatusTypeDef ICM20948_WriteGyroOffsetRegisters(int16_vector3* gyro_offset);

HAL_StatusTypeDef ICM20948_Wake();
HAL_StatusTypeDef ICM20948_Sleep();
HAL_StatusTypeDef ICM20948_Reset();

HAL_StatusTypeDef AK09916_Init();
HAL_StatusTypeDef AK09916_Read(AK09916_Register regi, uint8_t* data);
HAL_StatusTypeDef AK09916_SetCNTL2(CNTL2_Mode mode);
HAL_StatusTypeDef AK09916_Reset();
/* ICM20948 FUNCTIONS END */
