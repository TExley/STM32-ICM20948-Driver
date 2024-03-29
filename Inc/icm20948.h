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
	const uint8_t init_value;
	const uint8_t reserved_mask;
} reg_RW;
/* ICM20948 REGISTER DEFINITION CONTINUED END */


/* ICM20948 USER UBANK 0 REGISTERS START */
static const reg_R REG_WHO_AM_I = { "WHO_AM_I", 0x00, UBANK_0 };
static const uint8_t REG_WHO_AM_I_VALUE = 0xEA; // Read Only Value

/* BIT 	| NAME 			| DESC
 * 7 	| -				| reserved
 * 6	| I2C_MST_CYCLE	| (See p68 for I2C_MST_ODR_CONFIG)
 * 5	| ACCEL_CYCLE	| 1 sets duty cycled mode
 * 4	| GYRO_CYCLE	| 1 sets duty cycled mode
 * 3:0	| -				| reserved
 * Reset Value: 0b01000000
 */
static const reg_RW REG_LP_CONFIG = { "LP_CONFIG", 0x05, UBANK_0 };

/* BIT 	| NAME 			| DESC
 * 7 	| DEVICE_RESET	| 1 resets all registers
 * 6	| Sleep			| 1 sets sleep mode
 * 5	| LP_EN			| 1 sets low power mode
 * 4	| -				| reserved
 * 3	| TEMP_DIS		| 1 disables temp sensor
 * 2:0	| CLKSEL		| 1-5 auto clock (DS p37)
 * Reset Value: 0b01000001
 */
static const reg_RW REG_PWR_MGMT_1 = { "PWR_MGMT_1", 0x06, UBANK_0, 0b01000001, 0b00010000 };
static const uint8_t REG_PWR_MGMT_1_VALUE_RESET = 0b10000000;
static const uint8_t REG_PWR_MGMT_1_VALUE_WAKE = 0b00111111 & REG_PWR_MGMT_1.init_value;
static const uint8_t REG_PWR_MGMT_1_VALUE_SLEEP = (0b01000000 | REG_PWR_MGMT_1.init_value) & ~REG_PWR_MGMT_1_VALUE_RESET;

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
/* ICM20948 USER UBANK 0 REGISTERS END */


/* ICM20948 USER UBANK 1 REGISTERS START */
/* BIT 	| NAME 			| DESC
 * 7:0 	| XA_OFFS[14:7]	| Upper bits of X accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_XA_OFFS_H = { "XA_OFFS_H", 0x14, UBANK_1, 0b00000000, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| XA_OFFS[6:0]	| Lower bits of X accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_XA_OFFS_L = { "XA_OFFS_L", 0x15, UBANK_1, 0b00000000, 0b00000001 };

/* BIT 	| NAME 			| DESC
 * 7:0 	| YA_OFFS[14:7]	| Upper bits of Y accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_YA_OFFS_H = { "YA_OFFS_H", 0x17, UBANK_1, 0b00000000, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| YA_OFFS[6:0]	| Lower bits of Y accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_YA_OFFS_L = { "YA_OFFS_L", 0x18, UBANK_1, 0b00000000, 0b00000001 };

/* BIT 	| NAME 			| DESC
 * 7:0 	| ZA_OFFS[14:7]	| Upper bits of Z accel offset cancellation
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_ZA_OFFS_H = { "ZA_OFFS_H", 0x1A, UBANK_1, 0b00000000, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| ZA_OFFS[6:0]	| Lower bits of Z accel offset cancellation
 * 0 	| -				| reserved
 * Reset Value: Trimmed on a per-part basis for optimal performance
 */
static const reg_RW REG_ZA_OFFS_L = { "ZA_OFFS_L", 0x1B, UBANK_1, 0b00000000, 0b00000001 };
/* ICM20948 USER UBANK 1 REGISTERS END */


/* ICM20948 USER UBANK 2 REGISTERS START */
/* BIT 	| NAME 				| DESC
 * 7:0 	| GYRO_SMPLRT_DIV	| sample rate dividor 1.1kHz/(1 + GYRO_SMPLRT_DIV)
 * Reset Value: 0b00000000
 */
static const reg_RW REG_GYRO_SMPLRT_DIV = { "GYRO_SMPLRT_DIV", 0x00, UBANK_2, 0b11111111, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5:3	| GYRO_DLPFCFG	| low pass filter config (DS p60)
 * 2:1	| GYRO_FS_SEL	| full scale select (DS p59)
 * 0	| GYRO_FCHOICE	| 1 enable DLPF
 * Reset Value: 0b00000000
 */
static const reg_RW REG_GYRO_CONFIG_1 = { "GYRO_CONFIG_1", 0x01, UBANK_2, 0b00000001, 0b11000000 };

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5	| XGYRO_CTEN	| 1 enable x self test
 * 4	| YGYRO_CTEN	| 1 enable y self test
 * 3	| ZGYRO_CTEN	| 1 enable z self test
 * 2:0	| GYRO_AVGCFG	| average lp-mode filter config (DS p60)
 * Reset Value: 0b00000000
 */
static const reg_RW REG_GYRO_CONFIG_2 = { "GYRO_CONFIG_2", 0x02, UBANK_2, 0b00000101, 0b11000000 };

/* BIT 	| NAME 					| DESC
 * 7:0 	| XG_OFFS_USER[15:8]	| Upper bits of X gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_XG_OFFS_USRH = { "XG_OFFS_USRH", 0x03, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| XG_OFFS_USER[7:0]	| Lower bits of X gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_XG_OFFS_USRL = { "XG_OFFS_USRL", 0x04, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 					| DESC
 * 7:0 	| YG_OFFS_USER[15:8]	| Upper bits of Y gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_YG_OFFS_USRH = { "YG_OFFS_USRH", 0x05, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| YG_OFFS_USER[7:0]	| Lower bits of Y gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_YG_OFFS_USRL = { "YG_OFFS_USRL", 0x06, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 					| DESC
 * 7:0 	| ZG_OFFS_USER[15:8]	| Upper bits of Z gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ZG_OFFS_USRH = { "ZG_OFFS_USRH", 0x07, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| ZG_OFFS_USER[7:0]	| Lower bits of Z gyro offset cancellation
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ZG_OFFS_USRL = { "ZG_OFFS_USRL", 0x08, UBANK_2, 0b00000000, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:1 	| -				| reserved
 * 0	| ODR_ALIGN_EN	| sensor start time alignment if and smprt set
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ODR_ALIGN_EN = { "ODR_ALIGN_EN", 0x09, UBANK_2, 0b00000001, 0b11111110 };

/* BIT 	| NAME 				| DESC
 * 7:4 	| -					| reserved
 * 3:0	| ACCEL_SMPLRT_DIV	| MSB for ACCEL_SMPLRT_DIV
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ACCEL_SMPLRT_DIV_1 = { "ACCEL_SMPLRT_DIV_1", 0x10, UBANK_2, 0b00000000, 0b11110000 };

/* BIT 	| NAME 				| DESC
 * 7:0 	| ACCEL_SMPLRT_DIV	| LSB for sample rate dividor 1.125kHz/(1 + ACCEL_SMPLRT_DIV[11:0])
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ACCEL_SMPLRT_DIV_2 = { "ACCEL_SMPLRT_DIV_2", 0x11, UBANK_2, 0b11111111, 0b00000000 };

/* BIT 	| NAME 			| DESC
 * 7:6 	| -				| reserved
 * 5:3	| ACCEL_DLPFCFG	| low pass filter config (DS p64)
 * 2:1	| ACCEL_FS_SEL	| full scale select (DS p64)
 * 0	| ACCEL_FCHOICE	| 1 enable DLPF
 * Reset Value: 0b00000001
 */
static const reg_RW REG_ACCEL_CONFIG_1 = { "ACCEL_CONFIG_1", 0x14, UBANK_2, 0b00000011, 0b11000000 };

/* BIT 	| NAME 			| DESC
 * 7:5 	| -				| reserved
 * 4	| AX_ST_EN_REG	| 1 enable x self test
 * 3	| AY_ST_EN_REG	| 1 enable y self test
 * 2	| AZ_ST_EN_REG	| 1 enable z self test
 * 1:0	| DEC3_CFG		| controls samples averaged in devimator (DS p65)
 * Reset Value: 0b00000000
 */
static const reg_RW REG_ACCEL_CONFIG_2 = { "ACCEL_CONFIG_2", 0x15, UBANK_2, 0b00000001, 0b11100000 };

/* BIT 	| NAME 			| DESC
 * 7:3 	| -				| unused (not labeled reserved)
 * 2:0	| TEMP_DLPFCFG	| low pass filter config (DS p67)
 * Reset Value: 0b00000000
 */
static const reg_RW REG_TEMP_CONFIG = { "TEMP_CONFIG", 0x53, UBANK_2, 0b00000000, 0b00000000 };
/* ICM20948 USER UBANK 2 REGISTERS END */


/* REGISTER SUPPORT CONSTS START */
// ODR computation formula in Hz (DS p59)
static const float GYRO_UPDATE_FREQUENCY = 1100 / (1 + REG_GYRO_SMPLRT_DIV.init_value);
static const uint32_t GYRO_UPDATE_PERIOD_MS = 1000 * (1 + (uint32_t) REG_GYRO_SMPLRT_DIV.init_value) / 1100;

// ODR computation formula in Hz (DS p63)
static const float ACCEL_UPDATE_FREQUENCY =
		1100 / (1 + (((uint16_t) REG_ACCEL_SMPLRT_DIV_1.init_value) << BITS_PER_BYTE) + REG_ACCEL_SMPLRT_DIV_2.init_value);
static const uint32_t ACCEL_UPDATE_PERIOD_MS =
		1000 * (1 + (((uint32_t) REG_ACCEL_SMPLRT_DIV_1.init_value) << BITS_PER_BYTE) + REG_ACCEL_SMPLRT_DIV_2.init_value) / 1100;

// 131 is typical value for FS_SEL = 0 (DS p11)
static const float GYRO_SENSITIVITY_SCALE_FACTOR = 1.f / (131 >> ((REG_GYRO_CONFIG_1.init_value & 0b110) >> 1));

// 16384 is typical value for FS_SEL = 0 (DS p11)
static const float ACCEL_SENSITIVITY_SCALE_FACTOR = 1.f / (16384 >> ((REG_ACCEL_CONFIG_1.init_value & 0b110) >> 1));
/* REGISTER SUPPORT CONSTS END */


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
// Start-up time for register read/write from power-up
static const uint32_t STARTUP_DELAY = 100;

/* Start-up time for sensor output from sleep
 * ACC ~ 30ms
 * GYR ~ 35ms
 * MAG ~ 8ms
 */
static const uint32_t WAKE_DELAY = 45;

// Exact time between sensor read requests
static const uint32_t READ_DELAY = 100;

// Maximum time between sensor sleep and wake-up
static const uint32_t MAXIMUM_SLEEP_DELAY = READ_DELAY - WAKE_DELAY;

// Maximum time-out to wait for any sensor ACK
static const uint32_t MAXIMUM_ICM_TIMEOUT = 20;
/* DELAYS AND TIMEOUTS END */
/* ICM20948 CONSTANTS END */


/* ICM20948 ENUMS START */
typedef enum SDO_Pinouts {SDO_LOW, SDO_HIGH} SDO_Pinouts;
/* ICM20948 ENUMS END */


/* ICM20948 FUNCTIONS START */
HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef* hi2c, SDO_Pinouts pinout);

HAL_StatusTypeDef ICM20948_ReadUserBank(uint8_t* data);
HAL_StatusTypeDef ICM20948_ChangeUserBank(user_bank ubank);

HAL_StatusTypeDef ICM20948_ReadRegister(const reg_R* regi, uint8_t* data);
HAL_StatusTypeDef ICM20948_ReadRegisters(const reg_R* regi, uint8_t* data, uint16_t size);
HAL_StatusTypeDef ICM20948_WriteRegister(const reg_RW* regi, uint8_t data);

HAL_StatusTypeDef ICM20948_ReadSensorRegisters(int16_vector3* accel, int16_vector3* gyro);

float_vector3 ICM20948_ScaleSensorVectors(int16_vector3* sensor_v, float scale_factor);

HAL_StatusTypeDef ICM20948_Wake();
HAL_StatusTypeDef ICM20948_Sleep();
/* ICM20948 FUNCTIONS END */
