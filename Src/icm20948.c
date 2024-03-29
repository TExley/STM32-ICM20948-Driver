/*
 * icm20948.c
 *
 *  Created on: Mar 3, 2024
 *      Author: Lain
 */

#include "icm20948.h"

#define NUMBER_SENSOR_REGISTERS 12 // 2 registers for each of 6dof

/* ICM20948 VARIABLES START */
static uint16_t ICM20948_ADDR;
static I2C_HandleTypeDef* hi2c;
static user_bank current_ubank;
/* ICM20948 VARIABLES END */

/* ICM20948 FUNCTIONS START */
HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef* h_i2c, SDO_Pinouts  SDO_pinout)
{
	HAL_Delay(STARTUP_DELAY);

	hi2c = h_i2c;
	if (SDO_pinout == SDO_LOW)
		ICM20948_ADDR = ICM20948_ADDR_L;
	else
		ICM20948_ADDR = ICM20948_ADDR_H;

	HAL_StatusTypeDef status;
	uint8_t data;

	status = ICM20948_ReadUserBank(&data);
	if (status != HAL_OK)
		return status;
	current_ubank = (user_bank) data;

	status = ICM20948_ReadRegister(&REG_WHO_AM_I, &data);
	if (status != HAL_OK)
		return status;
	else if (data != REG_WHO_AM_I_VALUE)
		return HAL_ERROR;

	status = ICM20948_WriteRegister(&REG_PWR_MGMT_1, REG_PWR_MGMT_1_VALUE_RESET);
	if (status != HAL_OK)
		return status;

	HAL_Delay(STARTUP_DELAY);

	status = ICM20948_Wake();
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_LP_CONFIG, REG_LP_CONFIG.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_SMPLRT_DIV, REG_GYRO_SMPLRT_DIV.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_1, REG_GYRO_CONFIG_1.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_2, REG_GYRO_CONFIG_2.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ODR_ALIGN_EN, REG_ODR_ALIGN_EN.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_SMPLRT_DIV_1, REG_ACCEL_SMPLRT_DIV_1.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_SMPLRT_DIV_2, REG_ACCEL_SMPLRT_DIV_2.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_1, REG_ACCEL_CONFIG_1.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_2, REG_ACCEL_CONFIG_2.init_value);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_TEMP_CONFIG, REG_TEMP_CONFIG.init_value);
	if (status != HAL_OK)
		return status;

	ICM20948_Sleep();

	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadUserBank(uint8_t* data)
{
	return HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, REG_UBANK_SEL.address, I2C_MEMADD_SIZE_8BIT, data, 1, MAXIMUM_ICM_TIMEOUT);;
}

HAL_StatusTypeDef ICM20948_ChangeUserBank(user_bank ubank)
{
	uint8_t data;
	HAL_StatusTypeDef status = ICM20948_ReadUserBank(&data);
	if (status != HAL_OK)
		return status;

	uint8_t i_ubank = ((uint8_t) ubank) + (data & REG_UBANK_SEL_RESERVED_MASK);
	status = HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_UBANK_SEL.address, I2C_MEMADD_SIZE_8BIT, &i_ubank, 1, MAXIMUM_ICM_TIMEOUT);
	if (status == HAL_OK)
		current_ubank = ubank;
	return status;
}

// Local function
HAL_StatusTypeDef L_CheckUserRegister(const reg_R* regi)
{
	if (regi->bank != current_ubank)
		return ICM20948_ChangeUserBank(regi->bank);
	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadRegister(const reg_R* regi, uint8_t* data)
{
	return ICM20948_ReadRegisters(regi, data, 1);
}

HAL_StatusTypeDef ICM20948_ReadRegisters(const reg_R* regi, uint8_t* data, uint16_t size)
{
	HAL_StatusTypeDef status = L_CheckUserRegister(regi);
	if (status != HAL_OK)
		return status;

	return HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, regi->address, I2C_MEMADD_SIZE_8BIT, data, size, MAXIMUM_ICM_TIMEOUT);
}

HAL_StatusTypeDef ICM20948_WriteRegister(const reg_RW* regi, uint8_t data)
{
	if (regi->reserved_mask != 0)
	{
		uint8_t data_read;
		HAL_StatusTypeDef status = ICM20948_ReadRegister((reg_R*) regi, &data_read);
		if (status != HAL_OK)
			return status;

		data = data & ~regi->reserved_mask; // Make sure we aren't writing to reserved bits
		data += data_read & regi->reserved_mask; // Copy the value of reserved bits into data
	} else // ICM20948_ReadRegister already checks the userbank so we only do so if we don't read first
	{
		HAL_StatusTypeDef status = L_CheckUserRegister((reg_R*) regi);
		if (status != HAL_OK)
			return status;
	}

	return HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, regi->address, I2C_MEMADD_SIZE_8BIT, &data, 1, MAXIMUM_ICM_TIMEOUT);
}

// Local function
int16_t L_CombineRegisters(uint8_t data_H, uint8_t data_L)
{
	return (((int16_t) data_H) << BITS_PER_BYTE) + data_L;
}

HAL_StatusTypeDef ICM20948_ReadSensorRegisters(int16_vector3* accel, int16_vector3* gyro)
{
	uint8_t data[NUMBER_SENSOR_REGISTERS];
	HAL_StatusTypeDef status = ICM20948_ReadRegisters(&REG_ACCEL_XOUT_H, data, NUMBER_SENSOR_REGISTERS);
	if (status != HAL_OK)
		return status;

	accel->x = L_CombineRegisters(data[0], data[1]);
	accel->y = L_CombineRegisters(data[2], data[3]);
	accel->z = L_CombineRegisters(data[4], data[5]);
	gyro->x = L_CombineRegisters(data[6], data[7]);
	gyro->y = L_CombineRegisters(data[8], data[9]);
	gyro->z = L_CombineRegisters(data[10], data[11]);
	return HAL_OK;
}

float_vector3 ICM20948_ScaleSensorVectors(int16_vector3* sensor_v, float scale_factor)
{
	float_vector3 ret;
	ret.x = sensor_v->x * scale_factor;
	ret.y = sensor_v->y * scale_factor;
	ret.z = sensor_v->z * scale_factor;
	return ret;
}

HAL_StatusTypeDef ICM20948_Wake()
{
	HAL_StatusTypeDef status = ICM20948_WriteRegister(&REG_PWR_MGMT_1, REG_PWR_MGMT_1_VALUE_WAKE);
	if (status == HAL_OK)
		HAL_Delay(WAKE_DELAY);
	return status;
}

HAL_StatusTypeDef ICM20948_Sleep()
{
	return ICM20948_WriteRegister(&REG_PWR_MGMT_1, REG_PWR_MGMT_1_VALUE_SLEEP);
}
/* ICM20948 FUNCTIONS END */
