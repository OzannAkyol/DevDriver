/*
 * test_functions_mma8452.c
 *
 *  Created on: Jun 26, 2024
 *      Author: oakyol
 */

#include "test_functions_mma8452.h"

extern I2C_HandleTypeDef hi2c1;

int MMA845x_Test_Sensor(I2C_HandleTypeDef hi2c, uint8_t Device_Address, uint32_t Trial){

	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c, (Device_Address << 1), Trial, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

