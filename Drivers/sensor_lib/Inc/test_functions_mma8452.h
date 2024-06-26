/*
 * test_functions_mma8452.h
 *
 *  Created on: Jun 26, 2024
 *      Author: oakyol
 */

#ifndef SENSOR_LIB_INC_TEST_FUNCTIONS_MMA8452_H_
#define SENSOR_LIB_INC_TEST_FUNCTIONS_MMA8452_H_

#include "mma8452.h"


int MMA845x_Test_Sensor(I2C_HandleTypeDef hi2c, uint8_t Device_Address, uint32_t Trial);

#endif /* SENSOR_LIB_INC_TEST_FUNCTIONS_MMA8452_H_ */
