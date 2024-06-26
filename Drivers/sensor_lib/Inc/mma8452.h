/*
 * mma8452.h
 *
 *  Created on: Jun 26, 2024
 *      Author: oakyol
 */

#ifndef SENSOR_LIB_INC_MMA8452_H_
#define SENSOR_LIB_INC_MMA8452_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>



/*Some Specific Sensor Address*/
#define MMA845x_DEVICE_ADDR						(0x1C)

/*Some Sensor Register*/
#define MMA845x_XYZ_DATA_CFG_REG	 			(0x0E)
#define MMA845x_WHO_AM_I_REG					(0x0D)
#define MMA845x_CTRL1_REG						(0x2A)
#define MMA845x_HP_FILTER_CUTOFF_REG			(0x0F)
#define MMA845x_CTRL2_REG						(0x2B)
#define MMA845x_STATUS_REG						(0x00)
#define MMA845x_OUT_X_MSB						(0x01)
#define MMA845x_OUT_X_LSB						(0x02)
#define MMA845x_OUT_Y_MSB						(0x03)
#define MMA845x_OUT_Y_LSB						(0x04)
#define MMA845x_OUT_Z_MSB						(0x05)
#define MMA845x_OUT_Z_LSB						(0x06)
#define REG_SIZE								1

/*Sensor Data Rate*/
#define SET_DATA_RATE_800Hz						(0x00)
#define SET_DATA_RATE_400Hz						(0x01)
#define SET_DATA_RATE_200Hz						(0x02)
#define SET_DATA_RATE_100Hz						(0x03)
#define SET_DATA_RATE_50Hz						(0x04)
#define SET_DATA_RATE_12_5Hz					(0x05)
#define SET_DATA_RATE_6_5Hz						(0x06)
#define SET_DATA_RATE_1_563Hz					(0x07)
#define MAX_NUM_OF_SCALE_VALUE					(0x08)

/*Sensor OverSampling Mode*/
#define SENSOR_NORMAL_POWER_MODE_VAL			(0x00)
#define SENSOR_LOW_NOISE_LOW_POWER_MODE_VAL		(0x01)
#define SENSOR_HIGH_RESOLUTION_MODE_VAL			(0x02)
#define SENSOR_LOW_POWER_MODE_VAL				(0x03)
#define MAX_NUM_OF_SENSOR_MODE 					(0x04)

/*Sensor */
#define SET_CUTOFF_FREQ_16Hz					(0x00)
#define SET_CUTOFF_FREQ_8Hz						(0x01)
#define SET_CUTOFF_FREQ_4Hz						(0x02)
#define SET_CUTOFF_FREQ_2Hz						(0x03)

/*Some Sensor's Specific Bits */
#define ZYXDR_BIT			(1 << 3)
#define ACTIVE_MASK_BIT		(1 << 0)

/*Sensor High Pass Filter Value*/
#define MAX_NUM_OF_HP_FILTER_VALUE				(0x04)
#define MMA845x_HPF_OUT_BIT_MASK				(0x10)

/*DATA_MASK*/
#define DR_MASK					 				(0x38)
#define MMA845x_CTRL2_REG_BIT3_BIT4_MASK		(0x18)
#define	MMA845x_CUTOFF_REG_BIT1_BIT0_MASK		(0x03)

typedef struct{
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;
}SensorData;

enum{

	TX_STATE_BUSY, 	/*0*/
	RX_STATE_BUSY,	/*1*/
	NUM_OF_STATES	/*2*/
};

typedef enum SENSOR_FILTER_TYPES{

	SENSOR_HIGH_PASS_FILTER, 			/*0*/
	SENSOR_LOW_PASS_FILTER				/*1*/

}SensorFilterType;

typedef enum SENSOR_STATES{

	SENSOR_STANDBY, 	/*0*/
	SENSOR_ACTIVE,		/*1*/
	SENSOR_OFF			/*2*/

}SensorStates;

typedef enum POWER_MODES {

	SENSOR_NORMAL_POWER_MODE,					/*0*/
	SENSOR_LOW_NOISE_LOW_POWER_MODE,			/*1*/
	SENSOR_HIGH_RESOLUTION_MODE,				/*2*/
	SENSOR_LOW_POWER_MODE

}PowerModeTypeDef;

typedef enum SENSOR_MODE{

	SENSOR_OK,		/*0*/
	SENSOR_ERROR	/*1*/

}SensorMode;

typedef enum SENSOR_G_MODE{
	SENSOR_2G_MODE,
	SENSOR_4G_MODE,
	SENSOR_8G_MODE
}G_mode;

/*Library Functions*/

uint8_t MMA845x_Read_ID(I2C_HandleTypeDef hi2c, uint8_t Device_Addres, uint8_t RegAddress);
int Set_Sensor_Scale(int SCALE_TYPE);
SensorMode MMA845x_Set_Sensor_G_Mode(G_mode mode);
SensorMode MMA845x_Set_Data_Rate(unsigned char Data_Rate_Value);
SensorMode MMA845x_Set_Sensor_Power_Mode(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes);
SensorMode MMA845x_Set_Sensor_HP_Cutoff_Frequency(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes);
SensorMode MMA845x_Set_Sensor_Filter_Type(SensorFilterType filter);
SensorMode MMA845x_Set_Sensor_State(SensorStates states);
SensorData MMA845x_Read_Sensor_Value();
void SCI_s12dec_Out(uint16_t sensor_data);


#endif /* SENSOR_LIB_INC_MMA8452_H_ */
