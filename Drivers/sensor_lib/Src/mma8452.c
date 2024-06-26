/*
 * mma8452.c
 *
 *  Created on: Jun 26, 2024
 *      Author: oakyol
 */

#include "mma8452.h"

I2C_HandleTypeDef hi2c1;

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void SCI_s12dec_Out(uint16_t sensor_data)
{
	uint8_t thousands, hundreds, tens, ones;
    uint16_t r;

    /*
     ** Check data's sign
     */
    if (sensor_data > 0x7FF)
    {
    	putchar('-');
    	sensor_data = (~sensor_data + 1) & 0x0FFF; // 2's complement and 12-bit mask
    }
    else
    {
    	putchar('+');
    }


    thousands = (uint8_t)(sensor_data / 1000);
    r = sensor_data % 1000;
    hundreds = (uint8_t)(r / 100);
    r = (uint8_t)(r % 100);
    tens = (uint8_t)(r / 10);
    ones = (uint8_t)(r % 10);

    /*
     ** Format adjustment for leading zeros
     */
    if (thousands == 0)
    {
    	thousands = 0xF0;
        if (hundreds == 0)
        {
        	hundreds = 0xF0;
        }
        	if (tens == 0)
        	{
        		tens = 0xF0;
        	}
    }

    /*
     ** Output result
     */
    putchar(thousands + '0');
    putchar(hundreds + '0');
    putchar(tens + '0');
    putchar(ones + '0');
}

SensorData MMA845x_Read_Sensor_Value(){
	uint8_t reg_status;
	uint8_t xyz_data_val[6];

	SensorData Data;
	HAL_StatusTypeDef ret_val = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_STATUS_REG, REG_SIZE, &reg_status, 1, 100);
	if(HAL_ERROR == ret_val){
		Data.x_data = -2;
		Data.y_data = -2;
		Data.z_data = -2;
		return Data;
	}

	/*Check X, Y, Z-axis new data ready. Polling ... */
	if(reg_status & ZYXDR_BIT){
		/*Read  */
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_X_MSB, REG_SIZE, &xyz_data_val [0], 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_X_LSB, REG_SIZE, &xyz_data_val [1], 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_Y_MSB, REG_SIZE, &xyz_data_val [2], 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_Y_LSB, REG_SIZE, &xyz_data_val [3], 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_Z_MSB, REG_SIZE, &xyz_data_val [4], 1, 100);
		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_OUT_Z_LSB, REG_SIZE, &xyz_data_val [5], 1, 100);
	}
	Data.x_data = (xyz_data_val[0] << 8) | xyz_data_val[1];
	Data.x_data >>= 4;

	Data.y_data = (xyz_data_val[2] << 8) | xyz_data_val[3];
	Data.y_data >>= 4;

	Data.z_data = (xyz_data_val[4] << 8) | xyz_data_val[5];
	Data.z_data >>= 4;

	return Data;
}

SensorMode MMA845x_Set_Sensor_Filter_Type(SensorFilterType filter){

	uint8_t reg_status;
	HAL_StatusTypeDef ret_val = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);

	if(HAL_ERROR == ret_val)
		return SENSOR_ERROR;

	if(SENSOR_HIGH_PASS_FILTER == filter ){
		reg_status &= ~(MMA845x_HPF_OUT_BIT_MASK);
		reg_status |= MMA845x_HPF_OUT_BIT_MASK;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);
		return SENSOR_OK;
	}else if(SENSOR_LOW_PASS_FILTER == filter) {
		reg_status &= ~(MMA845x_HPF_OUT_BIT_MASK);
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);
		return SENSOR_OK;
	}
	return SENSOR_ERROR;
}

SensorMode MMA845x_Set_Sensor_HP_Cutoff_Frequency(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes){

	uint8_t cutoff_reg_status;
	HAL_StatusTypeDef ret_val;

	if(Sensor_Mode_Value >= MAX_NUM_OF_HP_FILTER_VALUE)
		return SENSOR_ERROR;

	ret_val = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_HP_FILTER_CUTOFF_REG, REG_SIZE, &cutoff_reg_status, 1, 100);

	if(ret_val != HAL_OK)
		return SENSOR_ERROR;

	cutoff_reg_status &= ~(MMA845x_CUTOFF_REG_BIT1_BIT0_MASK);
	cutoff_reg_status |= Sensor_Mode_Value;

	if(SENSOR_NORMAL_POWER_MODE == Modes)
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_HP_FILTER_CUTOFF_REG, REG_SIZE, &cutoff_reg_status, 1, 100);
	else if(SENSOR_LOW_NOISE_LOW_POWER_MODE == Modes)
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_HP_FILTER_CUTOFF_REG, REG_SIZE, &cutoff_reg_status, 1, 100);
	else if (SENSOR_HIGH_RESOLUTION_MODE == Modes)
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_HP_FILTER_CUTOFF_REG, REG_SIZE, &cutoff_reg_status, 1, 100);
	else if (SENSOR_LOW_POWER_MODE == Modes)
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR<<1), MMA845x_HP_FILTER_CUTOFF_REG, REG_SIZE, &cutoff_reg_status, 1, 100);
	else{
		return SENSOR_ERROR;
	}
		return SENSOR_OK;
}


SensorMode MMA845x_Set_Sensor_Power_Mode(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes){
	uint8_t reg_status;

	Sensor_Mode_Value = Sensor_Mode_Value << 3;

	if(Sensor_Mode_Value >= MAX_NUM_OF_SENSOR_MODE)
		return SENSOR_ERROR;

	if(SENSOR_NORMAL_POWER_MODE == Modes){

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		return SENSOR_OK;

	}else if(SENSOR_LOW_NOISE_LOW_POWER_MODE == Modes){

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);
		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		return SENSOR_OK;

	}else if(SENSOR_HIGH_RESOLUTION_MODE == Modes){

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		return SENSOR_OK;

	}else if(SENSOR_LOW_POWER_MODE == Modes){

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		return SENSOR_OK;

	}else
		return SENSOR_ERROR;


}

SensorMode MMA845x_Set_Data_Rate(unsigned char Data_Rate_Value){

	if(Data_Rate_Value > MAX_NUM_OF_SCALE_VALUE)
		return SENSOR_ERROR;

	uint8_t reg_status;

	Data_Rate_Value = Data_Rate_Value << 3;																				  /* To write this Control Register's Bit5, Bit4, Bit3 */

	HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100); /* To Read Control Register Status  */

	reg_status &= ~(DR_MASK);																			  /* Clear the Control Register's Bit5, Bit4, Bit3 */

	reg_status |= Data_Rate_Value;																		  /*Set Data Rate */

	HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);

	return SENSOR_OK;
}

SensorMode MMA845x_Set_Sensor_G_Mode(G_mode mode){

	uint8_t reg_status;

	HAL_StatusTypeDef retval;
	retval = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);

	if( HAL_ERROR == retval )
		return SENSOR_ERROR;

	if(SENSOR_2G_MODE == mode){
		reg_status &= ~(0<<1);
		reg_status &= ~(1<<1);

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);
	}else if(SENSOR_4G_MODE == mode){
		reg_status |=  (0<<1);
		reg_status &= ~(1<<1);

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);
	}else if(SENSOR_8G_MODE == mode){

		reg_status &= ~(0<<1);
		reg_status |=  (1<<1);

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_XYZ_DATA_CFG_REG, REG_SIZE, &reg_status, 1, 100);
	}else
		return SENSOR_ERROR;


	return SENSOR_OK;


}

SensorMode MMA845x_Set_Sensor_State(SensorStates states){
	uint8_t reg_status;
	HAL_StatusTypeDef retval;
	retval = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);

	if(HAL_ERROR == retval)
		return SENSOR_ERROR;

	if(SENSOR_STANDBY == states ){
		reg_status &= ~(ACTIVE_MASK_BIT);
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);
		return SENSOR_OK;
	}else if(SENSOR_ACTIVE == states){
		reg_status |= (ACTIVE_MASK_BIT);
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1 , 100);
		return SENSOR_OK;
	}
	return SENSOR_ERROR;

}


uint8_t MMA845x_Read_ID(I2C_HandleTypeDef hi2c, uint8_t Device_Addres, uint8_t RegAddress){

	uint8_t RxBuffer[1];
	HAL_I2C_Mem_Read(&hi2c, (Device_Addres << 1), RegAddress, REG_SIZE, RxBuffer, 1, 100);

	if(0x2A == RxBuffer[0])
		return *RxBuffer;
	else
		return 0;
}
