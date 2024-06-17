#include "main.h"
#include <stdlib.h>

I2C_HandleTypeDef hi2c1;

/*Some Specific Sensor Address*/
#define MMA845x_ACTIVE_MASK_BIT			(0<<1)
#define MMA845x_DEVICE_ADDR				(0x1C)
#define MMA_845x_ACTIVE					(0x2B)


/*Some Sensor Register*/
#define MMA845x_XYZ_DATA_CFG_REG	 	(0x0E)
#define MMA845x_WHO_AM_I_REG			(0x0D)
#define MMA845x_CTRL1_REG				(0x2A)
#define MMA845x_HP_FILTER_CUTOFF_REG	(0x0F)
#define MMA845x_CTRL2_REG				(0x2B)
#define REG_SIZE						1

/*Sensor Data Rate*/
#define SET_DATA_RATE_800Hz			(0x00)
#define SET_DATA_RATE_400Hz			(0x01)
#define SET_DATA_RATE_200Hz			(0x02)
#define SET_DATA_RATE_100Hz			(0x03)
#define SET_DATA_RATE_50Hz			(0x04)
#define SET_DATA_RATE_12_5Hz		(0x05)
#define SET_DATA_RATE_6_5Hz			(0x06)
#define SET_DATA_RATE_1_563Hz		(0x07)
#define MAX_NUM_OF_SCALE_VALUE		(0x08)

/*Sensor OverSampling Mode*/
#define SENSOR_NORMAL_POWER_MODE_VAL					(0x00)
#define SENSOR_LOW_NOISE_LOW_POWER_MODE_VAL				(0x01)
#define SENSOR_HIGH_RESOLUTION_MODE_VAL					(0x02)
#define SENSOR_LOW_POWER_MODE_VAL						(0x03)
#define MAX_NUM_OF_SENSOR_MODE							(0x04)

/*Sensor */
#define SET_CUTOFF_FREQ_16Hz			(0x00)
#define SET_CUTOFF_FREQ_8Hz				(0x01)
#define SET_CUTOFF_FREQ_4Hz				(0x02)
#define SET_CUTOFF_FREQ_2Hz				(0x03)


/*Sensor High Pass Filter Value*/
#define MAX_NUM_OF_HP_FILTER_VALUE				(0x04)
#define MMA845x_HPF_OUT_BIT_MASK				(0x10)

/*DATA_MASK*/
#define DR_MASK					 					(0x38)
#define MMA845x_CTRL2_REG_BIT3_BIT4_MASK			(0x18)
#define	MMA845x_CUTOFF_REG_BIT1_BIT0_MASK			(0x03)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


int retval =0;
uint8_t Sensor_ID_Response =0;


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

}SensorModeTypeDef;

typedef enum SENSOR_G_MODE{
	SENSOR_2G_MODE,
	SENSOR_4G_MODE,
	SENSOR_8G_MODE
}G_mode;

/*Library Functions*/
int MMA845x_Test_Sensor(I2C_HandleTypeDef hi2c, uint8_t Device_Addres);
uint8_t MMA845x_Read_ID(I2C_HandleTypeDef* hi2c, uint8_t Device_Addres, uint8_t RegAddress);
int Set_Sensor_Scale(int SCALE_TYPE);
SensorModeTypeDef MMA845x_Set_Sensor_G_Mode(G_mode mode);
SensorModeTypeDef MMA845x_Set_Data_Rate(unsigned char Data_Rate_Value);
SensorModeTypeDef MMA845x_Set_Sensor_Power_Mode(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes);
SensorModeTypeDef MMA845x_Set_Sensor_HP_Cutoff_Frequency(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes);
SensorModeTypeDef MMA845x_Set_Sensor_Filter(SensorFilterType filter);
SensorModeTypeDef MMA845x_Set_Sensor_State(SensorStates states);

uint8_t RxBuffer[1];
uint8_t TxBuffer[1]={0x0D};





int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_I2C1_Init();

  retval = MMA845x_Test_Sensor(hi2c1, MMA845x_DEVICE_ADDR);
  Sensor_ID_Response = MMA845x_Read_ID(&hi2c1, MMA845x_DEVICE_ADDR, MMA845x_WHO_AM_I_REG);

  MMA845x_Set_Data_Rate(SET_DATA_RATE_800Hz);
  MMA845x_Set_Sensor_Power_Mode(SENSOR_NORMAL_POWER_MODE_VAL, SENSOR_NORMAL_POWER_MODE);
  MMA845x_Set_Sensor_HP_Cutoff_Frequency(SET_CUTOFF_FREQ_16Hz, SENSOR_LOW_POWER_MODE);



   while (1)
  {


  }

}


SensorModeTypeDef MMA845x_Set_Sensor_Filter(SensorFilterType filter){/* sensor_active mode ?? sensor_standby_mode ?? */

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



SensorModeTypeDef MMA845x_Set_Sensor_HP_Cutoff_Frequency(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes){

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


SensorModeTypeDef MMA845x_Set_Sensor_Power_Mode(unsigned char Sensor_Mode_Value, PowerModeTypeDef Modes){
	uint8_t reg_status;

	Sensor_Mode_Value = Sensor_Mode_Value << 3;

	if(Sensor_Mode_Value >= MAX_NUM_OF_SENSOR_MODE)
		return SENSOR_ERROR;

	if(SENSOR_NORMAL_POWER_MODE == Modes){

		MMA845x_Set_Sensor_State(SENSOR_STANDBY);

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		MMA845x_Set_Sensor_State(SENSOR_ACTIVE);

		return SENSOR_OK;

	}else if(SENSOR_LOW_NOISE_LOW_POWER_MODE == Modes){

		MMA845x_Set_Sensor_State(SENSOR_STANDBY);

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);
		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		MMA845x_Set_Sensor_State(SENSOR_ACTIVE);

		return SENSOR_OK;

	}else if(SENSOR_HIGH_RESOLUTION_MODE == Modes){

		MMA845x_Set_Sensor_State(SENSOR_STANDBY);

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		MMA845x_Set_Sensor_State(SENSOR_ACTIVE);

		return SENSOR_OK;

	}else if(SENSOR_LOW_POWER_MODE == Modes){

		MMA845x_Set_Sensor_State(SENSOR_STANDBY);

		HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		/*DATA_ MASK*/
		reg_status &= ~(MMA845x_CTRL2_REG_BIT3_BIT4_MASK);

		reg_status |=	Sensor_Mode_Value;

		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL2_REG, REG_SIZE, &reg_status, 1, 100);

		MMA845x_Set_Sensor_State(SENSOR_ACTIVE);

		return SENSOR_OK;

	}else
		return SENSOR_ERROR;


}




//HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100); ????????????????????????????

SensorModeTypeDef MMA845x_Set_Data_Rate(unsigned char Data_Rate_Value){

	if(Data_Rate_Value > MAX_NUM_OF_SCALE_VALUE)
		return SENSOR_ERROR;

	uint8_t reg_status;

	MMA845x_Set_Sensor_State(SENSOR_STANDBY);

	Data_Rate_Value = Data_Rate_Value << 3;																				  /* To write this Control Register's Bit5, Bit4, Bit3 */

	HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100); /* To Read Control Register Status  */

	reg_status &= ~(DR_MASK);																			  /* Clear the Control Register's Bit5, Bit4, Bit3 */

	reg_status |= Data_Rate_Value;																		  /*Set Data Rate */

	HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);

	MMA845x_Set_Sensor_State(SENSOR_ACTIVE);

	return SENSOR_OK;
}

SensorModeTypeDef MMA845x_Set_Sensor_G_Mode(G_mode mode){

	uint8_t reg_status;
	MMA845x_Set_Sensor_State(SENSOR_STANDBY);

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


	MMA845x_Set_Sensor_State(SENSOR_ACTIVE);
		return SENSOR_OK;


}

SensorModeTypeDef MMA845x_Set_Sensor_State(SensorStates states){
	uint8_t reg_status;
	HAL_StatusTypeDef retval;
	retval = HAL_I2C_Mem_Read(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);

	if(HAL_ERROR == retval)
		return SENSOR_ERROR;

	if(SENSOR_STANDBY == states ){
		reg_status &= ~(MMA845x_ACTIVE_MASK_BIT);
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1, 100);
		return SENSOR_OK;
	}else if(SENSOR_ACTIVE == states){
		reg_status |= (MMA845x_ACTIVE_MASK_BIT);
		HAL_I2C_Mem_Write(&hi2c1, (MMA845x_DEVICE_ADDR << 1), MMA845x_CTRL1_REG, REG_SIZE, &reg_status, 1 , 100);
		return SENSOR_OK;
	}
	return SENSOR_ERROR;

}

int MMA845x_Test_Sensor(I2C_HandleTypeDef hi2c, uint8_t Device_Addres){

	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c, (Device_Addres << 1), 4, 100);
	if(status == HAL_OK)
		return 1;
	else
		return 0;
}

uint8_t MMA845x_Read_ID(I2C_HandleTypeDef* hi2c, uint8_t Device_Addres, uint8_t RegAddress){

	uint8_t RxBuffer[1];
	HAL_I2C_Mem_Read(hi2c, (Device_Addres << 1), RegAddress, REG_SIZE, RxBuffer, 1, 100);

	if(0x2A == RxBuffer[0])
		return *RxBuffer;
	else
		return 0;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
