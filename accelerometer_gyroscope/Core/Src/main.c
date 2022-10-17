/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_DEFAULT_ADDRESS (0x68 << 1)
#define MPU6050_WHO_AM_I (0x75)
#define MPU6050_RA_PWR_MGMT_1 (0x6B)
#define MPU6050_CLOCK_PLL_ZGYRO (0x03)
#define MPU6050_RA_CONFIG (0x1A)
#define MPU6050_RA_SMPLRT_DIV (0x19)
#define MPU6050_RA_GYRO_CONFIG (0x18)
#define MPU6050_GYRO_FS_2000 (0x3 << 3)
#define MPU6050_RA_ACCEL_CONFIG (0x1C)
#define MPU6050_ACCEL_FS_16 (0x3 << 3)
#define MPU6050_RA_INT_PIN_CFG (0x37)
#define MPU6050_RA_SIGNAL_PATH_RESET (0x68)
#define MPU6050_RA_INT_ENABLE (0x38)

// output registers
#define MPU6050_RA_ACCEL_XOUT_H (0x3B)
#define MPU6050_RA_GYRO_XOUT_H (0x43)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static int MPU6050_started = 0;
uint16_t mpu6050_raw_data[7];
uint8_t raw_data[6];

int Accel_X_RAW;
int Accel_Y_RAW;
int Accel_Z_RAW;

int Gyro_X_RAW;
int Gyro_Y_RAW;
int Gyro_Z_RAW;

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

struct mpu6050_data{
	float accelX;
	float accelY;
	float accelZ;
	float temp;
	float gyroX;
	float gyroY;
	float gyroZ;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct mpu6050_data mpu_data;

uint8_t MPU6050_Read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read( &hi2c1, slaveAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
			&data, 1, 100 );
	if (status != HAL_OK) {
		Error_Handler();
		// Error_Handler does not return
		return 0;
	}
	return data;
}

void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write( &hi2c1, slaveAddr, regAddr,
			I2C_MEMADD_SIZE_8BIT, &data, 1, 100 );
	if (status != HAL_OK) {
		Error_Handler();
	}
}

void read_accel() {

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, raw_data, 6, 1000);

	Accel_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data [1]);
	Accel_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data [3]);
	Accel_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data [5]);

	Ax = Accel_X_RAW/2048.0;
	Ay = Accel_Y_RAW/2048.0;
	Az = Accel_Z_RAW/2048.0;
}

void read_gyro() {

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, raw_data, 6, 1000);

	Gyro_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data [1]);
	Gyro_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data [3]);
	Gyro_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data [5]);

	Gx = Gyro_X_RAW/16.4;
	Gy = Gyro_Y_RAW/16.4;
	Gz = Gyro_Z_RAW/16.4;
}

void MPU6050_Init()
{
	//reset the whole module first
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1<<7);
	//wait for 50ms for the gyro to stable
	HAL_Delay(50);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);
	//PLL with Z axis gyroscope reference
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);
	//DLPF_CFG = 1: Fs=1khz; bandwidth=42hz
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1);
	// 1kHz/2
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);
	//Gyro full scale setting to ± 2000°/s
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);
	//Accel full scale setting to ± 16g
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1<<4);
	//interrupt status bits are cleared on any read operation
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x07);
	//reset gyro and accel sensor
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1<<0);
	//interrupt occurs when data is ready.
	MPU6050_started = 1;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(!MPU6050_started) {
		return;
	}

	if(GPIO_Pin != GPIO_PIN_1) {
		return;
	}


	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA( &hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*)&mpu6050_raw_data, sizeof(mpu6050_raw_data));

	if( status != HAL_OK ) {
		Error_Handler();
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

	mpu_data.accelX = __REVSH(mpu6050_raw_data[0]) * 1000 * 16 / (1U<<15);
	mpu_data.accelY = __REVSH(mpu6050_raw_data[1]) * 1000 * 16 / (1U<<15);
	mpu_data.accelZ = __REVSH(mpu6050_raw_data[2]) * 1000 * 16 / (1U<<15);

	// in milli-degree per second
	mpu_data.gyroX = __REVSH(mpu6050_raw_data[4]) * 1000 * 2000 / (1U<<15);
	mpu_data.gyroY = __REVSH(mpu6050_raw_data[5]) * 1000 * 2000 / (1U<<15);
	mpu_data.gyroZ = __REVSH(mpu6050_raw_data[6]) * 1000 * 2000 / (1U<<15);

	// Temperature in degrees C
	mpu_data.temp = __REVSH(mpu6050_raw_data[3])/340 + 36;


	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  __SEV();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
