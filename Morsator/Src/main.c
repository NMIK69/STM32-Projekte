/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define n_L 900   // long signal in ms
#define n_S 300   // short signal in ms
#define DTBB 100  // Default Time Between Blinks in ms
#define WB 300    // Word Break, time between words in ms (gets added to CB)
#define SB 900    // Symbol Break, time between symbols in ms
#define CB 300    // Character Break, time between characters in ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char *msg = "Eingabe: \r\n";

uint8_t val = 0;
uint8_t buffer_max_size = 100;

uint8_t bindex = 0;
uint8_t buffer[100];

// morse code for all alphanumerics, A-Z(index 0-26) 0-9(index 26-36)
unsigned int morse_code_an[36][5] = {{n_S, n_L, 0, 0, 0}, {n_L, n_S, n_S, n_S, 0}, {n_L, n_S, n_L, n_S, 0}, {n_L, n_S, n_S, 0, 0}, {n_S, 0, 0, 0, 0}, {n_S, n_S, n_L, n_S, 0}, {n_L, n_L, n_S, 0, 0}, {n_S, n_S, n_S, n_S, 0}, {n_S, n_S, 0 , 0, 0}, {n_S, n_L, n_L, n_L, 0}, {n_L, n_S, n_L, 0 , 0}, {n_S, n_L, n_S, n_S, 0}, {n_L, n_L, 0, 0, 0}, {n_L, n_S, 0, 0, 0}, {n_L, n_L, n_L, 0, 0}, {n_S, n_L, n_L, n_S, 0}, {n_L, n_L, n_S, n_L, 0}, {n_S, n_L, n_S, 0, 0}, {n_S, n_S, n_S, 0, 0}, {n_L, 0, 0, 0, 0}, {n_S, n_S, n_L, 0, 0}, {n_S, n_S, n_S, n_L, 0}, {n_S, n_L, n_L, 0, 0}, {n_L, n_S, n_S, n_L, 0}, {n_L, n_S, n_L, n_L, 0}, {n_L, n_L, n_S, n_S, 0}, {n_L, n_L, n_L, n_L, n_L}, {n_S, n_L, n_L, n_L, n_L}, {n_S, n_S, n_L, n_L, n_L}, {n_S, n_S, n_S, n_L, n_L}, {n_S, n_S, n_S, n_S, n_L}, {n_S, n_S, n_S, n_S, n_S}, {n_L, n_S, n_S, n_S, n_S}, {n_L, n_L, n_S, n_S, n_S}, {n_L, n_L, n_L, n_S, n_S}, {n_L, n_L, n_L, n_L, n_S}};

// morse code for symbols with sym_has for reference
unsigned int morse_code_sym[16][6] = {{n_S,n_L,n_S,n_L,n_S,n_L},{n_L,n_L,n_S,n_S,n_L,n_L},{n_L,n_L,n_L,n_S,n_S,n_S},{n_L,n_S,n_L,n_S,n_L,n_S},{n_S,n_S,n_L,n_L,n_S,n_S},{n_L,n_S,n_L,n_S,n_L,n_L},{n_L,n_S,n_S,n_S,n_S,n_L},{n_S,n_S,n_L,n_L,n_S,n_L},{n_L,n_S,n_L,n_L,n_S,0},{n_L,n_S,n_L,n_L,n_S,n_L},{n_S,n_L,n_L,n_L,n_L,n_S},{n_S,n_L,n_S,n_S,n_L,n_S},{n_L,n_S,n_S,n_S,n_L,0},{n_S,n_L,n_S,n_L,n_S,0},{n_L,n_S,n_S,n_L,n_S,0},{n_S,n_L,n_L,n_S,n_L,n_S}};
uint8_t sym_hash[16] = {'.',',',':',';','?','!','-','_','(',')','\'','"','=','+','/','@'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void receive_message(void);
void morse(uint8_t);
uint8_t get_key(uint8_t);
uint8_t get_sym_key(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void receive_message() {
	while(bindex < buffer_max_size - 1) {

		if(HAL_UART_Receive(&huart2, &val, 1, 5000) == 0) {

			HAL_UART_Transmit(&huart2, &val, 1, 10);
			if(val == '\n') break;

			buffer[bindex] = val;
			bindex += 1;
		}
	}
	buffer[bindex] = '\n';


	//HAL_UART_Transmit(&huart2, buffer, bindex, 1000);
	HAL_Delay(1000);


	HAL_Delay(2000);
	for(uint8_t i = 0; i < bindex; i++) {
		morse(buffer[i]);
		HAL_Delay(CB);
	}

	val = 0;
	bindex = 0;
}


uint8_t get_sym_key(uint8_t val) {
	for(uint8_t i = 0; i < 16; i++) {
		if(val == sym_hash[i]) return i;
	}

	//if it is not found in sym_hash it gets ignored
	return 200;
}


uint8_t get_key(uint8_t val) {
	//number
	if(val >= 48 && val <= 57) return val - 48 + 26;

	// capital letter
	if(val >= 65 && val <= 90) return val - 65;

	// small letter
	if(val >= 97 && val <= 122) return val - 97;

	// WB
	if(val == 32) return 100;

	// SB
	return 101;
}


void morse(uint8_t val) {
	uint8_t i = 0;
	uint8_t key = get_key(val);


	// if it is a whitespace
	if(key == 100) {
		HAL_Delay(WB);
		return;
	}


	// is a symbol
	else if(key == 101) {
		// if it is a symbol get the symbol key
		key = get_sym_key(val);

		if(key < 16) {
			while(i < 6 && morse_code_sym[key][i] != 0) {

				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
				HAL_Delay(morse_code_sym[key][i]);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
				HAL_Delay(DTBB);

				i += 1;
			}
		}

		// Symbol break
		HAL_Delay(SB);
		return;
	}


	//alphanumeric
	else {
		while(i < 5 && morse_code_an[key][i] != 0) {

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_Delay(morse_code_an[key][i]);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_Delay(DTBB);

			i += 1;
		}
	}

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
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	receive_message();
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
