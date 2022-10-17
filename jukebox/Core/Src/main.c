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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// defining all note freqeuncies
#define CFREQ 32000000 // Clock frequency
#define _C0    16
#define _CS0   17
#define _D0    18
#define _DS0   19
#define _E0    20
#define _F0    21
#define _FS0   23
#define _G0    24
#define _GS0   25
#define _A0    27
#define _AS0   29
#define _B0    30
#define _C1    32
#define _CS1   34
#define _D1    36
#define _DS1   38
#define _E1    41
#define _F1    43
#define _FS1   46
#define _G1    49
#define _GS1   51
#define _A1    55
#define _AS1   58
#define _B1    61
#define _C2    65
#define _CS2   69
#define _D2    73
#define _DS2   77
#define _E2    82
#define _F2    87
#define _FS2   92
#define _G2    98
#define _GS2   103
#define _A2    110
#define _AS2   116
#define _B2    123
#define _C3    130
#define _CS3   138
#define _D3    146
#define _DS3   155
#define _E3    164
#define _F3    174
#define _FS3   185
#define _G3    196
#define _GS3   207
#define _A3    220
#define _AS3   233
#define _B3    246
#define _C4    261
#define _CS4   277
#define _D4    293
#define _DS4   311
#define _E4    329
#define _F4    349
#define _FS4   369
#define _G4    392
#define _GS4   415
#define _A4    440
#define _AS4   466
#define _B4    493
#define _C5    523
#define _CS5   554
#define _D5    587
#define _DS5   622
#define _E5    659
#define _F5    698
#define _FS5   739
#define _G5    783
#define _GS5   830
#define _A5    880
#define _AS5   932
#define _B5    987
#define _C6    1046
#define _CS6   1108
#define _D6    1174
#define _DS6   1244
#define _E6    1318
#define _F6    1396
#define _FS6   1479
#define _G6    1567
#define _GS6   1661
#define _A6    1760
#define _AS6   1864
#define _B6    1975
#define _C7    2093
#define _CS7   2217
#define _D7    2349
#define _DS7   2489
#define _E7    2637
#define _F7    2793
#define _FS7   2959
#define _G7    3135
#define _GS7   3322
#define _A7    3520
#define _AS7   3729
#define _B7    3951
#define _C8    4186
#define _CS8   4434
#define _D8    4698
#define _DS8   4978
#define _E8    5274
#define _F8    5587
#define _FS8   5919
#define _G8    6271
#define _GS8   6644
#define _A8    7040
#define _AS8   7458
#define _B8    7902
#define BPM    60
#define _R     0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t play = 0; // meldoy play index
uint16_t interrupted; // used to break out of HAL_Delay
volatile uint16_t change_melody = 0; // used to indicate a melody change


uint16_t melody_1 [][2] = {{_C4, 4}, {_G3, 8}, {_G3, 8}, {_C3, 4}, {_G3, 4}, {_B3, 4}, {_C4, 4}, {0, 1}};

uint16_t melody_2 [][2] = {{_C4, 8}, {_G3, 4}, {_G3, 4}, {_C3, 8}, {_G3, 8}, {_B3, 8}, {_C4, 8}, {0, 1}};

uint16_t melody_3 [][2] = {{_C3, 1}, {_G3, 4}, {_A2, 1}, {_C3, 4}, {_C3, 8}, {_C3, 8}, {0, 1}};

uint16_t melody_4 [][2] = {{_B3, 2}, {_C3, 6}, {_D2, 2}, {_C3, 4}, {_C3, 8}, {_C3, 2}, {0, 1}};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//check if the right pin is set
	if(GPIO_Pin == BTN_PIN_Pin) {
		HAL_TIM_Base_Start_IT(&htim1);

	}

	// if not do nothing
	else{
		__NOP();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


	HAL_TIM_Base_Stop_IT(&htim1);

	// check if the button is still pressed
	if(HAL_GPIO_ReadPin(GPIOA, BTN_PIN_Pin)) {

		//change the song
		if(play < 4) play += 1;
		else play = 0;

		// set variables to break out of the song loop 
		interrupted = 1;
		change_melody = 1;
	}
}



//*************** LED HANDLE FUNCTIONS START **********************************//
void reset_LED_Pins() {
	/*
	 *  This function resets the Pins of both LED's
	 */

	HAL_GPIO_WritePin(GPIOB, LED_LEFT_Pin, RESET);
	HAL_GPIO_WritePin(GPIOB, LED_RIGHT_Pin, RESET);
}



void led_rythem(uint16_t index) {
	/*
	 * This helper function handles the rythem of the LED's to match the melody
	 */

	if(index == 0) {
		// turn on the left LED and turn off the right LED
		HAL_GPIO_WritePin(GPIOB, LED_LEFT_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LED_RIGHT_Pin, RESET);
	}


	if(index == 1) {
		// turn on the right LED and turn of the left LED
		HAL_GPIO_WritePin(GPIOB, LED_RIGHT_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LED_LEFT_Pin, RESET);
	}
}

//*************** LED HANDLE FUNCTIONS END **********************************//



// ************* PWM HELPER FUNTIONS START **********************************//
static void tone_set(uint16_t freq) {

	/*
	 * this function takes the frequency and sets the Period and Dutycycle
	 *
	 */

	uint16_t period = CFREQ / (freq); // determine the period


	__HAL_TIM_SET_AUTORELOAD(&htim2, period); // change the ARR to the period value
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (period / 2)); // change the dutycycle
	__HAL_TIM_SET_COUNTER(&htim2, 0); // reset the counter to 0
}



void tone_start(uint16_t play_duration) {
	/*
	 * This function starts to play a tone and waits for a given amount of time
	 *
	 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // start PWM (play the tone)
	HAL_Delay(play_duration); // play the note for a duration
}



void tone_stop(uint16_t play_duration) {
	/*
	 * This function stops the playing of a tone and waits for 10 ms
	 */

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // stop the PWM (stop playing tone)
	HAL_Delay(10); // wait for a short duration to better distinguish between notes
}
// ************* PWM HELPER FUNTIONS END **********************************//



// ************* MELODY PLAYING FUNTIONS START ***************************//
uint16_t note_duration(uint16_t note_length) {

	/*
	 * this function takes the note length and returns the time it should be played in ms
	 *
	 * therefore BPM(beats per minute) gets divided by 60 which gives us the beats per second
	 * then it gets multiplied by 1000 in order to get the time in  ms
	 * and then divided by the note length which gives the time the note is played in ms
	 */

	return (((BPM / 60) * 1000) / note_length);
}



void play_song(uint16_t melody[][2]) {

	/*
	 * this function takes a melody as an input and plays it
	 *
	 * it also checks whether the button to change the melody has been pressed
	 */



	uint16_t i = 0; // used to index the notes in the melody
	uint16_t j = 0; // used to alternate the LED's


	while(melody[i][0] != 0 && change_melody == 0) {
		/*
		 * check if the break button has been pressed to change the melody or
		 * if the end of the melody has been reached
		 */

		if(j > 1) j = 0; //


		uint16_t note_freq = melody[i][0]; // get the note frequency
		uint16_t note_dur =  note_duration(melody[i][1]); // get the note duration


		tone_set(note_freq); // the set note


		led_rythem(j); // change the LED's to match the melody


		tone_start(note_dur); // start playing the tone for the duration
		tone_stop(note_dur); // stop playing the tone


		j++;
		i++;
	}


	reset_LED_Pins(); // reset the LED's after the melody has been played


	if(change_melody == 1) change_melody = 0; // change it back to 0 if the button has been pressed


	HAL_Delay(1000); // to wait between every melody
}



void choose_melody() {

	/*
	 * This function decides which melody is going to be played
	 *
	 * if play == 0 no song will be played
	 */

	if(play == 1) play_song(melody_1);
	if(play == 2) play_song(melody_2);
	if(play == 3) play_song(melody_3);
	if(play == 4) play_song(melody_4);

}
// ************* MELODY PLAYING FUNTIONS END ***************************//

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  choose_melody();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LED_RIGHT_Pin|LED_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_PIN_Pin */
  GPIO_InitStruct.Pin = BTN_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LED_RIGHT_Pin LED_LEFT_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LED_RIGHT_Pin|LED_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
