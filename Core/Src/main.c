/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_7

#define R2_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_6

#define R3_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_5

#define R4_GPIO_Port GPIOA
#define R4_Pin GPIO_PIN_4


#define C1_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_3

#define C2_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_2

#define C3_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_1

#define C4_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char read_keypad (void)
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		return 'D';
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		return 'C';
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		return 'B';
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		return 'A';
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		return '#';
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		return '9';
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		return '6';
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		return '3';
	}


	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		return '0';
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		return '8';
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		return '5';
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		return '2';
	}


	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);  // Pull the R4 High

	if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
		return '*';
	}

	if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
		return '7';
	}

	if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
		return '4';
	}

	if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
		return '1';
	}

	return 0x01;  // /if nothing is pressed

}


uint8_t key;
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	SSD1306_Init();
  char snum[5];
	// Knum to hold the key
	char knum[5];

  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("Distance cm", &Font_11x18, 1);


	//IR sensor code
	HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim1);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim1);

    Distance = (Value2-Value1)* 0.034/2;
    
		key = read_keypad ();
		if(key!= 0x01)
		{
				SSD1306_Clear();
				sprintf(knum, "%c", key);
				SSD1306_GotoXY (0, 0);
				SSD1306_Puts ("Key pressed:", &Font_11x18, 1);
				SSD1306_GotoXY (0, 30);
				SSD1306_Puts (knum, &Font_11x18, 1);
				SSD1306_UpdateScreen();
				HAL_Delay(800);
					
				SSD1306_GotoXY (0,0);
				SSD1306_Puts ("Distance cm", &Font_11x18, 1);
		}
		
		// key is ascii value. So -48 to convert to number
		if(Distance< (key-48)){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			
		}
		else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
		}
		
			sprintf(snum, "%d", Distance); 
			SSD1306_GotoXY (0, 30);
			SSD1306_Puts ("             ", &Font_16x26, 1);
			SSD1306_UpdateScreen();
			if(Distance < 10) {
				SSD1306_GotoXY (53, 30);  // 1 DIGIT
			}
			else if (Distance < 100 ) {
				SSD1306_GotoXY (45, 30);  // 2 DIGITS
			}
			else if (Distance < 1000 ) {
				SSD1306_GotoXY (37, 30);  // 3 DIGITS
			}
			else {
				SSD1306_GotoXY (30, 30);  // 4 DIGIS
			}
			SSD1306_Puts (snum, &Font_16x26, 1);
			
		
			SSD1306_UpdateScreen();
			HAL_Delay (100);
	  //}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_8;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
