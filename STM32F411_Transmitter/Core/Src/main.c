/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Jakob Frabosilio
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DRIVER_STRING_LEN 560  /**< number of instructions to send in one transmission (message_string * on_X_string) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim4_up;

/* USER CODE BEGIN PV */
volatile uint32_t message_int = 0;  /**< integer value to transmit over audio, between 0 and 255 */
volatile char message_char;         /**< 8-bit character representation of message_int */

/**< list of pulses to send for complete message, last 8 bits will be filled with message_char */
volatile uint8_t message_string[] = {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**< list of GPIO ODR instructions to send a 1 bit over audio, positive input to motor driver */
uint8_t on_a_string[] = {
		1, 0,
		1, 0,
		1, 0,
		1, 0,
		1, 0,
		1,
		1, 0,
		1, 0,
		1, 0,
		1, 0,
		1, 0,
		1, 0,
		1,
		1, 0,
		1, 0,
		1, 0,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

/**< list of GPIO ODR instructions to send a o1ne bit over audio, negative input to motor driver */
uint8_t on_b_string[] = {
		0, 1,
		0, 1,
		0, 1,
		0, 1,
		0, 1,
		1,
		0, 1,
		0, 1,
		0, 1,
		0, 1,
		0, 1,
		0, 1,
		1,
		0, 1,
		0, 1,
		0, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

volatile uint32_t driver_string[DRIVER_STRING_LEN];   /**< full list of ODR instructions to send message_string over audio */
volatile uint16_t instruction_counter = 0;            /**< counter to iterate through driver_string */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim5);              // start the 80kHz timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 990-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 800-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Function that is called when a timer updates.
 * @param htim The particular timer that triggers the callback.
 *
 * This function has two purposes.
 *
 * First, every 1000ms (when TIM5 updates), a new message_string is generated. This message is generated as
 * four 1 bits, two 0 bits, and then an 8-bit character. The four 1 bits and two 0 bits are a header for the
 * message; they alert the receiver that a character is about to be sent.
 *
 * Transmitting a single 1 bit requires a fairly complex signal to be sent. Because this system works at the
 * resonant frequency of the transducers (to increase the range of the system), sending a single 40kHz pulse
 * would produce significant ringing in the receiver. This system, inspired by the paper below, sends the
 * following sequence for a 1 bit in the message string:
 *    - Five in-phase waves
 *    - Six out-of-phase waves
 *    - Three in-phase waves
 *
 * After generating the message string, a "driver string" is generated. This is a list of instructions to
 * send to the motor driver; when TIM4 updates at 80kHz, the GPIO ODR (output data register, sets a particular
 * GPIO port) is set to the next value of the driver_string. Essentially, for a 1 bit in the message_string,
 * the STM32 alternates two GPIO pins (which are connected to the inputs of a motor driver) to produce the
 * waveforms in the list above. For a 0 bit, it sets both GPIO pins to low. Generating this driver_string
 * beforehand means that the timing of the motor driver can be much more precise, as it just has to follow
 * a list of instructions.
 *
 * @see https://iopscience.iop.org/article/10.1088/1361-6463/abd582
 *
 * (In a full implementation, these individual characters would be replaced by characters from a GPS string.
 * A buoy on the surface of the water would have a GPS receiver and acoustic transmitter, and would send the
 * GPS position of the buoy to an AUV (or ROV). The underwater vehicle can then record the full GPS message
 * and decode it, and use parts of it to determine its position relative to the buoy. Read my thesis for the
 * full explanation.)
 *
 * Second, every 12.5us (when TIM4 updates), the GPIO ODR is set to the next value of driver_string. This
 * iterates through the list of instructions generated when TIM5 updates.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5){																			// every 1000 ms...
		message_int ++;                                                 // pick the next number
//		HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t *) &message_int);	// OR generate a random number...
		message_int %= 256;																							// between 0 and 255 (8-bit length),
		message_char = (char)message_int;																// and save it as a char
		for(int i = 8; i > 0; i--){																			// convert that integer...
			message_string[14-i] = (message_int >> (i-1)) & 1;						// to a bit array and save it in the last 8 bits of message_string
		}
		message_string[0] = 1;                                          // initialize the message with a header bit
		message_string[1] = 1;
		message_string[2] = 1;
		message_string[3] = 1;
		message_string[4] = 0;
		message_string[5] = 0;
		int index = 0;																									// generate driver_string...
		for (int i = 0; i < sizeof(message_string); i++) {
			if (message_string[i] == 1) {                                 // if the next bit of message_string is a 1,
				for (int j = 0; j < sizeof(on_a_string); j++) {             // write the values from on_a/b_string to the driver_string
				  driver_string[index] = (on_a_string[j] << 6) | (on_b_string[j] << 7);
					index++;
				}
			} else {                                                      // if the next bit of message_string is a 0,
				for (int j = 0; j < sizeof(on_a_string); j++) {             // write all zeros to the driver string
				  driver_string[index] = 0;
					index++;
				}
			}
		}
		instruction_counter = 0;                                        // start the instruction counter at 0
		HAL_TIM_Base_Start_IT(&htim4);                                  // start TIM4, which starts setting the GPIO ODR to driver_string
		__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);									// reset TIM5 interrupt
	}

	if (htim->Instance == TIM4){                                      // once started, every 12.5us...
		GPIOB->ODR = driver_string[instruction_counter];                // set the ODR of GPIOB to the next value of driver_string (controls motor driver pos/neg)
		instruction_counter++;                                          // increment the counter
		if (instruction_counter >= DRIVER_STRING_LEN){                  // if all of driver_string has been sent,
			instruction_counter = 0;                                      // reset the instruction counter
			HAL_TIM_Base_Stop(&htim4);                                    // stop this timer
		}
	}
}
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
