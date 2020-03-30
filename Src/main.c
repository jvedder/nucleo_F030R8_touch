/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdio.h>

#define SYSTICK_MASK  (0x80L)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char msg[MSG_SIZE];
uint16_t msg_size;
char TIM3_labels[20][8] = {
        "CR1    ","CR2    ","SMCR   ","DIER   ","SR     ",
        "EGR    ","CCMR1  ","CCMR2  ","CCER   ","CNT    ",
        "PSC    ","ARR    ","RCR    ","CCR1   ","CCR2   ",
        "CCR3   ","CCR4   ","BDTR   ","DCR    ","DMAR   "
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
#if 0
static void Dump_TIM3(void);
#endif

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\nPOR\r\n");
  printf("Build: " __DATE__ ", " __TIME__ "\r\n");


  /* synchronize to SYSTTICK */
  while ( (HAL_GetTick() & SYSTICK_MASK) != 0)
  {
      //printf("T1 %08lX\r\n", HAL_GetTick());
      //spin wait
  }
  printf(".\r\n");
  while ( (HAL_GetTick() & SYSTICK_MASK) == 0)
  {
      //printf("T2 %08lX\r\n", HAL_GetTick());
      //spin wait
  }
  printf(".\r\n");
  while ( (HAL_GetTick() & SYSTICK_MASK) != 0)
  {
      //printf("T3 %08lX\r\n", HAL_GetTick());
      //spin wait
  }
  printf(".\r\n");


  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */

      /* config timer for rising edge */
      TIM3->CR1 = 0x0000;   // disable timer
      TIM3->SR = 0x0000;    // clear interrupt flags
      TIM3->ARR = 0xFFFF;   // autoreload value
      TIM3->CCER = 0x0100;  // CH3 Rising Edge
      TIM3->CNT = 0x0000;   // clear counter
      TIM3->CCR3 = 0x0000;  // clear result
      TIM3->CR1 = 0x0001;   // enable timer

      /* Start Cap Charge & flag Start */
      //printf("H\r\n");
      HAL_GPIO_WritePin(GPIOC, CAP_DRIVE_Pin | SCOPE_TRIGGER1_Pin, GPIO_PIN_SET);
      while ( (TIM3->SR & 0x0008) == 0)
      {
          //spin wait
      }

      /* flag TIM3 Input Capture */
      HAL_GPIO_WritePin(GPIOC, SCOPE_TRIGGER2_Pin, GPIO_PIN_SET);

      /* show results */
      printf("%04X  ", (uint16_t) TIM3->CCR3);

      /* wait for SYSTICK bit high */
      while ( (HAL_GetTick() & SYSTICK_MASK) == 0)
      {
          //spin wait
      }

      /* config timer for falling edge */
      TIM3->CR1 = 0x0000;   // disable timer
      TIM3->SR = 0x0000;    // clear interrupt flags
      TIM3->ARR = 0xFFFF;   // autoreload value
      TIM3->CCER = 0x0300;  // CH3 Falling Edge
      TIM3->CNT = 0x0000;   // clear counter
      TIM3->CCR3 = 0x0000;  // clear result
      TIM3->CR1 = 0x0001;   // enable timer

      /* Start Cap Discharge & flag Start */
      //printf("L\r\n");
      HAL_GPIO_WritePin(GPIOC, CAP_DRIVE_Pin | SCOPE_TRIGGER1_Pin, GPIO_PIN_RESET);
      while ( (TIM3->SR & 0x0008) == 0)
      {
          //spin wait
      }

      /* flag TIM3 Input Capture */
      HAL_GPIO_WritePin(GPIOC, SCOPE_TRIGGER2_Pin, GPIO_PIN_RESET);

      /* show results */
      printf("%04X\r\n", (uint16_t) TIM3->CCR3);

      /* wait for SYSTICK bit low */
      while ( (HAL_GetTick() & SYSTICK_MASK) != 0)
      {
          //spin wait
      }


    /* USER CODE END 2 */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCOPE_TRIGGER2_Pin|CAP_DRIVE_Pin|SCOPE_TRIGGER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_N_Pin|BUZZ_P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCOPE_TRIGGER2_Pin CAP_DRIVE_Pin SCOPE_TRIGGER1_Pin */
  GPIO_InitStruct.Pin = SCOPE_TRIGGER2_Pin|CAP_DRIVE_Pin|SCOPE_TRIGGER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_N_Pin BUZZ_P_Pin */
  GPIO_InitStruct.Pin = BUZZ_N_Pin|BUZZ_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


#if 0
static void Dump_TIM3(void)
{
//    for(int i=0; i<20; i++)
//    {
//        uint32_t reg = READ_REG32(TIM3_BASE+4*i);
//        printf("%s%08lX\r\n", TIM3_labels[i], reg);
//    }
    printf("%s%08lX\r\n", "SR   ", TIM3->SR);
    printf("%s%08lX\r\n", "CNT  ", TIM3->CNT);
    printf("%s%08lX\r\n", "CCR3 ", TIM3->CCR3);
}

static void CAP_DriveMode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAP_DRIVE_GPIO_Port, CAP_DRIVE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CAP_DRIVE_Pin */
  GPIO_InitStruct.Pin = CAP_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAP_DRIVE_GPIO_Port, &GPIO_InitStruct);
}

static void CAP_SenseMode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAP_DRIVE_GPIO_Port, CAP_DRIVE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CAP_DRIVE_Pin */
  GPIO_InitStruct.Pin = CAP_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAP_DRIVE_GPIO_Port, &GPIO_InitStruct);
}
#endif


void print(const char *text)
{
	  sprintf(msg, "%s\r\n", text );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);
}


void print1(const char *text, uint8_t x)
{
	  sprintf(msg, "%s 0x%02X\r\n", text, (int) x );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);
}


void print2(const char *text, uint8_t x, uint8_t y)
{
	  sprintf(msg, "%s 0x%02X 0x%02X\r\n", text, (int) x, (int) y );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);
}


void printstr(const char *text, uint8_t * data)
{
	  sprintf(msg, "%s: %s\r\n", text, (char *) data );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);
}


void printhex(uint8_t x)
{
      sprintf(msg, "%02X", (int) x );
      msg_size = strlen(msg);
      HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);
}


/* set pitch to 0,1,2,3 with 0 highest pitch */
void beep(uint32_t duration_ms, uint8_t pitch)
{
	uint32_t tick;

	uint32_t mask = (1 << pitch);

	uint32_t start = HAL_GetTick();
	do {
		tick = HAL_GetTick();
		if (tick & mask)
		{
			// Piezo/Speaker positive voltage
			HAL_GPIO_WritePin(BUZZ_P_GPIO_Port, BUZZ_P_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BUZZ_N_GPIO_Port, BUZZ_N_Pin, GPIO_PIN_RESET);
		}
		else
		{
			// Piezo/Speaker negative voltage
			HAL_GPIO_WritePin(BUZZ_P_GPIO_Port, BUZZ_P_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BUZZ_N_GPIO_Port, BUZZ_N_Pin, GPIO_PIN_SET);
		}
	} while ((tick - start) < duration_ms);

	// Peizo/Speaker zero voltage
	HAL_GPIO_WritePin(BUZZ_P_GPIO_Port, BUZZ_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZ_N_GPIO_Port, BUZZ_N_Pin, GPIO_PIN_RESET);

	return;
}


void Delay_ms(uint32_t delay_ms)
{
    /**
     * This should correctly handle SysTic roll-overs.
     * See:
     *   https://stackoverflow.com/questions/61443/rollover-safe-timer-tick-comparisons
     */
    uint32_t start_time_ms = HAL_GetTick();
    while ( (HAL_GetTick() - start_time_ms) < delay_ms)
    {
        // spin wait
    }

    return;
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
  while(1)
  {
      beep(100,3);
      Delay_ms(900);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
