/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

#define MSG_SIZE 64
#define TIMEOUT_1_SEC 10000

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define WRITE_REG32(REG, VAL)   (*(uint32_t*)(REG) = (uint32_t)(VAL))
#define READ_REG32(REG)         (*(uint32_t*)(REG))

/* USER CODE END EM */

/* Exported global variables -------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

extern char msg[MSG_SIZE];
extern uint16_t msg_size;


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void print(const char *text);
void print1(const char *text, uint8_t x );
void print2(const char *text, uint8_t x, uint8_t y);
void printstr(const char *text, uint8_t * data);
void beep(uint32_t duration_ms, uint8_t pitch);
void Delay_ms(uint32_t delay_ms);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define SCOPE_TRIGGER2_Pin GPIO_PIN_3
#define SCOPE_TRIGGER2_GPIO_Port GPIOC
#define STLINK_TX_Pin GPIO_PIN_2
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_3
#define STLINK_RX_GPIO_Port GPIOA
#define CAP_DRIVE_Pin GPIO_PIN_6
#define CAP_DRIVE_GPIO_Port GPIOC
#define CAP_SENSE_Pin GPIO_PIN_8
#define CAP_SENSE_GPIO_Port GPIOC
#define BUZZ_N_Pin GPIO_PIN_8
#define BUZZ_N_GPIO_Port GPIOA
#define BUZZ_P_Pin GPIO_PIN_9
#define BUZZ_P_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SCOPE_TRIGGER1_Pin GPIO_PIN_10
#define SCOPE_TRIGGER1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
