/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define CAN_HEARTBEAT_LED_Pin GPIO_PIN_6
#define CAN_HEARTBEAT_LED_GPIO_Port GPIOA
#define UART_ACTIVITY_LED_Pin GPIO_PIN_7
#define UART_ACTIVITY_LED_GPIO_Port GPIOA
#define BTN_SEG_ID_DOWN_Pin GPIO_PIN_7
#define BTN_SEG_ID_DOWN_GPIO_Port GPIOC
#define BTN_SEG_ID_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define BTN_SERVO_ID_UP_Pin GPIO_PIN_8
#define BTN_SERVO_ID_UP_GPIO_Port GPIOA
#define BTN_SERVO_ID_UP_EXTI_IRQn EXTI9_5_IRQn
#define BTN_SERVO_ID_DOWN_Pin GPIO_PIN_9
#define BTN_SERVO_ID_DOWN_GPIO_Port GPIOA
#define BTN_SERVO_ID_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define BTN_SEG_ID_UP_Pin GPIO_PIN_10
#define BTN_SEG_ID_UP_GPIO_Port GPIOA
#define BTN_SEG_ID_UP_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BTN_SERVO_ANGLE_DOWN_Pin GPIO_PIN_4
#define BTN_SERVO_ANGLE_DOWN_GPIO_Port GPIOB
#define BTN_SERVO_ANGLE_DOWN_EXTI_IRQn EXTI4_IRQn
#define BTN_SERVO_ANGLE_UP_Pin GPIO_PIN_5
#define BTN_SERVO_ANGLE_UP_GPIO_Port GPIOB
#define BTN_SERVO_ANGLE_UP_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
