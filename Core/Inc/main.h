/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define ANO   1								/* 匿名地面站 */
#define VOFA  2								/* 伏特加地面站 */
#define TRANSFER_TYPE VOFA		/* 通信方式选择 */

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
#define BLINK_LED_Pin GPIO_PIN_13
#define BLINK_LED_GPIO_Port GPIOC
#define MOTOR_EN1_Pin GPIO_PIN_14
#define MOTOR_EN1_GPIO_Port GPIOC
#define MOTOR_DIR1_Pin GPIO_PIN_15
#define MOTOR_DIR1_GPIO_Port GPIOC
#define TIM2_ENCODER1_A_Pin GPIO_PIN_0
#define TIM2_ENCODER1_A_GPIO_Port GPIOA
#define TIM2_ENCODER1_B_Pin GPIO_PIN_1
#define TIM2_ENCODER1_B_GPIO_Port GPIOA
#define USART2_BLUETOOT_TX_Pin GPIO_PIN_2
#define USART2_BLUETOOT_TX_GPIO_Port GPIOA
#define USART2_BLUETOOTH_RX_Pin GPIO_PIN_3
#define USART2_BLUETOOTH_RX_GPIO_Port GPIOA
#define VBAT_AD_Pin GPIO_PIN_4
#define VBAT_AD_GPIO_Port GPIOA
#define TIM3_ENCODER2_A_Pin GPIO_PIN_6
#define TIM3_ENCODER2_A_GPIO_Port GPIOA
#define TIM3_ENCODER2_B_Pin GPIO_PIN_7
#define TIM3_ENCODER2_B_GPIO_Port GPIOA
#define MOTOR_EN2_Pin GPIO_PIN_0
#define MOTOR_EN2_GPIO_Port GPIOB
#define MOTOR_DIR2_Pin GPIO_PIN_1
#define MOTOR_DIR2_GPIO_Port GPIOB
#define USART3_DEBUG_TX_Pin GPIO_PIN_10
#define USART3_DEBUG_TX_GPIO_Port GPIOB
#define USART3_DEBUG_RX_Pin GPIO_PIN_11
#define USART3_DEBUG_RX_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_14
#define AIN2_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_15
#define STBY_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_3
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
