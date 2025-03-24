/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_1_Pin GPIO_PIN_3
#define BUTTON_1_GPIO_Port GPIOH
#define BUTTON_1_EXTI_IRQn EXTI3_IRQn
#define ENCODER_RIGHT_A_Pin GPIO_PIN_0
#define ENCODER_RIGHT_A_GPIO_Port GPIOA
#define ENCODER_RIGHT_B_Pin GPIO_PIN_1
#define ENCODER_RIGHT_B_GPIO_Port GPIOA
#define IR_FAR_RIGHT_COLLECT_Pin GPIO_PIN_2
#define IR_FAR_RIGHT_COLLECT_GPIO_Port GPIOA
#define IR_MID_RIGHT_COLLECT_Pin GPIO_PIN_3
#define IR_MID_RIGHT_COLLECT_GPIO_Port GPIOA
#define IR_MID_LEFT_COLLECT_Pin GPIO_PIN_4
#define IR_MID_LEFT_COLLECT_GPIO_Port GPIOA
#define IR_FAR_LEFT_COLLECT_Pin GPIO_PIN_5
#define IR_FAR_LEFT_COLLECT_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOA
#define VOLTAGE_IN_Pin GPIO_PIN_7
#define VOLTAGE_IN_GPIO_Port GPIOA
#define MOTOR_LEFT_Pin GPIO_PIN_8
#define MOTOR_LEFT_GPIO_Port GPIOA
#define MOTOR_RIGHT_Pin GPIO_PIN_9
#define MOTOR_RIGHT_GPIO_Port GPIOA
#define MOTOR_RIGHT_DIR_Pin GPIO_PIN_2
#define MOTOR_RIGHT_DIR_GPIO_Port GPIOB
#define IR_FAR_LEFT_EMIT_Pin GPIO_PIN_0
#define IR_FAR_LEFT_EMIT_GPIO_Port GPIOB
#define IR_MID_LEFT_EMIT_Pin GPIO_PIN_1
#define IR_MID_LEFT_EMIT_GPIO_Port GPIOB
#define IR_MID_RIGHT_EMIT_Pin GPIO_PIN_4
#define IR_MID_RIGHT_EMIT_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOA
#define MOTOR_LEFT_DIR_Pin GPIO_PIN_12
#define MOTOR_LEFT_DIR_GPIO_Port GPIOA
#define BUTTON_2_Pin GPIO_PIN_15
#define BUTTON_2_GPIO_Port GPIOA
#define BUTTON_2_EXTI_IRQn EXTI15_10_IRQn
#define IR_FAR_RIGHT_EMIT_Pin GPIO_PIN_4
#define IR_FAR_RIGHT_EMIT_GPIO_Port GPIOB
#define ENCODER_LEFT_B_Pin GPIO_PIN_5
#define ENCODER_LEFT_B_GPIO_Port GPIOB
#define ENCODER_LEFT_A_Pin GPIO_PIN_7
#define ENCODER_LEFT_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
