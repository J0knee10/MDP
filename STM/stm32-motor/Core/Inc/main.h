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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM9_CH1_DCMOTOR_B_Pin GPIO_PIN_5
#define TIM9_CH1_DCMOTOR_B_GPIO_Port GPIOE
#define TIM9_CH2_DCMOTOR_B_Pin GPIO_PIN_6
#define TIM9_CH2_DCMOTOR_B_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define TIM12_CH1_SERVO_Pin GPIO_PIN_14
#define TIM12_CH1_SERVO_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RES_Pin GPIO_PIN_12
#define OLED_RES_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define TIM8_CH2_ULTRASONIC_Pin GPIO_PIN_7
#define TIM8_CH2_ULTRASONIC_GPIO_Port GPIOC
#define ULTRASONIC_SENSOR_Pin GPIO_PIN_8
#define ULTRASONIC_SENSOR_GPIO_Port GPIOC
#define TIM2_CH1_ENCODER_A_Pin GPIO_PIN_15
#define TIM2_CH1_ENCODER_A_GPIO_Port GPIOA
#define TIM2_CH2_ENCODER_A_Pin GPIO_PIN_3
#define TIM2_CH2_ENCODER_A_GPIO_Port GPIOB
#define TIM3_CH1_ENCODER_B_Pin GPIO_PIN_4
#define TIM3_CH1_ENCODER_B_GPIO_Port GPIOB
#define TIM3_CH2_ENCODER_B_Pin GPIO_PIN_5
#define TIM3_CH2_ENCODER_B_GPIO_Port GPIOB
#define TIM4_CH3_DCMOTOR_A_Pin GPIO_PIN_8
#define TIM4_CH3_DCMOTOR_A_GPIO_Port GPIOB
#define TIM4_CH4_DCMOTOR_A_Pin GPIO_PIN_9
#define TIM4_CH4_DCMOTOR_A_GPIO_Port GPIOB
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOE
#define USER_BTN_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
