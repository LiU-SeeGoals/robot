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
#include "stm32h7xx_hal.h"

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
#define MOTOR4_BREAK_Pin GPIO_PIN_2
#define MOTOR4_BREAK_GPIO_Port GPIOE
#define BTN_USER_Pin GPIO_PIN_13
#define BTN_USER_GPIO_Port GPIOC
#define BTN_USER_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR1_ENCODER_Pin GPIO_PIN_6
#define MOTOR1_ENCODER_GPIO_Port GPIOF
#define MOTOR2_ENCODER_Pin GPIO_PIN_10
#define MOTOR2_ENCODER_GPIO_Port GPIOF
#define MOTOR3_ENCODER_Pin GPIO_PIN_2
#define MOTOR3_ENCODER_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_5
#define NRF_SCK_GPIO_Port GPIOA
#define NRF_MISO_Pin GPIO_PIN_6
#define NRF_MISO_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define MOTOR1_BREAK_Pin GPIO_PIN_2
#define MOTOR1_BREAK_GPIO_Port GPIOB
#define MOTOR3_REVERSE_Pin GPIO_PIN_7
#define MOTOR3_REVERSE_GPIO_Port GPIOE
#define MOTOR2_REVERSE_Pin GPIO_PIN_8
#define MOTOR2_REVERSE_GPIO_Port GPIOE
#define MOTOR1_PWM_Pin GPIO_PIN_9
#define MOTOR1_PWM_GPIO_Port GPIOE
#define MOTOR2_PWM_Pin GPIO_PIN_11
#define MOTOR2_PWM_GPIO_Port GPIOE
#define MOTOR3_PWM_Pin GPIO_PIN_13
#define MOTOR3_PWM_GPIO_Port GPIOE
#define MOTOR4_PWM_Pin GPIO_PIN_14
#define MOTOR4_PWM_GPIO_Port GPIOE
#define MOTOR4_REVERSE_Pin GPIO_PIN_15
#define MOTOR4_REVERSE_GPIO_Port GPIOE
#define KICKER_DISCHARGE_Pin GPIO_PIN_10
#define KICKER_DISCHARGE_GPIO_Port GPIOB
#define KICKER_CHARGE_Pin GPIO_PIN_11
#define KICKER_CHARGE_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_15
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR3_BREAK_Pin GPIO_PIN_12
#define MOTOR3_BREAK_GPIO_Port GPIOD
#define MOTOR2_BREAK_Pin GPIO_PIN_13
#define MOTOR2_BREAK_GPIO_Port GPIOD
#define MOTOR4_ENCODER_Pin GPIO_PIN_6
#define MOTOR4_ENCODER_GPIO_Port GPIOG
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOC
#define SPI_CS_OUTPUT_Pin GPIO_PIN_3
#define SPI_CS_OUTPUT_GPIO_Port GPIOD
#define SPI_CS_RESET_Pin GPIO_PIN_4
#define SPI_CS_RESET_GPIO_Port GPIOD
#define SPI_CS_STORE_Pin GPIO_PIN_5
#define SPI_CS_STORE_GPIO_Port GPIOD
#define SPI_CS_SHIFT_Pin GPIO_PIN_6
#define SPI_CS_SHIFT_GPIO_Port GPIOD
#define SPI_CS_CONF_Pin GPIO_PIN_7
#define SPI_CS_CONF_GPIO_Port GPIOD
#define SPI_MOSI_Pin GPIO_PIN_5
#define SPI_MOSI_GPIO_Port GPIOB
#define MOTOR1_REVERSE_Pin GPIO_PIN_6
#define MOTOR1_REVERSE_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_8
#define NRF_CSN_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define MOTOR1_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR2_TIM_CHANNEL TIM_CHANNEL_2
#define MOTOR3_TIM_CHANNEL TIM_CHANNEL_3
#define MOTOR4_TIM_CHANNEL TIM_CHANNEL_4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
