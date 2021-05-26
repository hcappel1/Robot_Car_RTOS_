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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ENC3_Pin GPIO_PIN_2
#define ENC3_GPIO_Port GPIOC
#define ENC3_EXTI_IRQn EXTI2_IRQn
#define ENC4_Pin GPIO_PIN_3
#define ENC4_GPIO_Port GPIOC
#define ENC4_EXTI_IRQn EXTI3_IRQn
#define ECHO_4_Pin GPIO_PIN_0
#define ECHO_4_GPIO_Port GPIOA
#define ECHO_5_Pin GPIO_PIN_1
#define ECHO_5_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TRIG_5_Pin GPIO_PIN_4
#define TRIG_5_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TRIG_3_Pin GPIO_PIN_6
#define TRIG_3_GPIO_Port GPIOA
#define ECHO_2_Pin GPIO_PIN_7
#define ECHO_2_GPIO_Port GPIOA
#define TRIG_2_Pin GPIO_PIN_4
#define TRIG_2_GPIO_Port GPIOC
#define TRIG_4_Pin GPIO_PIN_5
#define TRIG_4_GPIO_Port GPIOC
#define TRIG_1_Pin GPIO_PIN_0
#define TRIG_1_GPIO_Port GPIOB
#define ECHO_3_Pin GPIO_PIN_1
#define ECHO_3_GPIO_Port GPIOB
#define BIN2_2_Pin GPIO_PIN_2
#define BIN2_2_GPIO_Port GPIOB
#define BIN1_2_Pin GPIO_PIN_10
#define BIN1_2_GPIO_Port GPIOB
#define STBY2_Pin GPIO_PIN_12
#define STBY2_GPIO_Port GPIOB
#define AIN1_2_Pin GPIO_PIN_13
#define AIN1_2_GPIO_Port GPIOB
#define AIN2_2_Pin GPIO_PIN_14
#define AIN2_2_GPIO_Port GPIOB
#define BIN2_1_Pin GPIO_PIN_15
#define BIN2_1_GPIO_Port GPIOB
#define BIN1_1_Pin GPIO_PIN_6
#define BIN1_1_GPIO_Port GPIOC
#define STBY_1_Pin GPIO_PIN_7
#define STBY_1_GPIO_Port GPIOC
#define AIN1_1_Pin GPIO_PIN_8
#define AIN1_1_GPIO_Port GPIOC
#define AIN2_1_Pin GPIO_PIN_9
#define AIN2_1_GPIO_Port GPIOC
#define PWMB_2_Pin GPIO_PIN_8
#define PWMB_2_GPIO_Port GPIOA
#define PWMA_2_Pin GPIO_PIN_9
#define PWMA_2_GPIO_Port GPIOA
#define PWMB_1_Pin GPIO_PIN_10
#define PWMB_1_GPIO_Port GPIOA
#define PWMA_1_Pin GPIO_PIN_11
#define PWMA_1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ECHO_1_Pin GPIO_PIN_4
#define ECHO_1_GPIO_Port GPIOB
#define ENC1_Pin GPIO_PIN_5
#define ENC1_GPIO_Port GPIOB
#define ENC1_EXTI_IRQn EXTI9_5_IRQn
#define ENC2_Pin GPIO_PIN_6
#define ENC2_GPIO_Port GPIOB
#define ENC2_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
