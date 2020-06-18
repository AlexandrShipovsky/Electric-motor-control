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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum modes
{
  TrackingState = 1,
  TestState = 2,
  CLIState = 3
};
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define M2_EN_Pin GPIO_PIN_14
#define M2_EN_GPIO_Port GPIOC
#define M1_EN_Pin GPIO_PIN_15
#define M1_EN_GPIO_Port GPIOC
#define M22_PWM_Pin GPIO_PIN_0
#define M22_PWM_GPIO_Port GPIOA
#define M21_PWM_Pin GPIO_PIN_1
#define M21_PWM_GPIO_Port GPIOA
#define M12_PWM_Pin GPIO_PIN_2
#define M12_PWM_GPIO_Port GPIOA
#define M11_PWM_Pin GPIO_PIN_3
#define M11_PWM_GPIO_Port GPIOA
#define ENCODER_1M_Pin GPIO_PIN_4
#define ENCODER_1M_GPIO_Port GPIOA
#define ENCODER_1P_Pin GPIO_PIN_6
#define ENCODER_1P_GPIO_Port GPIOA
#define M22_SDADC_Pin GPIO_PIN_0
#define M22_SDADC_GPIO_Port GPIOB
#define M21_SDADC_Pin GPIO_PIN_1
#define M21_SDADC_GPIO_Port GPIOB
#define M12_SDADC_Pin GPIO_PIN_2
#define M12_SDADC_GPIO_Port GPIOB
#define M11_SDADC_Pin GPIO_PIN_8
#define M11_SDADC_GPIO_Port GPIOE
#define BAT_SDADC_Pin GPIO_PIN_8
#define BAT_SDADC_GPIO_Port GPIOD
#define ENCODER_2P_Pin GPIO_PIN_6
#define ENCODER_2P_GPIO_Port GPIOB
#define ENCODER_2M_Pin GPIO_PIN_7
#define ENCODER_2M_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
