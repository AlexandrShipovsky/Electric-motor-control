/**
  ******************************************************************************
  * @file    MotorDC.h
  * @author  ClearSky
  * @brief   Header file of DC motor.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
#ifndef MotorDC_H
#define MotorDC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f3xx_hal.h"
#include "main.h"

#define DIRECT_ROTATION (uint8_t)0
#define REVERSE_ROTATION (uint8_t)1
  /**
  * @brief  MotorDC init structure definition
  */
  typedef struct
  {
    GPIO_TypeDef *port;        /*Порт пина разрешающего работу H-моста*/
    uint16_t pin;              /*Пин разрешающий работу H-моста */
    TIM_HandleTypeDef *htim;   /*Указатель на таймер ШИМа*/
    uint32_t PWM_ChannelFirst; /* ШИМ канал первого плеча H-моста */
    uint32_t PWM_ChannelSnd;   /* ШИМ канал второго плеча H-моста */
    float pulse;               /* Ширина импульса ШИМ от 0 до 100 % */
    uint8_t DirOfRot;          /*Направление вращения. Если 0 - прямое, >0 - обратное*/
  } MotorDCTypeDef;

  /* Private function prototypes -----------------------------------------------*/
  /** @addtogroup MotorDC function
  * @{
  */

  void rotation(MotorDCTypeDef *motor);
  void StopRotation(MotorDCTypeDef *motor);

#ifdef __cplusplus
}
#endif

#endif /* MotorDC_H */