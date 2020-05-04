/**
  ******************************************************************************
  * @file    
  * @author  
  * @brief   Header file of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

    typedef struct
    {
        float Kp;      /* Коэфициент пропорциональной составляющей*/
        float Ki;      /* Коэфициент интегральной составляющей*/
        float Kd;      /* Коэфициент дифференциальной составляющей*/
        int32_t epsilon; /* Ошибка рассогласования*/
        int32_t epsilonPrev; /* Ошибка рассогласования предыдущая*/
        int32_t integral; /*Интеграл ошибки рассогласования*/
        uint16_t dt;      /* Шаг времени*/

        int16_t ProcessVal; /* Текущее значение датчика*/

        int16_t SetPoint; /* Уставка*/

        float ManipulVal; /* Управляющий сигнал (от 0 до 100%)*/
        uint8_t DirOfRot; /*Направление вращения. Если 0 - прямое, > 0 - обратное*/

        TIM_HandleTypeDef *htim; /* Таймер для отсчета времени*/

    }pidTypeDef;

void pidUpdate(pidTypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */