/**
  ******************************************************************************
  * @file    
  * @author  
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VBAT_H
#define __VBAT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

    typedef struct
    {
        SDADC_HandleTypeDef *hsdadc; /*Указатель на хэндл ацп которое измеряет напряжение*/
        uint32_t channel;            /* Канал АЦП которые измеряет напряжение*/
        float k;
        float b; /* voltage = k*adc_value+b*/
    } vbatTypeDef;

    float GetVoltageBat(vbatTypeDef *vbat); /*Возвращает напряжение в Вольтах*/

#ifdef __cplusplus
}
#endif

#endif /*__VBAT */