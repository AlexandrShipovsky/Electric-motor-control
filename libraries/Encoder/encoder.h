/**
  ******************************************************************************
  * @file    encoder.h
  * @author  ClearSky
  * @brief   Header file of incremental encoder.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/*  -------------------------------------*/
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "main.h"

/**
  * @brief  Encoder init structure definition
  */
typedef struct 
{
    TIM_HandleTypeDef *htim;   /*Указатель на таймер энкодера*/
    uint16_t channel; /*Канал таймера по которому идет счет*/
    uint32_t zero; /* Значение регистра CNT в нуле*/
    uint8_t DirOfRot; /*Направление вращения. 0 - прямое (значение увеличивается), 1 - обратное*/

}EncTypeDef;

 /* Private function prototypes -----------------------------------------------*/
  /** @addtogroup MotorDC function
  * @{
  */

 int16_t GetEnc(EncTypeDef *encoder);
 HAL_StatusTypeDef SetZero(EncTypeDef *encoder);
 

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */