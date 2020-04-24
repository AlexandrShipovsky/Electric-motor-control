/**
  ******************************************************************************
  * @file    encoder.c
  * @author  ClearSky
  * @brief   
  */
/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/**
  * @brief  
  * @param  
  * @retval None
  */
  int16_t GetEnc(EncTypeDef *encoder)
  {
    //Дополнить направлением вращения
      int16_t val = -15;
      val = __HAL_TIM_GET_COUNTER(encoder->htim);
      val-=encoder->zero;
      return val;
  }

  /**
  * @brief  
  * @param  
  * @retval None
  */
 HAL_StatusTypeDef SetZero(EncTypeDef *encoder)
 {
     __HAL_TIM_SET_COUNTER(encoder->htim,encoder->zero);
     if(__HAL_TIM_GET_COUNTER(encoder->htim) == encoder->zero)
     {
         return HAL_OK;
     }
     return HAL_ERROR;
 }