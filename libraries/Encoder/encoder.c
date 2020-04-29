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
  int16_t val = -15;
  encoder->DirOfRot = __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim);
  val = __HAL_TIM_GET_COUNTER(encoder->htim);
  val -= encoder->zero;
  return val;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
HAL_StatusTypeDef SetZero(EncTypeDef *encoder)
{
  __HAL_TIM_SET_COUNTER(encoder->htim, encoder->zero);
  if (__HAL_TIM_GET_COUNTER(encoder->htim) == encoder->zero)
  {
    return HAL_OK;
  }
  return HAL_ERROR;
}