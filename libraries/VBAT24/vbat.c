/**
  ******************************************************************************
  * @file    
  * @author  
  * @brief   
*/

#include "vbat.h"

float GetVoltageBat(vbatTypeDef *vbat) /*Возвращает напряжение в Вольтах*/
{
    if (HAL_SDADC_PollForInjectedConversion(vbat->hsdadc, 1000) != HAL_OK)
    {
        return 5.55;
    };
    return (vbat->k * (float)(HAL_SDADC_InjectedGetValue(vbat->hsdadc, &vbat->channel)) + vbat->b);
}