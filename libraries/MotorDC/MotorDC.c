#include "MotorDC.h"

/**
  * @brief  Rotation motor.
  * @param  motor is motor handle
  * @retval None
  */
void rotation(MotorDCTypeDef *motor)
{
    uint32_t pulse;

    if ((motor->pulse > 100.0) | (motor->pulse < 0.0))
    {
        return;
    }

    HAL_GPIO_WritePin(motor->port, motor->pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(motor->htim, motor->PWM_ChannelFirst);
    HAL_TIM_PWM_Stop(motor->htim, motor->PWM_ChannelSnd);
    pulse = (uint32_t)((motor->htim->Init.Period * motor->pulse) / 100);
    if (motor->DirOfRot)
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_ChannelFirst, pulse);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_ChannelSnd, 0);
    }
    else
    {

        __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_ChannelFirst, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_ChannelSnd, pulse);
    }

    HAL_GPIO_WritePin(motor->port, motor->pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(motor->htim, motor->PWM_ChannelFirst);
    HAL_TIM_PWM_Start(motor->htim, motor->PWM_ChannelSnd);
}

/**
  * @brief  Srop rotation motor.
  * @param  motor is motor handle
  * @retval None
  */
void StopRotation(MotorDCTypeDef *motor)
{
    HAL_GPIO_WritePin(motor->port, motor->pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(motor->htim, motor->PWM_ChannelFirst);
    HAL_TIM_PWM_Stop(motor->htim, motor->PWM_ChannelSnd);
}