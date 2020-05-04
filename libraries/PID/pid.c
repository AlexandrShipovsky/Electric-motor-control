/* Includes ------------------------------------------------------------------*/
#include "pid.h"

void pidUpdate(pidTypeDef *pid)
{
    pid->dt = __HAL_TIM_GET_COUNTER(pid->htim);

    if (pid->dt == 0)
    {
        return;
    }

    pid->epsilon = (pid->SetPoint - pid->ProcessVal);
    pid->integral += pid->dt * pid->epsilon;
    float diff;
    diff = ((pid->epsilon - pid->epsilonPrev)*100) / (float)(pid->dt);

    pid->DirOfRot = 0;
    pid->ManipulVal = pid->Kp * (pid->epsilon + pid->Ki * pid->integral + pid->Kd * diff);
    if (pid->ManipulVal < 0)
    {
        pid->DirOfRot = 1;
        pid->ManipulVal *= (-1);
    }

    if (pid->ManipulVal > 100.0)
    {
        pid->ManipulVal = 100.0;
    }

    pid->epsilonPrev = pid->epsilon;

    __HAL_TIM_SET_COUNTER(pid->htim, (uint16_t)0);
    return;
}