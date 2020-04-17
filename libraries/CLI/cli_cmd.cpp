/*
 * cli_cmd.cpp
 *
 *  Created on: 03 ���. 2019 �.
 *      Author: d.semenyuk
 */

#include <stdint.h>
#include <string.h>
#include "cli_config.h"
#include "cli_port.h"
#include "cli_base.h"
#include "cli_cmd.h"
#include "stm32f1xx_hal.h"



extern float rpm;

void CLI_CommandsParser(const TCLI_IO *const io, char *ps, CLI_InputStrLen_t len)
{

	CLI_IF_CMD("LED", "Commands for switching LED PC13") // ������� ���������� ����������� PC13
	{
		CLI_NEXT_WORD();
		CLI_IF_CMD("ON", "LED ON")
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			ok;
			return;
		}
		CLI_IF_CMD("OFF", "LED OFF")
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			ok;
			return;
		}
		CLI_IF_CMD("SWITCH", "LED SWITCH")
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			ok;
			return;
		}
		CLI_INVALID_KEYWORD();
		return;
	}
	CLI_IF_CMD("ENC", "Value of encoder")
	{
		int32_t EncCount = 0;
		extern TIM_HandleTypeDef htim2;
		while (1)
		{
			EncCount = __HAL_TIM_GET_COUNTER(&htim2);
			DbgPrintf("EnCount = %d\n\r", EncCount);
			HAL_Delay(200);
		}
		return;
	}
	CLI_IF_CMD("RPM", "Value of RPM")
	{
		DbgPrintf("\n\rRPM = %f\n\r", rpm);
		return;
	}
	

	//----------------------------------------------------------------------------------
	CLI_UNKNOWN_COMMAND();
}
