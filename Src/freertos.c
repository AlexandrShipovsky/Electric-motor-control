/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "arm_math.h"
#include "usb_device.h"
#include "cli.h"
#include "MotorDC.h"
#include "encoder.h"
#include "vbat.h"
#include "pid.h"

#include "prothawk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern unsigned long ulHighFrequencyTimerTicks;

xQueueHandle CANReceiveQueueHandle = NULL; // Очередь для приема байт из CAN

uint8_t state = TrackingState; // Режим работы
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  ulHighFrequencyTimerTicks = 0UL;
}

__weak unsigned long getRunTimeCounterValue(void)
{
  return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  CANReceiveQueueHandle = xQueueCreate(8, 8);

  /* Infinite loop */
  for (;;)
  {

    vTaskDelay(500);
  }
  /* USER CODE END 5 */
}
/**
* @brief Function implementing the cliTask thread.
* @param argument: Not used
* @retval None
*/
void cliStartTask(void const *argument)
{
  /* USER CODE BEGIN cliStartTask */
  extern SemaphoreHandle_t CliMutex;
  CliMutex = xSemaphoreCreateMutex();
  /* Infinite loop */
  for (;;)
  {
    DBG_CLI_USB_Task();
    vTaskDelay(10);
  }
}
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pidStartTask */
void pidStartTask(void const *argument)
{
  /* USER CODE BEGIN pidStartTask */

  extern MotorDCTypeDef MotorRoll;
  extern MotorDCTypeDef MotorPitch;
  extern EncTypeDef EncRoll;
  extern EncTypeDef EncPitch;
  extern pidTypeDef pidPitch;
  extern pidTypeDef pidRoll;

  /* Infinite loop */
  for (;;)
  {
    if ((pidRoll.MaxSetPoint == 0) | (pidRoll.MinSetPoint == 0) | (pidPitch.MaxSetPoint == 0) | (pidPitch.MinSetPoint == 0))
    {
      state = CLIState;
    }
    switch (state)
    {
    case TrackingState:
      taskENTER_CRITICAL();
      pidPitch.ProcessVal = GetEnc(&EncPitch);
      pidUpdate(&pidPitch);
      MotorPitch.DirOfRot = pidPitch.DirOfRot;
      MotorPitch.pulse = pidPitch.ManipulVal;
      rotation(&MotorPitch);
      taskEXIT_CRITICAL();

      taskENTER_CRITICAL();
      pidRoll.ProcessVal = GetEnc(&EncRoll);
      pidUpdate(&pidRoll);
      MotorRoll.DirOfRot = pidRoll.DirOfRot;
      MotorRoll.pulse = pidRoll.ManipulVal;
      rotation(&MotorRoll);

      taskEXIT_CRITICAL();
      break;
    case TestState:
      pidPitch.ProcessVal = GetEnc(&EncPitch);
      pidRoll.ProcessVal = GetEnc(&EncRoll);
      StopRotation(&MotorPitch);
      StopRotation(&MotorRoll);
      break;
    case CLIState:
      break;
    default:
      break;
    }
    vTaskDelay(10); //???????
  }
}
/* USER CODE BEGIN Header_StartParserCANTask */
/**
* @brief Function implementing the ParserCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParserCANTask */
void StartParserCANTask(void const *argument)
{
  /* USER CODE BEGIN StartParserCANTask */
  uint8_t buf[8];
  extern pidTypeDef pidRoll;
  extern pidTypeDef pidPitch;
  extern EncTypeDef EncRoll;
  extern EncTypeDef EncPitch;

  extern CAN_HandleTypeDef hcan;
  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t buftx[8];

  // Передача на блок управления приводами
  TxHeader.DLC = 8;
  TxHeader.StdId = 0x0000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* Infinite loop */
  for (;;)
  {
    if (CANReceiveQueueHandle != NULL)
    {
      if (xQueueReceive(CANReceiveQueueHandle,
                        buf,
                        (TickType_t)0) == pdPASS)
      {
        /* *pxRxedPointer now points to xMessage. */
      }
      switch (buf[0])
      {
      case PitchRollCommand:
        memcpy(&pidPitch.SetPoint, &buf[1], sizeof(pidPitch.SetPoint));
        memcpy(&pidRoll.SetPoint, &buf[3], sizeof(pidRoll.SetPoint));
        break;
      case TestMode:
        state = TestState;
        taskENTER_CRITICAL();
        buftx[0] = TestMode;
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
        {
        }
        taskEXIT_CRITICAL();
        break;
      case CalibComplied:
        SetZero(&EncPitch);
        SetZero(&EncRoll);
        state = TrackingState;
        buftx[0] = CalibComplied;
        taskENTER_CRITICAL();
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
        {
        }
        taskEXIT_CRITICAL();
        break;
      case PitchMinMax:
        memcpy(&pidPitch.MinSetPoint, &buf[1], sizeof(pidPitch.MinSetPoint));
        memcpy(&pidPitch.MaxSetPoint, &buf[3], sizeof(pidPitch.MaxSetPoint));
        break;
      case RollMinMax:
        memcpy(&pidRoll.MinSetPoint, &buf[1], sizeof(pidRoll.MinSetPoint));
        memcpy(&pidRoll.MaxSetPoint, &buf[3], sizeof(pidRoll.MaxSetPoint));
        break;
      default:
        memset(buf, 0x00, sizeof(buf)); // Очистить буфер
        break;
      }
    }
    memset(buf, 0x00, sizeof(buf)); // Очистить буфер
    vTaskDelay(1);
  }
  /* USER CODE END StartParserCANTask */
}
/*
*
*
*
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t buf[8];
  CAN_RxHeaderTypeDef RxHeader;

  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
  {
    //Ошибка
  }
  else
  {
    if (CANReceiveQueueHandle != NULL)
    {
      xQueueSendToBackFromISR(CANReceiveQueueHandle,
                              (void *)buf,
                              &xHigherPriorityTaskWoken);
    }
  }
}
/* USER CODE BEGIN Header_StartCANTxTask */
/**
* @brief Function implementing the CANTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTxTask */
void StartCANTxTask(void const *argument)
{
  /* USER CODE BEGIN StartCANTxTask */
  extern pidTypeDef pidPitch;
  extern pidTypeDef pidRoll;
  extern vbatTypeDef vbat;
  extern MotorDCTypeDef MotorPitch;

  extern CAN_HandleTypeDef hcan;
  uint32_t TxMailBox; //= CAN_TX_MAILBOX0;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t buftx[8];
  float voltage = 0.0;
  /* Infinite loop */
  for (;;)
  {
    // Считывание значения напряжения аккумулятора
    voltage = GetVoltageBat(&vbat);
    // Передача на блок управления приводами
    TxHeader.DLC = 5;
    TxHeader.StdId = 0x0000;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.TransmitGlobalTime = DISABLE;

    buftx[0] = PitchRollCommand;
    buftx[1] = (uint8_t)(pidPitch.ProcessVal & 0xFF);
    buftx[2] = (uint8_t)(pidPitch.ProcessVal >> 8);
    buftx[3] = (uint8_t)(pidRoll.ProcessVal & 0xFF);
    buftx[4] = (uint8_t)(pidRoll.ProcessVal >> 8);

    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();

    vTaskDelay(10);

    buftx[0] = PitchMinMax;
    buftx[1] = (uint8_t)(pidPitch.MinSetPoint & 0xFF);
    buftx[2] = (uint8_t)(pidPitch.MinSetPoint >> 8);
    buftx[3] = (uint8_t)(pidPitch.MaxSetPoint & 0xFF);
    buftx[4] = (uint8_t)(pidPitch.MaxSetPoint >> 8);

    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();

    vTaskDelay(10);

    buftx[0] = RollMinMax;
    buftx[1] = (uint8_t)(pidRoll.MinSetPoint & 0xFF);
    buftx[2] = (uint8_t)(pidRoll.MinSetPoint >> 8);
    buftx[3] = (uint8_t)(pidRoll.MaxSetPoint & 0xFF);
    buftx[4] = (uint8_t)(pidRoll.MaxSetPoint >> 8);

    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();
    vTaskDelay(10);

    buftx[0] = VBATCommand;
    memcpy(&buftx[1], &voltage, sizeof(voltage));

    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();
    vTaskDelay(10);

    buftx[0] = PitchForceCommand;
    memcpy(&buftx[1], &MotorPitch.torque, sizeof(MotorPitch.torque));

    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buftx, &TxMailBox) != HAL_OK)
    {
      //Error_Handler();
    }
    taskEXIT_CRITICAL();
    vTaskDelay(10);
  }
  /* USER CODE END StartCANTxTask */
}
/* USER CODE BEGIN Header_StartSDADC_Handler */
/**
* @brief Обработчик массива данных от сигма-дельта АЦП
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDADC_Handler */
void StartSDADC_Handler(void const *argument)
{

  /* USER CODE BEGIN StartSDADC_Handler */
  extern SDADC_HandleTypeDef hsdadc1;
  extern DMA_HandleTypeDef hdma_sdadc1;
  extern MotorDCTypeDef MotorPitch;
#define SIZE_DATA 128

  volatile int16_t data[SIZE_DATA] = {
      0x00,
  };
  float sum_sqrt;
  HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1);
  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, 500) != HAL_OK)
  {
    // Error
  }

  if (HAL_SDADC_InjectedStart_DMA(&hsdadc1, (uint32_t *)data, SIZE_DATA) != HAL_OK)
  {
    // Error
  }
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(5);
    HAL_SDADC_InjectedStop_DMA(&hsdadc1);

    uint64_t sum = 0;
    uint8_t i;
    if (MotorPitch.DirOfRot == DIRECT_ROTATION)
    {
      for (i = 0; i <= SIZE_DATA; i += 2)
      {
        sum += data[i] * data[i];
      }
      sum_sqrt = sum * 2 / (float)SIZE_DATA;
      arm_sqrt_f32(sum_sqrt, &MotorPitch.torque);
      if (MotorPitch.torque > 200)
      {
        MotorPitch.torque -= 2675;
        MotorPitch.torque *= 0.003;
      }
      else
      {
        MotorPitch.torque = 0.0;
      }
    }
    else
    {
      for (i = 1; i < SIZE_DATA; i += 2)
      {
        sum += data[i] * data[i];
      }
      sum_sqrt = sum * 2 / (float)SIZE_DATA;
      arm_sqrt_f32(sum_sqrt, &MotorPitch.torque);
      if (MotorPitch.torque > 200)
      {
        MotorPitch.torque -= 2036;
        MotorPitch.torque *= 0.004659;
        MotorPitch.torque *= -1.0;
      }
      else
      {
        MotorPitch.torque = 0.0;
      }
    }

    if (HAL_SDADC_InjectedStart_DMA(&hsdadc1, (uint32_t *)data, SIZE_DATA) != HAL_OK)
    {
      // Error
    }
  }
  /* USER CODE END StartSDADC_Handler */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
