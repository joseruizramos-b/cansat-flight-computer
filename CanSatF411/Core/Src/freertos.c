/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mission_data.h"
#include "bridge_manager.h" // Lo usaremos ahora
#include "sensors_manager.h" // <--- VERIFICÁ ESTA LÍNEA
#include "mission_manager.h"
#include "mission_data.h"
#include <stdio.h>          // Para el sprintf de debug
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

/* USER CODE END Variables */
/* Definitions for AcquisitionTask */
osThreadId_t AcquisitionTaskHandle;
const osThreadAttr_t AcquisitionTask_attributes = {
  .name = "AcquisitionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MissionTask */
osThreadId_t MissionTaskHandle;
const osThreadAttr_t MissionTask_attributes = {
  .name = "MissionTask",
  .stack_size = 612 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BridgeTask */
osThreadId_t BridgeTaskHandle;
const osThreadAttr_t BridgeTask_attributes = {
  .name = "BridgeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAcquisitionTask(void *argument);
void StartMissionTask(void *argument);
void StartControlTask(void *argument);
void StartBridgeTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

	MissionData_Init();
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AcquisitionTask */
  AcquisitionTaskHandle = osThreadNew(StartAcquisitionTask, NULL, &AcquisitionTask_attributes);

  /* creation of MissionTask */
  MissionTaskHandle = osThreadNew(StartMissionTask, NULL, &MissionTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of BridgeTask */
  BridgeTaskHandle = osThreadNew(StartBridgeTask, NULL, &BridgeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartAcquisitionTask */
/**
  * @brief  Function implementing the AcquisitionTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAcquisitionTask */
void StartAcquisitionTask(void *argument)
{
  /* USER CODE BEGIN StartAcquisitionTask */
  MissionData_t local_data = {0};

  // Inicializamos sensores AQUÍ. Si falla, el TICK ya está corriendo y no se cuelga.
  Sensors_Init();
  Mission_Init(); // Inicializamos una vez
  for(;;)
  {

    local_data.timestamp = osKernelGetTickCount();
    MissionData_Update(&local_data);
    Mission_RunIteration();
    osDelay(200);
  }
  /* USER CODE END StartAcquisitionTask */
}

/* USER CODE BEGIN Header_StartMissionTask */
/**
* @brief Function implementing the MissionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMissionTask */
void StartMissionTask(void *argument)
{
  /* USER CODE BEGIN StartMissionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMissionTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartBridgeTask */
/**
* @brief Function implementing the BridgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBridgeTask */
void StartBridgeTask(void *argument)
{
  /* USER CODE BEGIN StartBridgeTask */
  extern UART_HandleTypeDef huart2; // La del USB/PC
  Bridge_Init(&huart2);
  MissionData_t data_to_send;
  for(;;) {
    MissionData_Read(&data_to_send);
    Bridge_SendTelemetry(&data_to_send);
    osDelay(500); // Enviamos cada 500ms
  }
  /* USER CODE END StartBridgeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

