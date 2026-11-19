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
#include "Mission_Data.h"
#include "bridge_manager.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include "ekf_orientation.h"

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
extern Mission_Data_t cansatData; // Definición de la instancia global
/* USER CODE END Variables */
/* Definitions for MissionTask */
osThreadId_t MissionTaskHandle;
const osThreadAttr_t MissionTask_attributes = {
  .name = "MissionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorsTask */
osThreadId_t SensorsTaskHandle;
const osThreadAttr_t SensorsTask_attributes = {
  .name = "SensorsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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

void StartMissionTask(void *argument);
void StartTaskSensorsTask(void *argument);
void StartTaskBridgeTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	Sensors_Init(); // Inicializa MPU9250 y BMP280 // Configura el estado inicial y el PWM a los 200m
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
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
  /* creation of MissionTask */
  MissionTaskHandle = osThreadNew(StartMissionTask, NULL, &MissionTask_attributes);

  /* creation of SensorsTask */
  SensorsTaskHandle = osThreadNew(StartTaskSensorsTask, NULL, &SensorsTask_attributes);

  /* creation of BridgeTask */
  BridgeTaskHandle = osThreadNew(StartTaskBridgeTask, NULL, &BridgeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMissionTask */
/**
  * @brief  Function implementing the MissionTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMissionTask */
void StartMissionTask(void *argument)
{
  /* USER CODE BEGIN StartMissionTask */
	Mission_Init();
  /* Infinite loop */
	for(;;) {
		Mission_Update();
	    osDelay(100);
	}
  /* USER CODE END StartMissionTask */
}

/* USER CODE BEGIN Header_StartTaskSensorsTask */
/**
* @brief Function implementing the SensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSensorsTask */
void StartTaskSensorsTask(void *argument)
{
  /* USER CODE BEGIN StartTaskSensorsTask */
  /* Infinite loop */
	for(;;) {
	    Sensors_Update();
	    osDelay(20); // 10Hz
	}
  /* USER CODE END StartTaskSensorsTask */
}

/* USER CODE BEGIN Header_StartTaskBridgeTask */
/**
* @brief Function implementing the BridgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBridgeTask */
void StartTaskBridgeTask(void *argument)
{
  /* USER CODE BEGIN StartTaskBridgeTask */
  /* Infinite loop */
	for(;;) {
	    Bridge_Update();
	    osDelay(500); // Enviamos mensaje cada medio segundo
	  }
  /* USER CODE END StartTaskBridgeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

