/* USER CODE BEGIN Header */

/**

******************************************************************************

* @file : main.c

* @brief : Main program body

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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // Para manejar cadenas de texto
#include <stdlib.h> // Para funciones como atof
#include <math.h>   // VITAL para senos, cosenos y raíces
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    ST_STANDBY = 0,
    ST_FLIGHT,
    ST_DESCENT,
    ST_DEPLOYMENT,
    ST_LANDED
} MissionState_t;

MissionState_t current_state = ST_STANDBY;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for NavTask */
osThreadId_t NavTaskHandle;
const osThreadAttr_t NavTask_attributes = {
  .name = "NavTask",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MissionTask */
osThreadId_t MissionTaskHandle;
const osThreadAttr_t MissionTask_attributes = {
  .name = "MissionTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// 1. Identificadores de Hardware
volatile uint8_t sensor_id = 0, mag_id = 0, bmp_id = 0;

// 2. Datos Crudos (Raw) del MPU
float ax, ay, az;
float gx, gy, gz;

// 3. Filtro de Kernel (Promedio Móvil de 5 muestras)
// Estos arreglos guardan las últimas lecturas para "suavizarlas"
float ax_kernel[5] = {0}, ay_kernel[5] = {0}, az_kernel[5] = {0};
uint8_t k_idx = 0; // Índice del buffer circular
float ax_f, ay_f, az_f; // Valores ya filtrados

// 4. Orientación (Euler) y Fusión
float roll = 0.0f, pitch = 0.0f;
const float alpha = 0.98f; // Peso del filtro complementario
const float dt = 0.02f;    // Tiempo de muestreo (50Hz)

// 5. Variables GPS
uint8_t gps_data;
char gps_buffer[100];
uint8_t gps_idx = 0;
volatile uint8_t gps_ready = 0; // <--- AGREGAR ESTA LÍNEA
char msg_telemetria[160];
/* --- BMP280: Calibración y Datos --- */
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
//float sea_level_pa = 101325.0f; // Presión estándar al nivel del mar (ajustable)


float pressure_ref = 1013.25f; // Se actualizará al calibrar
volatile uint8_t calibrated = 0; // Flag de seguridad


float altitude = 0.0f;
volatile float temperature = 0.0f; // El volatile le prohíbe al compilador borrarla
float pressure_hpa = 0.0f; // <--- AGREGAR ESTA LÍNEA
volatile float mx, my, mz; // MicroTeslas

float last_altitude = 0.0f;
float vertical_velocity = 0.0f;
uint32_t last_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartMotorTask(void *argument);
void StartSensorTask(void *argument);
void StartNavTask(void *argument);
void StartMissionTask(void *argument);

/* USER CODE BEGIN PFP */
float Compensate_Pressure(int32_t adc_P, int32_t adc_T);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  /* USER CODE BEGIN 2 */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(100);
  /* USER CODE BEGIN 2 */
  // --- TEST DE SUPERVIVENCIA PB6 Y PB7 ---
  HAL_UART_Transmit(&huart1, (uint8_t*)"[!] INICIANDO TEST DE FUERZA BRUTA EN PB6/PB7...\r\n", 51, 100);

  GPIO_InitTypeDef GPIO_InitStructTest = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configuramos como OUTPUT PUSH-PULL (Fuerza máxima)
  GPIO_InitStructTest.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStructTest.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructTest.Pull = GPIO_NOPULL;
  GPIO_InitStructTest.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructTest);

  // FORZAMOS EL "1" LÓGICO (Debería dar 3.3V)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

  // Aquí te doy tiempo para medir con el multímetro
  HAL_Delay(10000);

  // LUEGO FORZAMOS EL "0" LÓGICO (Debería dar 0V)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  /* USER CODE END 2 */
  // 1. Reset y Despertar MPU
  uint8_t res_v = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, (0x68<<1), 0x6B, 1, &res_v, 1, 100);
  HAL_Delay(50);
  uint8_t wk_up = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, (0x68<<1), 0x6B, 1, &wk_up, 1, 100);

  // 2. Configurar el puente (Bypass) y desactivar Master Mode
  uint8_t master_dis = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, (0x68<<1), 0x6A, 1, &master_dis, 1, 100);
  uint8_t byp = 0x02;
  HAL_I2C_Mem_Write(&hi2c1, (0x68<<1), 0x37, 1, &byp, 1, 100);
  HAL_Delay(10);

  // 3. LECTURA DE IDENTIDAD (Handshake) - Esto debe ir ANTES de los IF
  HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x75, 1, (uint8_t*)&sensor_id, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x00, 1, (uint8_t*)&mag_id, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, (0x76<<1), 0xD0, 1, (uint8_t*)&bmp_id, 1, 100);

  // 4. Inicializar BMP280 si fue detectado
  if(bmp_id == 0x58) {
      uint8_t bmp_conf = 0x57;
      HAL_I2C_Mem_Write(&hi2c1, (0x76<<1), 0xF4, 1, &bmp_conf, 1, 100);

      uint8_t calib[24];
      HAL_I2C_Mem_Read(&hi2c1, (0x76<<1), 0x88, 1, calib, 24, 100);
      dig_T1 = (calib[1] << 8) | calib[0];
      dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
      dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
      dig_P1 = (calib[7] << 8) | calib[6];
      dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
      dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
      dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
      dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
      dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
      dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
      dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
      dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);
  }

  if(bmp_id == 0x58) {
      float p_sum = 0;
      for(int i = 0; i < 20; i++) {
          uint8_t b_data[6];
          HAL_I2C_Mem_Read(&hi2c1, (0x76<<1), 0xF7, 1, b_data, 6, 100);
          int32_t adc_P = (b_data[0] << 12) | (b_data[1] << 4) | (b_data[2] >> 4);
          int32_t adc_T = (b_data[3] << 12) | (b_data[4] << 4) | (b_data[5] >> 4);

          // AHORA SÍ calculamos la presión real en cada iteración
          p_sum += Compensate_Pressure(adc_P, adc_T);
          HAL_Delay(50);
      }
      pressure_ref = p_sum / 20.0f;
      calibrated = 1;
  }
  // 5. Inicializar Magnetómetro si fue detectado
  if(mag_id == 0x48) {
      uint8_t m_mode = 0x16;
      HAL_I2C_Mem_Write(&hi2c1, (0x0C<<1), 0x0A, 1, &m_mode, 1, 100);
  }

HAL_UART_Receive_IT(&huart1, &gps_data, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of NavTask */
  NavTaskHandle = osThreadNew(StartNavTask, NULL, &NavTask_attributes);

  /* creation of MissionTask */
  MissionTaskHandle = osThreadNew(StartMissionTask, NULL, &MissionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */



  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */



  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */



  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */



  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */



  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOT_DIR2_Pin|MOT_DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_DIR2_Pin MOT_DIR1_Pin */
  GPIO_InitStruct.Pin = MOT_DIR2_Pin|MOT_DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */



  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Almacenamos el byte recibido en el buffer
        if (gps_data != '\n' && gps_idx < 99) {
            gps_buffer[gps_idx++] = gps_data;
        } else {
            gps_buffer[gps_idx] = '\0'; // Cerramos la cadena
            gps_ready = 1;              // Avisamos que hay una trama lista
            gps_idx = 0;                // Reiniciamos índice
        }
        // ¡VITAL! Volver a activar la interrupción para el siguiente byte
        HAL_UART_Receive_IT(&huart1, &gps_data, 1);
    }
}

float Compensate_Pressure(int32_t adc_P, int32_t adc_T) {
    double v1, v2, t_fine, p_var1, p_var2, p;

    // Compensación de Temperatura
    v1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
    v2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
         (((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)) * ((double)dig_T3);
    t_fine = v1 + v2;
    temperature = (float)(t_fine / 5120.0);
    // Compensación de Presión
    p_var1 = (t_fine/2.0) - 64000.0;
    p_var2 = p_var1 * p_var1 * ((double)dig_P6) / 32768.0;
    p_var2 = p_var2 + p_var1 * ((double)dig_P5) * 2.0;
    p_var2 = (p_var2/4.0) + (((double)dig_P4) * 65536.0);
    p_var1 = (((double)dig_P3) * p_var1 * p_var1 / 524288.0 + ((double)dig_P2) * p_var1) / 524288.0;
    p_var1 = (1.0 + p_var1 / 32768.0) * ((double)dig_P1);

    if (p_var1 == 0.0) return 0;
    p = 1048576.0 - (double)adc_P;
    p = (p - (p_var2 / 4096.0)) * 6250.0 / p_var1;
    p_var1 = ((double)dig_P9) * p * p / 2147483648.0;
    p_var2 = p * ((double)dig_P8) / 32768.0;
    return (float)((p + (p_var1 + p_var2 + ((double)dig_P7)) / 16.0) / 100.0);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 1. Limpiar flags de error (Overrun, Noise, Frame Error)
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

        // 2. Reiniciar la escucha por interrupción
        HAL_UART_Receive_IT(huart, &gps_data, 1);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  // 1. Arrancamos los periféricos de hardware
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Servo 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Motor N20

  // 2. Aseguramos que empiecen en REPOSO (0V)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n[!] MOTOR_TASK: READY\r\n", 26, 100);

  for(;;) {
    // Esperamos la señal de la MissionTask (Infrarrojo o Altura)
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    if(flags == 0x01) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"[!] DESPLIEGUE INICIADO\r\n", 25, 100);

        // A. Movemos Servos (MG90S a ~45 grados -> 1250us)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1250);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1250);

        osDelay(200); // Pausa para evitar pico de corriente masivo

        // B. Arrancamos Motor N20 (80% potencia para vencer inercia)
        HAL_GPIO_WritePin(GPIOB, MOT_DIR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, MOT_DIR2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800);
    }
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint8_t mpu_buf[14]; // Buffer para lectura rápida (Burst Read)

  for(;;)
  {
    // Solo leemos si el sensor respondió al inicio (ID 112 o 113)
    if(sensor_id != 0)
    {
    	// --- LECTURA MAGNETÓMETRO (AK8963) ---
    	if(mag_id == 0x48) // Solo si lo encontró al inicio
    	{
    	    uint8_t m_buf[7]; // 6 bytes de datos + 1 de Status (ST2)
    	    // Leemos desde el registro 0x03 (X-Low) hasta el 0x09 (Status 2)
    	    if(HAL_I2C_Mem_Read(&hi2c1, (0x0C<<1), 0x03, 1, m_buf, 7, 20) == HAL_OK)
    	    {
    	        // Los datos son Little Endian (Byte bajo primero)
    	        // El factor 0.15f convierte el valor crudo a MicroTeslas (uT)
    	        mx = (int16_t)(m_buf[1] << 8 | m_buf[0]) * 0.15f;
    	        my = (int16_t)(m_buf[3] << 8 | m_buf[2]) * 0.15f;
    	        mz = (int16_t)(m_buf[5] << 8 | m_buf[4]) * 0.15f;
    	    }
    	}
      if(HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x3B, 1, mpu_buf, 14, 20) == HAL_OK)
      {
        // 1. Convertir datos crudos a unidades físicas (Aceleración y Giro)
        float raw_ax = (int16_t)(mpu_buf[0] << 8 | mpu_buf[1]) / 16384.0f;
        float raw_ay = (int16_t)(mpu_buf[2] << 8 | mpu_buf[3]) / 16384.0f;
        float raw_az = (int16_t)(mpu_buf[4] << 8 | mpu_buf[5]) / 16384.0f;

        gx = ((int16_t)(mpu_buf[8] << 8 | mpu_buf[9]) / 131.0f);
        gy = ((int16_t)(mpu_buf[10] << 8 | mpu_buf[11]) / 131.0f);
        gz = ((int16_t)(mpu_buf[12] << 8 | mpu_buf[13]) / 131.0f);

        // ... dentro del bloque HAL_OK ...
        // 2. APLICAR FILTRO DE KERNEL (Promedio Móvil)
        ax_kernel[k_idx] = raw_ax;
        ay_kernel[k_idx] = raw_ay;
        az_kernel[k_idx] = raw_az;
        k_idx = (k_idx + 1) % 5;

        // Promedios corregidos
        ax_f = (ax_kernel[0] + ax_kernel[1] + ax_kernel[2] + ax_kernel[3] + ax_kernel[4]) / 5.0f;
        ay_f = (ay_kernel[0] + ay_kernel[1] + ay_kernel[2] + ay_kernel[3] + ay_kernel[4]) / 5.0f;
        az_f = (az_kernel[0] + az_kernel[1] + az_kernel[2] + az_kernel[3] + az_kernel[4]) / 5.0f; // <--- Cambiar 'ax' por 'az'
      }

      float roll_acc = atan2f(ay_f, az_f) * 57.2957f;
      float pitch_acc = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 57.2957f;

      roll = alpha * (roll + gx * dt) + (1.0f - alpha) * roll_acc;
      pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
    }

    /* --- LECTURA BARÓMETRO --- */
    if(bmp_id == 0x58) {
        uint8_t b_data[6];
        if(HAL_I2C_Mem_Read(&hi2c1, (0x76<<1), 0xF7, 1, b_data, 6, 20) == HAL_OK) {
            int32_t adc_P = (b_data[0] << 12) | (b_data[1] << 4) | (b_data[2] >> 4);
            int32_t adc_T = (b_data[3] << 12) | (b_data[4] << 4) | (b_data[5] >> 4);

            // Usamos la función de compensación
            pressure_hpa = Compensate_Pressure(adc_P, adc_T);

            // Altitud relativa al suelo de Cali/Pasto
            altitude = 44330.0f * (1.0f - powf(pressure_hpa / pressure_ref, 0.1903f));

            // Cálculo de Velocidad Vertical
            uint32_t current_tick = osKernelGetTickCount();
            float dt_v = (current_tick - last_tick) / 1000.0f;

            if(dt_v > 0) {
                vertical_velocity = (altitude - last_altitude) / dt_v;
                last_altitude = altitude;
                last_tick = current_tick;
            }
        }
    } // <--- Solo una llave para cerrar el if(bmp_id)


    osDelay(20); // Muestreo a 50Hz (Consistente con el dt de 0.02s)
}
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartNavTask */

/**

* @brief Function implementing the NavTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartNavTask */
void StartNavTask(void *argument)
{
  /* USER CODE BEGIN StartNavTask */
  for(;;) {
      // 1. Preparamos los datos con casting a (long) para evitar los warnings
      int32_t i_alt_int = (int32_t)altitude;
      int32_t i_alt_dec = (int32_t)abs((int32_t)((altitude - i_alt_int) * 10));

      int32_t temp_int = (int32_t)temperature;
      int32_t temp_dec = (int32_t)abs((int32_t)((temperature - temp_int) * 10));

      // 2. sprintf usando %ld y casting explícito a (long)
      // Esto elimina el warning de "type mismatch"
      int len = sprintf(msg_telemetria, "ATISQUE,ST:%d,A:%ld.%ld,T:%ld.%ld,GPS:%s\r\n",
                       (int)current_state,
                       (long)i_alt_int, (long)i_alt_dec,
                       (long)temp_int, (long)temp_dec,
                       gps_buffer);

      // 3. Transmisión
      HAL_UART_Transmit(&huart1, (uint8_t*)msg_telemetria, len, 100);

      osDelay(200); // Telemetría a 5Hz.
  }
  /* USER CODE END StartNavTask */
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
  for(;;) {
      /* * --- MODO DE PRUEBA 2: DISPARO POR INFRARROJO (PB1) ---
       * Nota: La mayoría de sensores IR envían un '0' (RESET) al detectar obstáculo.
       */
      if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET && current_state == ST_STANDBY)
      {
          current_state = ST_DEPLOYMENT;

          // Activamos la bandera para la tarea de motores
          osThreadFlagsSet(MotorTaskHandle, 0x01);
      }

      /* --- CÓDIGO DE TEMPERATURA (COMENTADO) ---
      if(temperature >= 37.0f && current_state == ST_STANDBY) {
          current_state = ST_DEPLOYMENT;
          osThreadFlagsSet(MotorTaskHandle, 0x01);
      }
      ------------------------------------------ */

      /* --- CÓDIGO ORIGINAL DE VUELO (COMENTADO PARA LA COMPETENCIA UNAM) ---
      switch(current_state) {
          case ST_STANDBY:
              if(altitude > 20.0f) current_state = ST_FLIGHT;
              break;
          case ST_FLIGHT:
              if(vertical_velocity < -1.5f) current_state = ST_DESCENT;
              break;
          case ST_DESCENT:
              if(altitude <= 350.0f) {
                  current_state = ST_DEPLOYMENT;
                  osThreadFlagsSet(MotorTaskHandle, 0x01);
              }
              break;
          case ST_DEPLOYMENT:
              if(altitude < 5.0f) current_state = ST_LANDED;
              break;
      }
      ---------------------------------------------------------------------- */

      osDelay(50); // Reducimos a 50ms para una respuesta más rápida al pasar la mano
  }
  /* USER CODE END StartMissionTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */



  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

/* User can add his own implementation to report the HAL error return state */

__disable_irq();

while (1)

{

}

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

/* User can add his own implementation to report the file name and line number,

ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
