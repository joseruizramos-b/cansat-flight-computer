/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
uint8_t check = 0;
uint8_t data = 0;
/* USER CODE BEGIN PV */
// --- MPU9250 (Acelerómetro y Giroscopio) ---
float ax, ay, az; // Aceleración en g
float gx, gy, gz; // Velocidad angular en dps

// --- AK8963 (Magnetómetro) ---
float mx, my, mz; // Campo magnético en uT

// --- BMP280 (Barómetro y Temperatura) ---
float temp_c;     // Temperatura en °C
float press_hpa;  // Presión en hPa

// Buffers para lectura de registros
uint8_t mpu_buf[14], mag_buf[7], baro_buf[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Definiciones de registros
  #define MPU9250_ADDR (0x68 << 1)
  #define PWR_MGMT_1   0x6B
  #define WHO_AM_I     0x75

  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  uint8_t reg_val;

  // 1. Reset y Despertar MPU9250
  reg_val = 0x80; // Reset total
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &reg_val, 1, 100);
  HAL_Delay(100);
  reg_val = 0x00; // Despertar
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &reg_val, 1, 100);

  // 2. ACTIVAR BYPASS (Vital para mx, my, mz)
  reg_val = 0x02;
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x37, 1, &reg_val, 1, 100);
  HAL_Delay(50);

  // 3. Inicializar Magnetómetro AK8963
  reg_val = 0x16; // Modo Continuo 2 (100Hz) y 16-bit
  HAL_I2C_Mem_Write(&hi2c1, (0x0C << 1), 0x0A, 1, &reg_val, 1, 100);

  // 4. Inicializar BMP280 (Modo Normal y Oversampling)
  reg_val = 0x27;
  HAL_I2C_Mem_Write(&hi2c1, (0x76 << 1), 0xF4, 1, &reg_val, 1, 100);
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      // A. Leer Acelerómetro y Giroscopio (14 bytes desde 0x3B)
      HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, mpu_buf, 14, 100);
      ax = (int16_t)(mpu_buf[0]<<8 | mpu_buf[1]) / 16384.0f;
      ay = (int16_t)(mpu_buf[2]<<8 | mpu_buf[3]) / 16384.0f;
      az = (int16_t)(mpu_buf[4]<<8 | mpu_buf[5]) / 16384.0f;
      gx = (int16_t)(mpu_buf[8]<<8 | mpu_buf[9]) / 131.0f;
      gy = (int16_t)(mpu_buf[10]<<8 | mpu_buf[11]) / 131.0f;
      gz = (int16_t)(mpu_buf[12]<<8 | mpu_buf[13]) / 131.0f;

      // B. Leer Magnetómetro (7 bytes desde 0x03)
      // Se leen 7 bytes para incluir el registro ST2 que libera los datos
      // 1. Leer 7 bytes (el séptimo es el registro ST2, obligatorio para desbloquear)
      HAL_I2C_Mem_Read(&hi2c1, (0x0C << 1), 0x03, 1, mag_buf, 7, 100);

      // 2. Verificar si hay "Magnetic Sensor Overflow" (opcional pero académico)
      if (!(mag_buf[6] & 0x08)) { // Si el bit 3 de ST2 no es 1, el dato es válido
          mx = (int16_t)(mag_buf[1] << 8 | mag_buf[0]) * 0.15f;
          my = (int16_t)(mag_buf[3] << 8 | mag_buf[2]) * 0.15f;
          mz = (int16_t)(mag_buf[5] << 8 | mag_buf[4]) * 0.15f;
      }
      // C. Leer Barómetro (6 bytes desde 0xF7)
      HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0xF7, 1, baro_buf, 6, 100);



      // Dentro del while(1) después de leer baro_buf
      temp_c = (float)((baro_buf[3] << 12) | (baro_buf[4] << 4) | (baro_buf[5] >> 4)) / 16384.0f;
      press_hpa = (float)((baro_buf[0] << 12) | (baro_buf[1] << 4) | (baro_buf[2] >> 4)) / 100.0f;
      // Nota: Los datos de baro_buf requieren fórmulas de compensación
      // Por ahora, verifica que los bytes cambien en Live Expressions.

      HAL_Delay(100); // Frecuencia de muestreo de 10Hz
      /* USER CODE END WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
