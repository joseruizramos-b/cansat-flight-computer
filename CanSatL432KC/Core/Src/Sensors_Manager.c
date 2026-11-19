/* Core/Src/Sensors_Manager.c */
#include "Mission_Data.h"
#include "bridge_manager.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include <stdio.h>
#include "usart.h"
#include "i2c.h"
#include <math.h>
#include "kalman_filter.h"
#include "ekf_orientation.h"


EKF_Orientation ekf;
Kalman_t kf_ax, kf_ay, kf_az;

extern Mission_Data_t cansatData;
//extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2; // Para imprimir el diagnóstico directo
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;




int32_t t_fine;
uint8_t Sensors_Init(void) {
   uint8_t mpu_id = 0;
   uint8_t bmp_id = 0;
    char msg[100];

    // 1. Intentar leer el ID del MPU9250 (Registro 0x75)
 //    Dirección: 0x68 << 1 = 0xD0
   HAL_StatusTypeDef mpu_stat = HAL_I2C_Mem_Read(&hi2c3, 0xD0, 0x75, 1, &mpu_id, 1, 100);

    // 2. Intentar leer el ID del BMP280 (Registro 0xD0)
  //   Dirección: 0x76 << 1 = 0xEC
    HAL_StatusTypeDef bmp_stat = HAL_I2C_Mem_Read(&hi2c3, 0xEC, 0xD0, 1, &bmp_id, 1, 100);

    // Imprimir resultado en Hercules para auditar
   int len = snprintf(msg, sizeof(msg),
              "\r\n--- AUDITORIA DE HARDWARE ---\r\n"
              "MPU ID: 0x%02X (Status: %d)\r\n"
            "BMP ID: 0x%02X (Status: %d)\r\n"
            "-----\r\n",
             mpu_id, mpu_stat, bmp_id, bmp_stat);

  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
  // Configurar BMP280
  uint8_t ctrl_meas = 0x27; // temp y pressure enabled
  HAL_I2C_Mem_Write(&hi2c3, BMP280_ADDR, 0xF4, 1, &ctrl_meas, 1, 100);

  BMP280_ReadCalibration();
    // El MPU9250 debe responder 0x71 o 0x73. El BMP280 debe responder 0x58.
    if (mpu_id == 0x71 && bmp_id == 0x58) {
        // DESPERTAR MPU (Escribir 0 en Power Management 1)
        uint8_t wake = 0x00;
        HAL_I2C_Mem_Write(&hi2c3, 0xD0, 0x6B, 1, &wake, 1, 100);

        Kalman_Init(&kf_ax, 0.001f, 0.1f);
            Kalman_Init(&kf_ay, 0.001f, 0.1f);
            Kalman_Init(&kf_az, 0.001f, 0.1f);

            // Inicializar EKF (dt = 0.02 si tu tarea corre a 50Hz)
                EKF_Init(&ekf, 0.02f, 0.01f, 0.001f);


        return 1;
    }
    return 0;
}

void Sensors_Update(void) {

    uint8_t raw_data[6];

    // =========================
    // ACELERÓMETRO
    // =========================
    if (HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDR, 0x3B, 1, raw_data, 6, 100) == HAL_OK) {

        int16_t ax_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        int16_t ay_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        int16_t az_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);

        cansatData.ax = ax_raw / 16384.0f;
        cansatData.ay = ay_raw / 16384.0f;
        cansatData.az = az_raw / 16384.0f;
    }

    // =========================
    // GIROSCOPIO
    // =========================
    if (HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDR, 0x43, 1, raw_data, 6, 100) == HAL_OK) {

        int16_t gx_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        int16_t gy_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        int16_t gz_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);

        cansatData.gyro_x = gx_raw / 131.0f;
        cansatData.gyro_y = gy_raw / 131.0f;
        cansatData.gyro_z = gz_raw / 131.0f;
    }
    // 1. Filtrar Aceleración (Kalman 1D)
        cansatData.ax_f = Kalman_Update(&kf_ax, cansatData.ax);
        cansatData.ay_f = Kalman_Update(&kf_ay, cansatData.ay);
        cansatData.az_f = Kalman_Update(&kf_az, cansatData.az);

        // 2. Ejecutar EKF (Fusión de sensores)
        // Usamos el giro crudo y la acel filtrada
        EKF_Predict(&ekf, cansatData.gyro_x, cansatData.gyro_y, cansatData.gyro_z);
        EKF_Update(&ekf, cansatData.ax_f, cansatData.ay_f, cansatData.az_f);

        // 3. Extraer ángulos de Euler
        EKF_GetEulerAngles(&ekf, &cansatData.roll, &cansatData.pitch, &cansatData.yaw);



    uint8_t data[6];

    if (HAL_I2C_Mem_Read(&hi2c3, BMP280_ADDR, 0xF7, 1, data, 6, 100) == HAL_OK) {

        int32_t adc_P = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
        int32_t adc_T = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

        // Temperatura
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

        t_fine = var1 + var2;
        float T = (t_fine * 5 + 128) >> 8;
        cansatData.temperature = T / 100.0f;

        // Presión
        int64_t var1_p = ((int64_t)t_fine) - 128000;
        int64_t var2_p = var1_p * var1_p * (int64_t)dig_P6;
        var2_p = var2_p + ((var1_p * (int64_t)dig_P5) << 17);
        var2_p = var2_p + (((int64_t)dig_P4) << 35);
        var1_p = ((var1_p * var1_p * (int64_t)dig_P3) >> 8) + ((var1_p * (int64_t)dig_P2) << 12);
        var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)dig_P1) >> 33;

        if (var1_p != 0) {
            int64_t p = 1048576 - adc_P;
            p = (((p << 31) - var2_p) * 3125) / var1_p;
            var1_p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            var2_p = (((int64_t)dig_P8) * p) >> 19;

            p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dig_P7) << 4);

            cansatData.pressure = p / 25600.0f; // hPa
        }
    }


    float P0 = 1013.25; // presión al nivel del mar

    cansatData.altitude = 44330.0f * (1.0f - powf(cansatData.pressure / P0, 0.1903f));
}

void BMP280_ReadCalibration(void) {
    uint8_t calib[24];

    HAL_I2C_Mem_Read(&hi2c3, BMP280_ADDR, 0x88, 1, calib, 24, 100);

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];

    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];
}
