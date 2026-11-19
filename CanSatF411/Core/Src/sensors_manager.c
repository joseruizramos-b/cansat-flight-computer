#include "sensors_manager.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>

// 1. DEFINICIONES DE HARDWARE
#define MPU_ADDR (0x68 << 1)
#define BMP_ADDR (0x76 << 1)
#define P_REF_CALI 906.0f

// 2. VARIABLES DE CALIBRACIÓN (Privadas)
static uint16_t dig_T1; static int16_t dig_T2, dig_T3;
static uint16_t dig_P1; static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

// 3. PROTOTIPOS DE FUNCIONES INTERNAS (Para evitar advertencias)
float Compensate_Pressure(int32_t adc_P, int32_t adc_T);

// 4. IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS
uint8_t Sensors_Init(void) {
    uint8_t id = 0, reg = 0;
    HAL_Delay(100);

    // Reset y configuración del MPU9250
    reg = 0x80; HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &reg, 1, 50);
    HAL_Delay(100);
    reg = 0x01; HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &reg, 1, 50);
    reg = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6A, 1, &reg, 1, 50);
    reg = 0x02; HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x37, 1, &reg, 1, 50);

    // Inicialización del BMP280
    if (HAL_I2C_Mem_Read(&hi2c1, BMP_ADDR, 0xD0, 1, &id, 1, 50) == HAL_OK && id == 0x58) {
        reg = 0x57; HAL_I2C_Mem_Write(&hi2c1, BMP_ADDR, 0xF4, 1, &reg, 1, 50);
        uint8_t cal[24];
        HAL_I2C_Mem_Read(&hi2c1, BMP_ADDR, 0x88, 1, cal, 24, 100);
        dig_T1 = (cal[1]<<8)|cal[0]; dig_T2 = (int16_t)((cal[3]<<8)|cal[2]); dig_T3 = (int16_t)((cal[5]<<8)|cal[4]);
        dig_P1 = (cal[7]<<8)|cal[6]; dig_P2 = (int16_t)((cal[9]<<8)|cal[8]); dig_P3 = (int16_t)((cal[11]<<8)|cal[10]);
        dig_P4 = (int16_t)((cal[13]<<8)|cal[12]); dig_P5 = (int16_t)((cal[15]<<8)|cal[14]); dig_P6 = (int16_t)((cal[17]<<8)|cal[16]);
        dig_P7 = (int16_t)((cal[19]<<8)|cal[18]); dig_P8 = (int16_t)((cal[21]<<8)|cal[20]); dig_P9 = (int16_t)((cal[23]<<8)|cal[22]);
        return 1;
    }
    return 0;
}

void Sensors_ReadIMU(MissionData_t *data) {
    uint8_t buffer[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, 1, buffer, 14, 25) == HAL_OK) {
        data->acc_x = (float)((int16_t)(buffer[0] << 8 | buffer[1])) / 16384.0f;
        data->acc_y = (float)((int16_t)(buffer[2] << 8 | buffer[3])) / 16384.0f;
        data->acc_z = (float)((int16_t)(buffer[4] << 8 | buffer[5])) / 16384.0f;
        data->gyro_x = (float)((int16_t)(buffer[8] << 8 | buffer[9])) / 131.0f;
        data->gyro_y = (float)((int16_t)(buffer[10] << 8 | buffer[11])) / 131.0f;
        data->gyro_z = (float)((int16_t)(buffer[12] << 8 | buffer[13])) / 131.0f;
    }
}

void Sensors_ReadEnv(MissionData_t *data) {
    uint8_t b[6];
    if (HAL_I2C_Mem_Read(&hi2c1, BMP_ADDR, 0xF7, 1, b, 6, 25) == HAL_OK) {
        int32_t adc_P = (b[0] << 12) | (b[1] << 4) | (b[2] >> 4);
        int32_t adc_T = (b[3] << 12) | (b[4] << 4) | (b[5] >> 4);
        float p_real = Compensate_Pressure(adc_P, adc_T);
        data->temperature = (float)t_fine / 5120.0f;
        if (p_real > 300.0f) {
            data->pressure = p_real;
            data->altitude = 44330.0f * (1.0f - powf(p_real / P_REF_CALI, 0.1903f));
        }
    }
}

// 5. IMPLEMENTACIÓN DE FUNCIONES INTERNAS
float Compensate_Pressure(int32_t adc_P, int32_t adc_T) {
    int32_t v1t, v2t;
    v1t = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    v2t = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = v1t + v2t;

    int64_t v1p, v2p, p;
    v1p = ((int64_t)t_fine) - 128000;
    v2p = v1p * v1p * (int64_t)dig_P6;
    v2p = v2p + ((v1p * (int64_t)dig_P5) << 17);
    v2p = v2p + (((int64_t)dig_P4) << 35);
    v1p = ((v1p * v1p * (int64_t)dig_P3) >> 8) + ((v1p * (int64_t)dig_P2) << 12);
    v1p = (((((int64_t)1) << 47) + v1p)) * ((int64_t)dig_P1) >> 33;
    if (v1p == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - v2p) * 3125) / v1p;
    v1p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2p = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + v1p + v2p) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0f / 100.0f;
}
