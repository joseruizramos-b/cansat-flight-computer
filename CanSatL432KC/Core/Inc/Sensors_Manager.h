/* Core/Inc/Sensors_Manager.h */
#ifndef INC_SENSORS_MANAGER_H_
#define INC_SENSORS_MANAGER_H_

#include "main.h"

// Direcciones I2C (7-bit)
#define MPU9250_ADDR 0x68 << 1
#define BMP280_ADDR  0x76 << 1

// Registros básicos
#define MPU_WHO_AM_I 0x75
#define BMP_ID       0xD0
extern Mission_Data_t cansatData;
uint8_t Sensors_Init(void);
void Sensors_Update(void);
void BMP280_ReadCalibration(void);
#endif
