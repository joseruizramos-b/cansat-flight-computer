#ifndef SENSORS_MANAGER_H_
#define SENSORS_MANAGER_H_

#include "stm32f4xx_hal.h"
#include "mission_data.h"

// Funciones públicas que otros archivos pueden ver
uint8_t Sensors_Init(void);
void Sensors_ReadIMU(MissionData_t *data);
void Sensors_ReadEnv(MissionData_t *data);

#endif
