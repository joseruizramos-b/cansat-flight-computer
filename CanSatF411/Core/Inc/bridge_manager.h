#ifndef BRIDGE_MANAGER_H_
#define BRIDGE_MANAGER_H_

#include "stm32f4xx_hal.h"
#include "mission_data.h"

void Bridge_Init(UART_HandleTypeDef *huart);
void Bridge_SendTelemetry(MissionData_t *data);

#endif
