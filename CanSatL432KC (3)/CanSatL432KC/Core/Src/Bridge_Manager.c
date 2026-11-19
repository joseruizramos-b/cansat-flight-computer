/* Core/Src/bridge_manager.c */
#include "bridge_manager.h"
#include "usart.h"
#include "Mission_Data.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern Mission_Data_t cansatData;
/* Core/Src/bridge_manager.c */
extern uint8_t gps_idx; // Agregá este extern

void Bridge_Update(void) {
	char buffer[256];

	int len = snprintf(buffer, sizeof(buffer),
	    "| STATE: %d |ACC: %.2f %.2f %.2f | "
	    "GYR: %.2f %.2f %.2f | "
	    "TEMP: %.2f | P: %.2f | ALT: %.2f | "
	    "GPS: %s\r\n",

		currentState, cansatData.ax, cansatData.ay, cansatData.az,
	    cansatData.gyro_x, cansatData.gyro_y, cansatData.gyro_z,
	    cansatData.temperature,
	    cansatData.pressure,
	    cansatData.altitude,
	    cansatData.gps_sentence
	);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
}
