// --- bridge_manager.c ---
#include "bridge_manager.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *bridge_uart;

void Bridge_Init(UART_HandleTypeDef *huart) {
    bridge_uart = huart;
}

void Bridge_SendTelemetry(MissionData_t *data) {
    char buffer[256];
    // snprintf con floats es pesado. Asegurate de que el Stack de la Task sea 512 o 1024
    int len = snprintf(buffer, sizeof(buffer),
        "T:%lu | ACC:%.2f,%.2f,%.2f | GYR:%.1f,%.1f,%.1f | P:%.1f | H:%.1f | GPS:%s\r\n",
        data->timestamp, data->acc_x, data->acc_y, data->acc_z,
        data->gyro_x, data->gyro_y, data->gyro_z,
        data->pressure, data->altitude, data->gps_sentence);

    HAL_UART_Transmit(bridge_uart, (uint8_t*)buffer, len, 50);
}
