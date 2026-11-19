
#include "Mission_Data.h"
#include "bridge_manager.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include "usart.h"
#include <string.h>

extern Mission_Data_t cansatData;
extern UART_HandleTypeDef huart1;

// Variables globales con tus nombres exactos
uint8_t gps_byte;
char gps_buffer[100];
uint8_t gps_idx = 0;

void GPS_Init(void) {
    gps_idx = 0;
    memset(gps_buffer, 0, sizeof(gps_buffer));
    HAL_UART_Receive_IT(&huart1, &gps_byte, 1);
}

/**
 * @brief Rescata la UART si hay ruidos o errores de baudrate.
 * Si el contador se clava en 1, es probable que el micro esté entrando AQUÍ.
 */
/* En GPS_Manager.c */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        if (gps_byte == '\n') {

            if (gps_idx > 5) {
                gps_buffer[gps_idx] = '\0';

                strncpy(cansatData.gps_sentence,
                        gps_buffer,
                        sizeof(cansatData.gps_sentence)-1);

                cansatData.gps_sentence[
                    sizeof(cansatData.gps_sentence)-1
                ] = '\0';
            }

            gps_idx = 0;
        }
        else if (gps_byte != '\r') {

            if (gps_idx < sizeof(gps_buffer)-1) {
                gps_buffer[gps_idx++] = gps_byte;
            } else {
                gps_idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &gps_byte, 1);
    }
}
