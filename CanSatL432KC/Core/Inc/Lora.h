#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>

/*
 * CONFIGURACION DEL ENLACE LoRa
 *
 * CanSat:
 *   ADDRESS = 1
 *
 * Estacion en tierra:
 *   ADDRESS = 2
 *
 * Ambos deben tener:
 *   mismo NETWORKID
 *   mismo BAND
 *   mismos PARAMETER
 */

#define LORA_MY_ADDRESS        1
#define LORA_DEST_ADDRESS      2
#define LORA_NETWORK_ID        6
#define LORA_BAND_HZ           915000000UL
#define LORA_RF_POWER_DBM      20

/*
 * Si tienes conectado TX y RX del LoRa al STM32, deja esto en 1.
 * Si por ahora SOLO conectaste TX del STM32 al RX del LoRa, ponlo en 0.
 *
 * Recomendado para el proyecto final: 1
 */
#define LORA_USE_RESPONSE      1

HAL_StatusTypeDef LoRa_Init(void);
HAL_StatusTypeDef LoRa_SendCommand(const char *cmd, char *response, uint16_t response_size, uint32_t timeout);
HAL_StatusTypeDef LoRa_SendData(const char *payload);
HAL_StatusTypeDef LoRa_Test(void);

#ifdef __cplusplus
}
#endif

#endif
