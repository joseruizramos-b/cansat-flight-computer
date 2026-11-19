#include "Lora.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/*
 * Este proyecto usará USART2 para hablar con el RYLR998.
 */
#define LORA_UART huart2

static void LoRa_FlushRx(void)
{
#if LORA_USE_RESPONSE
    uint8_t dummy;

    /*
     * Limpiamos posibles respuestas viejas:
     * +OK, +ERR, basura de arranque, etc.
     */
    while (HAL_UART_Receive(&LORA_UART, &dummy, 1, 5) == HAL_OK)
    {
        /* No hacemos nada, solo vaciar RX */
    }
#endif
}

static HAL_StatusTypeDef LoRa_ReadResponse(char *response, uint16_t response_size, uint32_t timeout)
{
#if LORA_USE_RESPONSE

    uint32_t start_time = HAL_GetTick();
    uint16_t idx = 0;
    uint8_t rx_byte;

    if (response == NULL || response_size == 0)
    {
        return HAL_ERROR;
    }

    memset(response, 0, response_size);

    while ((HAL_GetTick() - start_time) < timeout)
    {
        if (HAL_UART_Receive(&LORA_UART, &rx_byte, 1, 20) == HAL_OK)
        {
            if (idx < response_size - 1)
            {
                response[idx++] = (char)rx_byte;
                response[idx] = '\0';
            }

            /*
             * Respuestas comunes del RYLR998:
             * +OK
             * +ERR=...
             * +READY
             */
            if (strstr(response, "+OK") != NULL ||
                strstr(response, "+ERR") != NULL ||
                strstr(response, "+READY") != NULL)
            {
                return HAL_OK;
            }
        }
    }

    if (idx > 0)
    {
        return HAL_OK;
    }

    return HAL_TIMEOUT;

#else

    /*
     * Si no estás leyendo RX del LoRa, no esperamos respuesta.
     */
    return HAL_OK;

#endif
}

HAL_StatusTypeDef LoRa_SendCommand(const char *cmd, char *response, uint16_t response_size, uint32_t timeout)
{
    char tx_buffer[320];
    int len;
    HAL_StatusTypeDef status;

    if (cmd == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Todo comando AT del RYLR998 debe terminar con \r\n
     */
    len = snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", cmd);

    if (len <= 0 || len >= (int)sizeof(tx_buffer))
    {
        return HAL_ERROR;
    }

    LoRa_FlushRx();

    status = HAL_UART_Transmit(&LORA_UART, (uint8_t *)tx_buffer, len, timeout);

    if (status != HAL_OK)
    {
        return status;
    }

#if LORA_USE_RESPONSE
    return LoRa_ReadResponse(response, response_size, timeout);
#else
    return HAL_OK;
#endif
}

HAL_StatusTypeDef LoRa_Init(void)
{
    char cmd[80];
    char response[120];
    HAL_StatusTypeDef status;

    /*
     * Pequeña espera para que el modulo RYLR998 arranque bien.
     */
    HAL_Delay(1000);

    /*
     * 1. Probar comunicacion AT
     */
    status = LoRa_SendCommand("AT", response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 2. Poner direccion del modulo del CanSat
     */
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d", LORA_MY_ADDRESS);
    status = LoRa_SendCommand(cmd, response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 3. Poner ID de red
     */
    snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d", LORA_NETWORK_ID);
    status = LoRa_SendCommand(cmd, response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 4. Poner frecuencia.
     * Para RYLR998 normalmente usamos 915 MHz.
     */
    snprintf(cmd, sizeof(cmd), "AT+BAND=%lu", LORA_BAND_HZ);
    status = LoRa_SendCommand(cmd, response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 5. Parametros RF.
     *
     * AT+PARAMETER=9,7,1,12
     *
     * SF = 9
     * BW = 7 -> 125 kHz
     * CR = 1 -> 4/5
     * Preamble = 12
     *
     * Es una configuracion inicial estable.
     */
    status = LoRa_SendCommand("AT+PARAMETER=9,7,1,12", response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 6. Potencia RF.
     * El modulo permite hasta 22 dBm, pero empezamos con 20 dBm.
     */
    snprintf(cmd, sizeof(cmd), "AT+CRFOP=%d", LORA_RF_POWER_DBM);
    status = LoRa_SendCommand(cmd, response, sizeof(response), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef LoRa_SendData(const char *payload)
{
    char cmd[320];
    char response[120];
    uint16_t payload_len;
    int len;

    if (payload == NULL)
    {
        return HAL_ERROR;
    }

    payload_len = strlen(payload);

    /*
     * El RYLR998 permite maximo 240 bytes de payload en AT+SEND.
     */
    if (payload_len == 0 || payload_len > 240)
    {
        return HAL_ERROR;
    }

    /*
     * Formato:
     * AT+SEND=DESTINO,LONGITUD,DATOS
     */
    len = snprintf(cmd, sizeof(cmd),
                   "AT+SEND=%d,%u,%s",
                   LORA_DEST_ADDRESS,
                   payload_len,
                   payload);

    if (len <= 0 || len >= (int)sizeof(cmd))
    {
        return HAL_ERROR;
    }

    return LoRa_SendCommand(cmd, response, sizeof(response), 1500);
}

HAL_StatusTypeDef LoRa_Test(void)
{
    return LoRa_SendData("HELLO_CANSAT");
}
