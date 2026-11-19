/* Core/Src/bridge_manager.c */

#include "bridge_manager.h"
#include "Mission_Data.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include "Lora.h"
#include <stdio.h>

extern Mission_Data_t cansatData;

void Bridge_Update(void)
{
    static uint32_t packet_counter = 0;

    char payload[220];
    int len;

    /*
     * Trama compacta para LoRa.
     *
     * No mandamos texto largo tipo "| STATE: ..."
     * porque LoRa no debe ir saturado.
     *
     * Tampoco metemos GPS completo por ahora,
     * porque las sentencias NMEA son largas.
     */
    len = snprintf(payload, sizeof(payload),
                   "N=%lu;S=%d;AX=%.2f;AY=%.2f;AZ=%.2f;"
                   "GX=%.1f;GY=%.1f;GZ=%.1f;"
                   "T=%.1f;PR=%.1f;ALT=%.1f;"
                   "R=%.1f;PIT=%.1f;Y=%.1f",
                   (unsigned long)packet_counter,
                   currentState,
                   cansatData.ax,
                   cansatData.ay,
                   cansatData.az,
                   cansatData.gyro_x,
                   cansatData.gyro_y,
                   cansatData.gyro_z,
                   cansatData.temperature,
                   cansatData.pressure,
                   cansatData.altitude,
                   cansatData.roll,
                   cansatData.pitch,
                   cansatData.yaw);

    if (len > 0 && len < (int)sizeof(payload))
    {
        LoRa_SendData(payload);
        packet_counter++;
    }
}
