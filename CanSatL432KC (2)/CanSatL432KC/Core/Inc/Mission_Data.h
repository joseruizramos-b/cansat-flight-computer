/* Core/Inc/Mission_Data.h */
#ifndef INC_MISSION_DATA_H_
#define INC_MISSION_DATA_H_

#include <stdint.h>

typedef struct {
    // Datos de Sensores (MPU/BMP)
    float pressure, altitude, temperature;
    float ax, ay, az;
    float gyro_x, gyro_y, gyro_z;
    // Datos de GPS - NUEVO
    float latitude, longitude;
    uint8_t satellites;
    char gps_sentence[80]; // Buffer para la trama cruda $GPGGA
    float pitch, roll, yaw;
        float ax_f, ay_f, az_f; // Aceleración filtrada por Kalman 1D
    // Control
    uint8_t mission_state;
} Mission_Data_t;

#endif
