#ifndef MISSION_DATA_H_
#define MISSION_DATA_H_

#include <stdint.h>

// Definición de la estructura de telemetría ATISQUE
typedef struct {
    // Datos GY-91
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float pressure, temperature;

    // Datos GPS
    double latitude, longitude;
    float altitude;
    uint8_t satellites;

    // Control y Estado de la FSM
    uint8_t current_state;
    uint32_t timestamp;
} MissionData_t;

// Prototipos de funciones seguras (Thread-Safe)
void MissionData_Init(void);
void MissionData_Update(MissionData_t *newData);
void MissionData_Read(MissionData_t *buffer);

#endif
