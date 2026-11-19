// --- mission_data.h ---
#ifndef MISSION_DATA_H_
#define MISSION_DATA_H_

#include <stdint.h>

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float pressure, altitude, temperature, baro_altitude;
    double latitude, longitude;
    float gps_altitude;
    uint8_t satellites;
    char gps_sentence[80];
    uint8_t current_state;
    uint32_t timestamp;
} MissionData_t;

void MissionData_Init(void);
void MissionData_Update(MissionData_t *newData);
void MissionData_Read(MissionData_t *buffer);
MissionData_t* MissionData_GetPointer(void);

#endif
