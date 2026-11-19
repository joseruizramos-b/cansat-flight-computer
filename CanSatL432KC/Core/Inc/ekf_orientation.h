#ifndef EKF_ORIENTATION_H
#define EKF_ORIENTATION_H

#include "stm32l4xx_hal.h" // ¡Ojo! Cambiado a L4

typedef struct {
    float q[4];         // Cuaternión
    float P[4][4];      // Covarianza
    float dt;           // Delta de tiempo
    float gyro_noise;
    float accel_noise;
} EKF_Orientation;

void EKF_Init(EKF_Orientation *ekf, float dt, float gyro_noise, float accel_noise);
void EKF_Predict(EKF_Orientation *ekf, float gx, float gy, float gz);
void EKF_Update(EKF_Orientation *ekf, float ax, float ay, float az);
void EKF_GetEulerAngles(EKF_Orientation *ekf, float *roll, float *pitch, float *yaw);

#endif
