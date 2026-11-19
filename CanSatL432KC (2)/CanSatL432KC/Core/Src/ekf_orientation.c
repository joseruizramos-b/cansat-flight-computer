#include "ekf_orientation.h"
#include <math.h>
#include <string.h>

static void normalize_quaternion(float *q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm < 0.0001f) return;
    for (int i = 0; i < 4; i++) q[i] /= norm;
}

void EKF_Init(EKF_Orientation *ekf, float dt, float gyro_noise, float accel_noise) {
    memset(ekf, 0, sizeof(EKF_Orientation));
    ekf->q[0] = 1.0f;
    ekf->dt = dt;
    ekf->gyro_noise = gyro_noise;
    ekf->accel_noise = accel_noise;
    for (int i = 0; i < 4; i++) ekf->P[i][i] = 0.01f;
}

void EKF_Predict(EKF_Orientation *ekf, float gx, float gy, float gz) {
    float dt = ekf->dt;
    float *q = ekf->q;
    float dq[4];

    // Derivada del cuaternión (Matemática de señales)
    dq[0] = 0.5f * (-q[1]*gx - q[2]*gy - q[3]*gz);
    dq[1] = 0.5f * ( q[0]*gx + q[2]*gz - q[3]*gy);
    dq[2] = 0.5f * ( q[0]*gy - q[1]*gz + q[3]*gx);
    dq[3] = 0.5f * ( q[0]*gz + q[1]*gy - q[2]*gx);

    for (int i = 0; i < 4; i++) q[i] += dq[i] * dt;
    normalize_quaternion(q);

    for (int i = 0; i < 4; i++) ekf->P[i][i] += ekf->gyro_noise;
}

void EKF_Update(EKF_Orientation *ekf, float ax, float ay, float az) {
    float *q = ekf->q;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    // Predicción de gravedad (Fusión con acelerómetro)
    float gx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float gy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    float ex = (ay*gz - az*gy);
    float ey = (az*gx - ax*gz);
    float ez = (ax*gy - ay*gx);

    float gain = 2.0f;
    q[1] += gain * ex * ekf->dt;
    q[2] += gain * ey * ekf->dt;
    q[3] += gain * ez * ekf->dt;

    normalize_quaternion(q);
    for (int i = 0; i < 4; i++) ekf->P[i][i] -= ekf->accel_noise;
}

void EKF_GetEulerAngles(EKF_Orientation *ekf, float *roll, float *pitch, float *yaw) {
    float *q = ekf->q;
    // Conversión con corrección de tipos y radianes a grados (Multiplicado por 57.29)
    *roll  = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * 57.2958f;
    *pitch = asinf(2.0f * (q[0]*q[2] - q[3]*q[1])) * 57.2958f;
    *yaw   = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * 57.2958f;
}
