#include "kalman_filter.h"

void Kalman_Init(Kalman_t *k, float Q, float R) {
    k->Q = Q;
    k->R = R;
    k->P = 0.1f;
    k->X = 0.0f;
}

float Kalman_Update(Kalman_t *k, float measurement) {
    // Predicción
    k->P += k->Q;
    // Actualización (Corrección)
    k->K = k->P / (k->P + k->R);
    k->X += k->K * (measurement - k->X);
    k->P *= (1.0f - k->K);
    return k->X;
}
