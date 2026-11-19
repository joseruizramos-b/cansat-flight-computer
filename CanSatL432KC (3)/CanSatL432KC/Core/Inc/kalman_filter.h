#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

typedef struct {
    float Q; // Ruido de proceso
    float R; // Ruido de medición
    float X; // Valor estimado
    float P; // Error de covarianza
    float K; // Ganancia de Kalman
} Kalman_t;

void Kalman_Init(Kalman_t *k, float Q, float R);
float Kalman_Update(Kalman_t *k, float measurement);

#endif
