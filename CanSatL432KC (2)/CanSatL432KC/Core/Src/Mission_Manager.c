#include "Mission_Data.h"
#include "bridge_manager.h"
#include "GPS_Manager.h"
#include "Sensors_Manager.h"
#include "Mission_Manager.h"
#include "kalman_filter.h"
#include "ekf_orientation.h"
#include "tim.h"
#include <string.h>

MissionState_t currentState = STATE_IDLE;

float altura_base = 0;
float descenso = 0;
static uint32_t triggerStartTime = 0; // Para guardar el momento del disparo
uint8_t contador_confirmacion = 0;

void Mission_Init(void) {
    cansatData.mission_state = 0;

    currentState = STATE_IDLE;

    altura_base = 0;
    descenso = 0;
    contador_confirmacion = 0;

    strcpy(cansatData.gps_sentence, "BUSCANDO_SENAL...");
}

void Mission_Update(void) {

    float alt = cansatData.altitude;

    if (currentState == STATE_FALLING || currentState == STATE_TRIGGERED) {

            // Usamos los ángulos "limpios" del EKF que inyectamos en Sensors_Manager
            float target_pitch = cansatData.pitch;
            float target_roll = cansatData.roll;

            // Mapeo Proporcional: Ángulo -> PWM
            // Fórmula: Centro(1500) + (Ángulo * Sensibilidad)
            // Si el EKF da grados, 1 grado son aprox 5.5 ticks (1000 ticks / 180 grados)
            uint32_t pulse_s1 = 1500 + (uint32_t)(target_pitch * 5.55f);
            uint32_t pulse_s2 = 1500 + (uint32_t)(target_roll * 5.55f);

            // Saturación de seguridad (No forzar el servo más allá de 1ms - 2ms)
            if (pulse_s1 < 1000) pulse_s1 = 1000; if (pulse_s1 > 2000) pulse_s1 = 2000;
            if (pulse_s2 < 1000) pulse_s2 = 1000; if (pulse_s2 > 2000) pulse_s2 = 2000;

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_s1);
            __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, pulse_s2);

        } else {
            // En IDLE o ARMED, servos en posición neutra (90°) para ahorrar torque
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
            __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 1500);
        }



    switch(currentState) {

        case STATE_IDLE:

            // Esperamos estabilización
            altura_base = alt;

            // Pasamos a armado después de unos ciclos
            static int init_counter = 0;
            init_counter++;

            if (init_counter > 20) {
                currentState = STATE_ARMED;
            }

        break;


        case STATE_ARMED:

            // Calculamos descenso
            descenso = altura_base - alt;

            // Detectar que realmente empezó a caer
            if (descenso > 2.0f) { // 2 metros
                currentState = STATE_FALLING;
            }

        break;


        case STATE_FALLING:
                    descenso = altura_base - alt;
                    if (descenso > 10.0f) {
                        contador_confirmacion++;
                        if (contador_confirmacion > 3) {
                            // 1. Activar salida (PWM)
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);

                            // 2. Capturar el momento exacto del disparo
                            triggerStartTime = HAL_GetTick();

                            currentState = STATE_TRIGGERED;
                        }
                    } else {
                        contador_confirmacion = 0;
                    }
                break;


        case STATE_TRIGGERED:
                    /* Lógica de temporización sin bloqueo (Non-blocking) */
                    if ((HAL_GetTick() - triggerStartTime) >= 2000) {

                        // 1. Apagar el actuador/pin
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

                        // 2. Mover a estado de misión finalizada
                        currentState = STATE_MISSION_FINISHED;
                    }
                break;

        case STATE_MISSION_FINISHED:
                    // El sistema se queda aquí hasta el reinicio físico.
                    // Esto evita que ruidos en el sensor vuelvan a disparar el estado 3.
                break;
    }
}


