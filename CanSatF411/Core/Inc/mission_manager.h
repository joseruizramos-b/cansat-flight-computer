#ifndef MISSION_MANAGER_H_
#define MISSION_MANAGER_H_

#include "stm32f4xx_hal.h"
#include "mission_data.h"

// Inicializa la lógica de la misión (Timers, estados iniciales)
void Mission_Init(void);

// Esta es la función que debe llamar la Task de FreeRTOS en cada ciclo
void Mission_RunIteration(void);

#endif
