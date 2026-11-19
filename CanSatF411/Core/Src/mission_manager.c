// --- mission_manager.c ---
#include "mission_manager.h"
#include "sensors_manager.h"
#include "bridge_manager.h"

void Mission_Init(void) {
    Sensors_Init(); // Inicializamos hardware
}

void Mission_RunIteration(void) {
    // 1. Apuntamos a la única verdad
    MissionData_t *current_data = MissionData_GetPointer();

    // 2. Actualizamos tiempo
    current_data->timestamp = HAL_GetTick();

    // 3. Leemos sensores
    Sensors_ReadIMU(current_data);
    Sensors_ReadEnv(current_data);

    // 4. Enviamos telemetría
    Bridge_SendTelemetry(current_data);

    // 5. LED PA5 (Si parpadea, el código NO está bloqueado)
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
