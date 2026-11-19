#include "mission_data.h"
#include "cmsis_os.h" // Requerido para el Mutex

// Instancia global privada de los datos
static MissionData_t atisque_storage;

// ID del Mutex para proteger los datos
osMutexId_t missionDataMutexHandle;

void MissionData_Init(void) {
    const osMutexAttr_t mutex_attr = {
      "MissionDataMutex",
      osMutexRecursive | osMutexPrioInherit,
      NULL, 0
    };
    missionDataMutexHandle = osMutexNew(&mutex_attr);
}

void MissionData_Update(MissionData_t *newData) {
    if (osMutexAcquire(missionDataMutexHandle, osWaitForever) == osOK) {
        atisque_storage = *newData;
        osMutexRelease(missionDataMutexHandle);
    }
}

void MissionData_Read(MissionData_t *buffer) {
    if (osMutexAcquire(missionDataMutexHandle, osWaitForever) == osOK) {
        *buffer = atisque_storage;
        osMutexRelease(missionDataMutexHandle);
    }
}
