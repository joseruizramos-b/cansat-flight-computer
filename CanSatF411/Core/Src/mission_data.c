// --- mission_data.c ---
#include "mission_data.h"
#include "cmsis_os2.h"
#include <string.h>

// SOLAMENTE UNA INSTANCIA REAL
static MissionData_t atisque_data;
osMutexId_t missionDataMutexHandle;

void MissionData_Init(void) {
    const osMutexAttr_t ThreadSafe_Mutex_attr = { "MissionMutex", osMutexRecursive, NULL, 0U };
    missionDataMutexHandle = osMutexNew(&ThreadSafe_Mutex_attr);

    memset(&atisque_data, 0, sizeof(MissionData_t));
    strcpy(atisque_data.gps_sentence, "WAITING_GPS");
}

void MissionData_Update(MissionData_t *newData) {
    if (missionDataMutexHandle != NULL && osMutexAcquire(missionDataMutexHandle, osWaitForever) == osOK) {
        atisque_data = *newData;
        osMutexRelease(missionDataMutexHandle);
    }
}

void MissionData_Read(MissionData_t *buffer) {
    if (missionDataMutexHandle != NULL && osMutexAcquire(missionDataMutexHandle, osWaitForever) == osOK) {
        *buffer = atisque_data;
        osMutexRelease(missionDataMutexHandle);
    }
}

MissionData_t* MissionData_GetPointer(void) {
    return &atisque_data;
}
