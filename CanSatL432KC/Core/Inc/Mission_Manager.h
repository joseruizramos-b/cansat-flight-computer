#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

typedef enum {
    STATE_IDLE = 0,
    STATE_ARMED,
    STATE_FALLING,
    STATE_TRIGGERED,
	STATE_MISSION_FINISHED  // <-- Nuevo estado final
} MissionState_t;

extern MissionState_t currentState;

void Mission_Init(void);
void Mission_Update(void);

#endif
