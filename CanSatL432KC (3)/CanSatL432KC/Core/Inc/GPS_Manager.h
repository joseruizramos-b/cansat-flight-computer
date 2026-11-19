#ifndef INC_GPS_MANAGER_H_
#define INC_GPS_MANAGER_H_

#include "main.h"
#include "Mission_Data.h"
void GPS_Init(void);
void GPS_ProcessChar(uint8_t rx_data);

#endif
