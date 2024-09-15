
#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "buffer.h"

//static uint8_t localTimeSyncRequired = 1;

//static RTC_TimeTypeDef lastAttemptedTime; //use this to set length of time attempted and when to reattempt

void gps_DownloadDateTimeViaSatellite(RTC_HandleTypeDef rtc);
int gps_TimeSyncRequired(RTC_HandleTypeDef rtc);

#endif /* GPS_H_ */
