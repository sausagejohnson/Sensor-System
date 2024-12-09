#include "gps.h"
//#include "buffer.h"

uint8_t numberOfAttempts = 0;

void gps_DownloadDateTimeViaSatellite(RTC_HandleTypeDef rtc){

	uint8_t timeBuffer[13] = {'\0'};
	gpsBuffer_ExtractDateTimeFromBuffer(timeBuffer);


	if (timeBuffer[0] != '\0'){

		//convert each string to num and put in to a date struct
		char hourString[3] = {(char)timeBuffer[0], (char)timeBuffer[1], '\0'};
		char minuteString[3] = {(char)timeBuffer[2], (char)timeBuffer[3], '\0'};
		char secondString[3] = {(char)timeBuffer[4], (char)timeBuffer[5], '\0'};
		char dayString[3] = {(char)timeBuffer[6], (char)timeBuffer[7], '\0'};
		char monthString[3] = {(char)timeBuffer[8], (char)timeBuffer[9], '\0'};
		char yearString[3] = {(char)timeBuffer[10], (char)timeBuffer[11], '\0'};

		RTC_TimeTypeDef currentTime;
		RTC_DateTypeDef currentDate;

		currentTime.Hours = (unsigned char)strtol(hourString, NULL, 10);
		currentTime.Minutes = (unsigned char)strtol(minuteString, NULL, 10);
		currentTime.Seconds = (unsigned char)strtol(secondString, NULL, 10);

		currentDate.Date = (unsigned char)strtol(dayString, NULL, 10);
		currentDate.Month = (unsigned char)strtol(monthString, NULL, 10);
		currentDate.Year = (unsigned char)strtol(yearString, NULL, 10);
		//Setting to RTC_WEEKDAY_SUNDAY in order to generate the correct year.
		//I have no way of knowing what day of the week my time and date is.
		//I had hoped this library would be able to calculate that.
		//For now, for the sake of year, I'll make it Sunday.
		currentDate.WeekDay = RTC_WEEKDAY_SUNDAY;

		HAL_RTC_SetTime(&rtc, &currentTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&rtc, &currentDate, RTC_FORMAT_BIN);

		//Read and clear
		uint8_t readBuffer[128];
		gpsBuffer_ReadChunkFromCircularBuffer(readBuffer);

	}

	numberOfAttempts++;

}

int gps_TimeSyncRequired(RTC_HandleTypeDef rtc){
	if (numberOfAttempts > 10)
		return 0;

	RTC_DateTypeDef currentDate;
	HAL_RTC_GetDate(&rtc, &currentDate, RTC_FORMAT_BIN);

	//System year of 0 determines if sync is required
	return currentDate.Year == 0;
}

void gps_ResetSyncAttempts(){
	numberOfAttempts = 0;
}
