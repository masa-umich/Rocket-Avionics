/*
 * time-sync.c
 *
 *  Created on: Oct 16, 2025
 *      Author: felix
 */

#include "time-sync.h"

SemaphoreHandle_t RTC_mutex; // For safe concurrent access of the RTC
RTC_HandleTypeDef *rtc;

// Callback for LWIP SNTP
void set_system_time(uint32_t sec, uint32_t us) {
	//HAL_PWR_EnableBkUpAccess(); // This should be enabled since we're using HSE divided as the RTC source - now done in main()
	//uint64_t ns_time = get_rtc_time(); // debug
	//float diff = (((sec * 1e9) + (us * 1e3)) - ns_time) / 1000000000.0; // debug

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	struct tm *tm_time;

	time_t epoch = sec;
	tm_time = gmtime(&epoch);

	sTime.Hours = tm_time->tm_hour;
	sTime.Minutes = tm_time->tm_min;
	sTime.Seconds = tm_time->tm_sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	//sTime.SubSeconds = 6249 - ((us / 1000000.0) * 6250);

	sDate.Year = tm_time->tm_year - 100;
	sDate.Month = tm_time->tm_mon + 1;
	sDate.Date = tm_time->tm_mday;
	sDate.WeekDay = (tm_time->tm_wday == 0) ? 7 : tm_time->tm_wday;

	if(xSemaphoreTake(RTC_mutex, 5) == pdPASS) {
		// These constants have to do with the counters associated with the RTC
		// 6250 is the 1hz counter, so every increment of that counter is 1/6250th of a second
		uint32_t subsecond_shift = 6249 - ((us / 1000000.0) * 6250);

		//uint32_t subsecond_shift = 6249ULL - ((((uint64_t) us) * ((uint64_t) 6250)) / 1000000ULL); // This works too but I think above is more memory efficient
		taskENTER_CRITICAL();
		HAL_RTC_SetDate(rtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(rtc, &sTime, RTC_FORMAT_BIN);
		taskEXIT_CRITICAL();

		HAL_RTCEx_SetSynchroShift(rtc, RTC_SHIFTADD1S_SET, subsecond_shift); // Shift sub-seconds register to do fine grain time sync

		//while ((hrtc.Instance->ISR & RTC_ISR_SHPF) != 0) {}
		//osDelay(1000);
		/*RTC_TimeTypeDef sTime1;
		RTC_DateTypeDef sDate1;

		HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate1, RTC_FORMAT_BIN);*/

		// Wait for shadow register to sync
		__HAL_RTC_WRITEPROTECTION_DISABLE(rtc);
		HAL_RTC_WaitForSynchro(rtc);
		__HAL_RTC_WRITEPROTECTION_ENABLE(rtc);;

		xSemaphoreGive(RTC_mutex);
	}
	else {
		// time not set
		log_message(ERR_RTC_NOT_SET, -1);
	}
}


/**
 * Get Unix timestamp in nanoseconds
 * Returns 0 if the RTC could not be accessed or if the RTC has not been set yet
 */
uint64_t get_rtc_time() {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(xSemaphoreTake(RTC_mutex, 5) == pdPASS) {
		if(__HAL_RTC_IS_CALENDAR_INITIALIZED(rtc) == 0) {
			xSemaphoreGive(RTC_mutex);
			// RTC not set yet
			return 0;
		}
		HAL_RTC_GetTime(rtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(rtc, &sDate, RTC_FORMAT_BIN);

		xSemaphoreGive(RTC_mutex);
	}
	else {
		return 0;
	}

	struct tm time;
	time.tm_year = sDate.Year + 100;
	time.tm_mon  = sDate.Month - 1;
	time.tm_mday = sDate.Date;
	time.tm_hour = sTime.Hours;
	time.tm_min  = sTime.Minutes;
	time.tm_sec  = sTime.Seconds;
	time.tm_isdst = 0;

	time_t sec = mktime(&time);
	int32_t us = ((((int32_t) 6249) - ((int32_t) sTime.SubSeconds)) / 6250.0) * 1e6;
	return (sec * 1e9) + (us * 1e3);
}

// outbuf MUST have at least 24 bytes of space. It will not be null terminated
void get_iso_time(char *outbuf) {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(xSemaphoreTake(RTC_mutex, 5) == pdPASS) {
		if(__HAL_RTC_IS_CALENDAR_INITIALIZED(rtc) == 0) {
			xSemaphoreGive(RTC_mutex);
			// RTC not set yet
			memcpy(outbuf, "0000-00-00T00:00:00.000Z", 24);
			return;
		}
		HAL_RTC_GetTime(rtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(rtc, &sDate, RTC_FORMAT_BIN);

		xSemaphoreGive(RTC_mutex);
	}
	else {
		memcpy(outbuf, "0000-00-00T00:00:00.000Z", 24);
		return;
	}
	int16_t ms = ((((int32_t) 6249) - ((int32_t) sTime.SubSeconds)) * 1e3) / 6250;
	// I wrote this before I discovered snprintf could do this in one line lol
	// This section will throw a lot of compiler warnings, but it's okay
	snprintf(outbuf, 5, "%04u", 2000 + (uint16_t) sDate.Year);
	outbuf[4] = '-';
	snprintf(&outbuf[5], 3, "%02u", sDate.Month);
	outbuf[7] = '-';
	snprintf(&outbuf[8], 3, "%02u", sDate.Date);
	outbuf[10] = 'T';
	snprintf(&outbuf[11], 3, "%02u", sTime.Hours);
	outbuf[13] = ':';
	snprintf(&outbuf[14], 3, "%02u", sTime.Minutes);
	outbuf[16] = ':';
	snprintf(&outbuf[17], 3, "%02u", sTime.Seconds);
	outbuf[19] = '.';
	snprintf(&outbuf[20], 4, "%03u", ms);
	outbuf[23] = 'Z';
}

void timesync_setup(RTC_HandleTypeDef *rtc_handle) {
	RTC_mutex = xSemaphoreCreateMutex();
	rtc = rtc_handle;
}
