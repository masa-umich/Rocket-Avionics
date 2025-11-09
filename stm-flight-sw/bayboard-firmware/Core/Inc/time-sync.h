/*
 * time-sync.h
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#ifndef INC_TIME_SYNC_H_
#define INC_TIME_SYNC_H_

#include "main.h"
#include "time.h"
#include "string.h"
#include "stdio.h"
#include "log_errors.h"
#include "logging.h"

void set_system_time(uint32_t sec, uint32_t us);

uint64_t get_rtc_time();

void get_iso_time(char *outbuf);

void timesync_setup(RTC_HandleTypeDef *rtc_handle);

#endif /* INC_TIME_SYNC_H_ */
