/*
 * time-sync.h
 *
 *  Created on: Oct 16, 2025
 *      Author: felix
 */

#ifndef INC_TIME_SYNC_H_
#define INC_TIME_SYNC_H_

#include "main.h"
#include "time.h"
#include "string.h"
#include "stdio.h"

void timesync_setup();

void get_iso_time(char *outbuf);

uint64_t get_rtc_time();

void set_system_time(uint32_t sec, uint32_t us);

#endif /* INC_TIME_SYNC_H_ */
