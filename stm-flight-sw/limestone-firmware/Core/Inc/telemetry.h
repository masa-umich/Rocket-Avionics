/*
 * telemetry.h
 *
 *  Created on: Oct 12, 2025
 *      Author: felix
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"
#include "log_errors.h"
#include "logging.h"
#include "M24256E.h"
#include "MS5611.h"
#include "ADS1120.h"
#include "MAX11128.h"
#include "LSM6DSO32XTR.h"
#include "utils.h"

void TelemetryTask(void *argument);

#endif /* INC_TELEMETRY_H_ */
