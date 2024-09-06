/*
 * NEO-M92-00B.h
 *
 *  Created on: Dec 1, 2023
 *      Author: Evan Eidt
 */

#pragma once
#ifndef NEO_M92_00B_H
#define NEO_M92_00B_H

#include "stm32h7xx_hal.h"
#include "semphr.h"

#include "minmea.h"

#define BUFFER_SIZE 100

typedef struct gps_data {
	float time; // seconds
	float latitude; // decimal degrees
	float longitude; // decimal degrees
	int fix_quality;
	int sats_tracked;
	float hdop;
	float altitude; // meters, according to reference site
} gps_data;

typedef struct gps_handler {
    UART_HandleTypeDef * huart; // UART handler
    uint8_t uart_rx_byte;
    SemaphoreHandle_t semaphore;

    char rx_buffer_1[BUFFER_SIZE];
    char rx_buffer_2[BUFFER_SIZE];

    uint8_t rx_buffer_1_pos;
    uint8_t rx_buffer_2_pos;

    uint8_t active_rx_buffer;
} gps_handler;

// Initializes the gps module, returns 0 if okay and -1 on error
int init_gps(gps_handler* hgps);

// IRQ callback function
void irq_gps_callback(gps_handler* hgps);

// Returns a struct with parsed GPS data
void parse_gps_sentence(const char* sentence, gps_data* gps);

/*
 * UBLOX stuff
 */

#define UBLOXCFG_CFG_NAVSPG_DYNMODEL_ID                    0x20110021                               //!< ID of CFG-NAVSPG-DYNMODEL
#define UBLOXCFG_CFG_NAVSPG_DYNMODEL_STR                   "CFG-NAVSPG-DYNMODEL"                    //!< Name of CFG-NAVSPG-DYNMODEL
#define UBLOXCFG_CFG_NAVSPG_DYNMODEL_TYPE                  E1                                       //!< Type of CFG-NAVSPG-DYNMODEL
//! constants for CFG-NAVSPG-DYNMODEL
typedef enum UBLOXCFG_CFG_NAVSPG_DYNMODEL_e
{
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_PORT                    = 0,                                       //!< Portable
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_STAT                    = 2,                                       //!< Stationary
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_PED                     = 3,                                       //!< Pedestrian
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_AUTOMOT                 = 4,                                       //!< Automotive
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_SEA                     = 5,                                       //!< Sea
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_AIR1                    = 6,                                       //!< Airborne with <1g acceleration
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_AIR2                    = 7,                                       //!< Airborne with <2g acceleration
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_AIR4                    = 8,                                       //!< Airborne with <4g acceleration
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_WRIST                   = 9,                                       //!< Wrist-worn watch (not available in all products)
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_BIKE                    = 10,                                      //!< Motorbike (not available in all products)
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_LAWNMOWER               = 11,                                      //!< Lawn mower (not available in all products)
    UBLOXCFG_CFG_NAVSPG_DYNMODEL_SCOOTER                 = 12                                       //!< Scooters (hyper hyper) (not available in all products)
} UBLOXCFG_CFG_NAVSPG_DYNMODEL_t;

#endif