/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MS5611.h"
#include "ADS1120.h"
#include "MAX11128.h"
#include "LSM6DSO32XTR.h"
#include "VLVs.h"
#include "sntp.h"
#include "time.h"
#include "api.h"
#include "string.h"
#include "server.h"
#include "messages.h"
#include "M24256E.h"
#include "utils.h"
#include "tftp_server.h"
#include "lmp_channels.h"
#include "math.h"
#include "W25N04KV.h"
#include "base64.h"
#include "lwip/udp.h"
#include "timers.h"
#include "log_errors.h"
//#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float pt1;
	float pt2;
	float pt3;
	float pt4;
	float pt5;
	float tc1;
	float tc2;
	float tc3;
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float gps_lat;
	float gps_long;
	float gps_alt;
	float bar1;
	float bar2;
	float bus24v_voltage;
	float bus24v_current;
	float bus12v_voltage;
	float bus12v_current;
	float bus5v_voltage;
	float bus5v_current;
	float bus3v3_voltage;
	float bus3v3_current;
	float vlv1_current;
	VLV_OpenLoad vlv1_old;
	float vlv2_current;
	VLV_OpenLoad vlv2_old;
	float vlv3_current;
	VLV_OpenLoad vlv3_old;

	uint64_t timestamp; // Only used to monitor how "fresh" the data is
} Flight_Computer_State_t;

typedef struct {
	float pt1;
	float pt2;
	float pt3;
	float pt4;
	float pt5;
	float pt6;
	float pt7;
	float pt8;
	float pt9;
	float pt10;
	float tc1;
	float tc2;
	float tc3;
	float tc4;
	float tc5;
	float tc6;
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float bar1;
	float bar2;
	float bus24v_voltage;
	float bus24v_current;
	float bus12v_voltage;
	float bus12v_current;
	float bus5v_voltage;
	float bus5v_current;
	float bus3v3_voltage;
	float bus3v3_current;
	float vlv1_current;
	VLV_OpenLoad vlv1_old;
	float vlv2_current;
	VLV_OpenLoad vlv2_old;
	float vlv3_current;
	VLV_OpenLoad vlv3_old;
	float vlv4_current;
	VLV_OpenLoad vlv4_old;
	float vlv5_current;
	VLV_OpenLoad vlv5_old;
	float vlv6_current;
	VLV_OpenLoad vlv6_old;
	float vlv7_current;
	VLV_OpenLoad vlv7_old;

	uint64_t timestamp;
} Bay_Board_State_t;

typedef struct {
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float bar1;
	float bar2;

	uint64_t timestamp;
} Flight_Recorder_State_t;

typedef struct {
	SemaphoreHandle_t fcState_access;
	SemaphoreHandle_t fcValve_access;
	Flight_Computer_State_t fcState;
	Valve fcValves[3];
	Valve_State_t fcValveStates[3];

	SemaphoreHandle_t bb1State_access;
	SemaphoreHandle_t bb1Valve_access;
	Bay_Board_State_t bb1State;
	Valve_State_t bb1ValveStates[7];

	SemaphoreHandle_t bb2State_access;
	SemaphoreHandle_t bb2Valve_access;
	Bay_Board_State_t bb2State;
	Valve_State_t bb2ValveStates[7];

	SemaphoreHandle_t bb3State_access;
	SemaphoreHandle_t bb3Valve_access;
	Bay_Board_State_t bb3State;
	Valve_State_t bb3ValveStates[7];

	SemaphoreHandle_t frState_access;
	Flight_Recorder_State_t frState;
} Rocket_State_t;

typedef struct {
	PT_t *pt1;
	PT_t *pt2;
	PT_t *pt3;
	PT_t *pt4;
	PT_t *pt5;

	uint8_t tc1_gain;
	uint8_t tc2_gain;
	uint8_t tc3_gain;

	VLV_Voltage vlv1_v;
	uint8_t vlv1_en;
	VLV_Voltage vlv2_v;
	uint8_t vlv2_en;
	VLV_Voltage vlv3_v;
	uint8_t vlv3_en;

	ip4_addr_t limewireIP;
	ip4_addr_t flightcomputerIP;
	ip4_addr_t bayboard1IP;
	ip4_addr_t bayboard2IP;
	ip4_addr_t bayboard3IP;
	ip4_addr_t flightrecordIP;
} EEPROM_conf_t;

typedef struct {
	IMU imu1_h;
	IMU imu2_h;

  	GPIO_MAX11128_Pinfo adc_h;

  	ADS_Main_t tc_main_h;
  	ADS_TC_t TCs[3];

  	MS5611 bar1_h;
  	MS5611 bar2_h;
  	MS5611_PROM_t prom1;
  	MS5611_PROM_t prom2;
} Sensors_t;

typedef struct {
	uint8_t *content;
	size_t len;
} errormsg_t;

typedef struct {
	TimerHandle_t buzzTimer;
	TimerHandle_t ledTimer;
} inittimers_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c5;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
SPI_HandleTypeDef hspi6;

UART_HandleTypeDef huart10;

/* Definitions for startTask */
osThreadId_t startTaskHandle;
const osThreadAttr_t startTask_attributes = {
  .name = "startTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t RTC_mutex; // For safe concurrent access of the RTC

Rocket_State_t Rocket_h; // Main rocket state handle
eeprom_t eeprom_h = {0};

extern struct netif gnetif;

size_t eeprom_cursor = 0;

ip4_addr_t fc_addr;

EEPROM_conf_t loaded_config = {0};
PT_t PT1_h = {0.5f, 5000.0f, 4.5f}; // Default
PT_t PT2_h = {0.5f, 5000.0f, 4.5f};
PT_t PT3_h = {0.5f, 5000.0f, 4.5f};
PT_t PT4_h = {0.5f, 5000.0f, 4.5f};
PT_t PT5_h = {0.5f, 5000.0f, 4.5f};

Sensors_t sensors_h = {0};

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetry_task_attr = {
  .name = "telemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t packetTaskHandle;
const osThreadAttr_t packet_task_attr = {
  .name = "packetTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t flashClearTaskHandle;
const osThreadAttr_t flash_clear_task_attr = {
  .name = "flashCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

W25N04KV_Flash flash_h = {0};
SemaphoreHandle_t flash_mutex;
SemaphoreHandle_t flash_clear_mutex;
uint8_t flashreadbuffer[2048 + 512];
uint16_t validflashbytes = 0;
uint8_t flashmsgtype = 0;

SemaphoreHandle_t errormsg_mutex;
uint8_t errormsgtimers[ERROR_MSG_TYPES / 2]; // Use 4 bits to store each timer

SemaphoreHandle_t perierrormsg_mutex;
uint8_t perierrormsgtimers[PERI_ERROR_MSG_TYPES];

struct netconn *errormsgudp = NULL;
SemaphoreHandle_t errorudp_mutex;

List* errorMsgList;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C5_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART10_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI6_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI3_Init(void);
void StartAndMonitor(void *argument);

/* USER CODE BEGIN PFP */
void FlashClearTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t getTimestamp() {
	return HAL_GetTick();
}

void toggleBuzzer(TimerHandle_t xTimer) {
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
}

void toggleLEDPins(TimerHandle_t xTimer) {
	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

void buzzerOff(TimerHandle_t xTimer) {
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);
}

void reset_board() {
	if(xSemaphoreTake(flash_mutex, 500) == pdPASS) {
		fc_finish_flash_write(&flash_h);
	}

	NVIC_SystemReset(); // This should never return
}

void log_flash_storage() {
	if(xSemaphoreTake(flash_mutex, 5) == pdPASS) {
		uint32_t used = 536870912UL - fc_get_bytes_remaining(&flash_h);
		xSemaphoreGive(flash_mutex);

		uint8_t percent = (used / 536870912.0f) * 100;
		if(used < 1024) {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 22];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B/512MB %u%%", used, percent);
	    	log_message(logmsg, -1);
		}
		else if(used < 1048576UL) {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 36];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu32 "KB/512MB %u%%", used, used >> 10, percent);
	    	log_message(logmsg, -1);
		}
		else {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 31];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu16 "MB/512MB %u%%", used, (uint16_t) (used >> 20), percent);
	    	log_message(logmsg, -1);
		}
	}
}

void clear_flash() {
	xSemaphoreGive(flash_clear_mutex); // signal to task to clear flash. This needs to be async since it takes almost 5 seconds to clear the flash
}

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
		uint32_t subsecond_shift = 6249 - ((us / 1000000.0) * 6250);
		//uint32_t subsecond_shift = 6249ULL - ((((uint64_t) us) * ((uint64_t) 6250)) / 1000000ULL); // This works too but I think above is more memory efficient
		taskENTER_CRITICAL();
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		taskEXIT_CRITICAL();

		HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_SET, subsecond_shift); // Shift sub-seconds register to do fine grain time sync

		//while ((hrtc.Instance->ISR & RTC_ISR_SHPF) != 0) {}
		//osDelay(1000);
		/*RTC_TimeTypeDef sTime1;
		RTC_DateTypeDef sDate1;

		HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate1, RTC_FORMAT_BIN);*/

		// Wait for shadow register to sync
		__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
		HAL_RTC_WaitForSynchro(&hrtc);
		__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);;

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
		if(__HAL_RTC_IS_CALENDAR_INITIALIZED(&hrtc) == 0) {
			xSemaphoreGive(RTC_mutex);
			// RTC not set yet
			return 0;
		}
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

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
		if(__HAL_RTC_IS_CALENDAR_INITIALIZED(&hrtc) == 0) {
			xSemaphoreGive(RTC_mutex);
			// RTC not set yet
			memcpy(outbuf, "0000-00-00T00:00:00.000Z", 24);
			return;
		}
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

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

// Pack telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int pack_fc_telemetry_msg(TelemetryMessage *msg, uint64_t timestamp, uint8_t timeout_ticks) {
	msg->board_id = BOARD_FC;
	msg->num_channels = FC_TELEMETRY_CHANNELS;
	msg->timestamp = timestamp;
	if(xSemaphoreTake(Rocket_h.fcState_access, timeout_ticks) == pdPASS) {
		msg->telemetry_data[FC_VLV1_CURRENT_I] = Rocket_h.fcState.vlv1_current;
		msg->telemetry_data[FC_VLV1_OLD_I] = Rocket_h.fcState.vlv1_old;
		msg->telemetry_data[FC_VLV2_CURRENT_I] = Rocket_h.fcState.vlv2_current;
		msg->telemetry_data[FC_VLV2_OLD_I] = Rocket_h.fcState.vlv2_old;
		msg->telemetry_data[FC_VLV3_CURRENT_I] = Rocket_h.fcState.vlv3_current;
		msg->telemetry_data[FC_VLV3_OLD_I] = Rocket_h.fcState.vlv3_old;
		msg->telemetry_data[FC_PT_1_I] = Rocket_h.fcState.pt1;
		msg->telemetry_data[FC_PT_2_I] = Rocket_h.fcState.pt2;
		msg->telemetry_data[FC_PT_3_I] = Rocket_h.fcState.pt3;
		msg->telemetry_data[FC_PT_4_I] = Rocket_h.fcState.pt4;
		msg->telemetry_data[FC_PT_5_I] = Rocket_h.fcState.pt5;
		msg->telemetry_data[FC_TC_1_I] = Rocket_h.fcState.tc1;
		msg->telemetry_data[FC_TC_2_I] = Rocket_h.fcState.tc2;
		msg->telemetry_data[FC_TC_3_I] = Rocket_h.fcState.tc3;
		msg->telemetry_data[FC_IMU1_X_I] = Rocket_h.fcState.imu1_A.XL_x;
		msg->telemetry_data[FC_IMU1_Y_I] = Rocket_h.fcState.imu1_A.XL_y;
		msg->telemetry_data[FC_IMU1_Z_I] = Rocket_h.fcState.imu1_A.XL_z;
		msg->telemetry_data[FC_IMU1_WP_I] = Rocket_h.fcState.imu1_W.G_y;
		msg->telemetry_data[FC_IMU1_WR_I] = Rocket_h.fcState.imu1_W.G_x;
		msg->telemetry_data[FC_IMU1_WY_I] = Rocket_h.fcState.imu1_W.G_z;
		msg->telemetry_data[FC_IMU2_X_I] = Rocket_h.fcState.imu2_A.XL_x;
		msg->telemetry_data[FC_IMU2_Y_I] = Rocket_h.fcState.imu2_A.XL_y;
		msg->telemetry_data[FC_IMU2_Z_I] = Rocket_h.fcState.imu2_A.XL_z;
		msg->telemetry_data[FC_IMU2_WP_I] = Rocket_h.fcState.imu2_W.G_y;
		msg->telemetry_data[FC_IMU2_WR_I] = Rocket_h.fcState.imu2_W.G_x;
		msg->telemetry_data[FC_IMU2_WY_I] = Rocket_h.fcState.imu2_W.G_z;
		msg->telemetry_data[FC_GPS_LAT_I] = Rocket_h.fcState.gps_lat;
		msg->telemetry_data[FC_GPS_LONG_I] = Rocket_h.fcState.gps_long;
		msg->telemetry_data[FC_GPS_ALT_I] = Rocket_h.fcState.gps_alt;
		msg->telemetry_data[FC_BAR_1_I] = Rocket_h.fcState.bar1;
		msg->telemetry_data[FC_BAR_2_I] = Rocket_h.fcState.bar2;
		msg->telemetry_data[FC_24V_VOLTAGE_I] = Rocket_h.fcState.bus24v_voltage;
		msg->telemetry_data[FC_24V_CURRENT_I] = Rocket_h.fcState.bus24v_current;
		msg->telemetry_data[FC_12V_VOLTAGE_I] = Rocket_h.fcState.bus12v_voltage;
		msg->telemetry_data[FC_12V_CURRENT_I] = Rocket_h.fcState.bus12v_current;
		msg->telemetry_data[FC_5V_VOLTAGE_I] = Rocket_h.fcState.bus5v_voltage;
		msg->telemetry_data[FC_5V_CURRENT_I] = Rocket_h.fcState.bus5v_current;
		msg->telemetry_data[FC_3V3_VOLTAGE_I] = Rocket_h.fcState.bus3v3_voltage;
		msg->telemetry_data[FC_3V3_CURRENT_I] = Rocket_h.fcState.bus3v3_current;
		xSemaphoreGive(Rocket_h.fcState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Unpack bay board message
// returns 0 if successful, 1 if the telemetry data could not be accessed or an invalid board id
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int unpack_bb_telemetry(TelemetryMessage *msg, uint8_t timeout_ticks) {
	switch(msg->board_id) {
	    case BOARD_BAY_1:
	    	if(xSemaphoreTake(Rocket_h.bb1State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb1State.vlv1_current = msg->telemetry_data[BB1_VLV1_CURRENT_I];
	    		Rocket_h.bb1State.vlv1_old = msg->telemetry_data[BB1_VLV1_OLD_I];
	    		Rocket_h.bb1State.vlv2_current = msg->telemetry_data[BB1_VLV2_CURRENT_I];
	    		Rocket_h.bb1State.vlv2_old = msg->telemetry_data[BB1_VLV2_OLD_I];
	    		Rocket_h.bb1State.vlv3_current = msg->telemetry_data[BB1_VLV3_CURRENT_I];
	    		Rocket_h.bb1State.vlv3_old = msg->telemetry_data[BB1_VLV3_OLD_I];
	    		Rocket_h.bb1State.vlv4_current = msg->telemetry_data[BB1_VLV4_CURRENT_I];
	    		Rocket_h.bb1State.vlv4_old = msg->telemetry_data[BB1_VLV4_OLD_I];
	    		Rocket_h.bb1State.vlv5_current = msg->telemetry_data[BB1_VLV5_CURRENT_I];
	    		Rocket_h.bb1State.vlv5_old = msg->telemetry_data[BB1_VLV5_OLD_I];
	    		Rocket_h.bb1State.vlv6_current = msg->telemetry_data[BB1_VLV6_CURRENT_I];
	    		Rocket_h.bb1State.vlv6_old = msg->telemetry_data[BB1_VLV6_OLD_I];
	    		Rocket_h.bb1State.vlv7_current = msg->telemetry_data[BB1_VLV7_CURRENT_I];
	    		Rocket_h.bb1State.vlv7_old = msg->telemetry_data[BB1_VLV7_OLD_I];
	    		Rocket_h.bb1State.pt1 = msg->telemetry_data[BB1_PT_1_I];
	    		Rocket_h.bb1State.pt2 = msg->telemetry_data[BB1_PT_2_I];
	    		Rocket_h.bb1State.pt3 = msg->telemetry_data[BB1_PT_3_I];
	    		Rocket_h.bb1State.pt4 = msg->telemetry_data[BB1_PT_4_I];
	    		Rocket_h.bb1State.pt5 = msg->telemetry_data[BB1_PT_5_I];
	    		Rocket_h.bb1State.pt6 = msg->telemetry_data[BB1_PT_6_I];
	    		Rocket_h.bb1State.pt7 = msg->telemetry_data[BB1_PT_7_I];
	    		Rocket_h.bb1State.pt8 = msg->telemetry_data[BB1_PT_8_I];
	    		Rocket_h.bb1State.pt9 = msg->telemetry_data[BB1_PT_9_I];
	    		Rocket_h.bb1State.pt10 = msg->telemetry_data[BB1_PT_10_I];
	    		Rocket_h.bb1State.tc1 = msg->telemetry_data[BB1_TC_1_I];
	    		Rocket_h.bb1State.tc2 = msg->telemetry_data[BB1_TC_2_I];
	    		Rocket_h.bb1State.tc3 = msg->telemetry_data[BB1_TC_3_I];
	    		Rocket_h.bb1State.tc4 = msg->telemetry_data[BB1_TC_4_I];
	    		Rocket_h.bb1State.tc5 = msg->telemetry_data[BB1_TC_5_I];
	    		Rocket_h.bb1State.tc6 = msg->telemetry_data[BB1_TC_6_I];
	    		Rocket_h.bb1State.imu1_A.XL_x = msg->telemetry_data[BB1_IMU1_X_I];
	    		Rocket_h.bb1State.imu1_A.XL_y = msg->telemetry_data[BB1_IMU1_Y_I];
	    		Rocket_h.bb1State.imu1_A.XL_z = msg->telemetry_data[BB1_IMU1_Z_I];
	    		Rocket_h.bb1State.imu1_W.G_y = msg->telemetry_data[BB1_IMU1_WP_I];
	    		Rocket_h.bb1State.imu1_W.G_x = msg->telemetry_data[BB1_IMU1_WR_I];
	    		Rocket_h.bb1State.imu1_W.G_z = msg->telemetry_data[BB1_IMU1_WY_I];
	    		Rocket_h.bb1State.imu2_A.XL_x = msg->telemetry_data[BB1_IMU2_X_I];
	    		Rocket_h.bb1State.imu2_A.XL_y = msg->telemetry_data[BB1_IMU2_Y_I];
	    		Rocket_h.bb1State.imu2_A.XL_z = msg->telemetry_data[BB1_IMU2_Z_I];
	    		Rocket_h.bb1State.imu2_W.G_y = msg->telemetry_data[BB1_IMU2_WP_I];
	    		Rocket_h.bb1State.imu2_W.G_x = msg->telemetry_data[BB1_IMU2_WR_I];
	    		Rocket_h.bb1State.imu2_W.G_z = msg->telemetry_data[BB1_IMU2_WY_I];
	    		Rocket_h.bb1State.bar1 = msg->telemetry_data[BB1_BAR_1_I];
	    		Rocket_h.bb1State.bar2 = msg->telemetry_data[BB1_BAR_2_I];
	    		Rocket_h.bb1State.bus24v_voltage = msg->telemetry_data[BB1_24V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus24v_current = msg->telemetry_data[BB1_24V_CURRENT_I];
	    		Rocket_h.bb1State.bus12v_voltage = msg->telemetry_data[BB1_12V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus12v_current = msg->telemetry_data[BB1_12V_CURRENT_I];
	    		Rocket_h.bb1State.bus5v_voltage = msg->telemetry_data[BB1_5V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus5v_current = msg->telemetry_data[BB1_5V_CURRENT_I];
	    		Rocket_h.bb1State.bus3v3_voltage = msg->telemetry_data[BB1_3V3_VOLTAGE_I];
	    		Rocket_h.bb1State.bus3v3_current = msg->telemetry_data[BB1_3V3_CURRENT_I];
	    		Rocket_h.bb1State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb1State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    case BOARD_BAY_2:
	    	if(xSemaphoreTake(Rocket_h.bb2State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb2State.vlv1_current = msg->telemetry_data[BB2_VLV1_CURRENT_I];
	    		Rocket_h.bb2State.vlv1_old = msg->telemetry_data[BB2_VLV1_OLD_I];
	    		Rocket_h.bb2State.vlv2_current = msg->telemetry_data[BB2_VLV2_CURRENT_I];
	    		Rocket_h.bb2State.vlv2_old = msg->telemetry_data[BB2_VLV2_OLD_I];
	    		Rocket_h.bb2State.vlv3_current = msg->telemetry_data[BB2_VLV3_CURRENT_I];
	    		Rocket_h.bb2State.vlv3_old = msg->telemetry_data[BB2_VLV3_OLD_I];
	    		Rocket_h.bb2State.vlv4_current = msg->telemetry_data[BB2_VLV4_CURRENT_I];
	    		Rocket_h.bb2State.vlv4_old = msg->telemetry_data[BB2_VLV4_OLD_I];
	    		Rocket_h.bb2State.vlv5_current = msg->telemetry_data[BB2_VLV5_CURRENT_I];
	    		Rocket_h.bb2State.vlv5_old = msg->telemetry_data[BB2_VLV5_OLD_I];
	    		Rocket_h.bb2State.vlv6_current = msg->telemetry_data[BB2_VLV6_CURRENT_I];
	    		Rocket_h.bb2State.vlv6_old = msg->telemetry_data[BB2_VLV6_OLD_I];
	    		Rocket_h.bb2State.vlv7_current = msg->telemetry_data[BB2_VLV7_CURRENT_I];
	    		Rocket_h.bb2State.vlv7_old = msg->telemetry_data[BB2_VLV7_OLD_I];
	    		Rocket_h.bb2State.pt1 = msg->telemetry_data[BB2_PT_1_I];
	    		Rocket_h.bb2State.pt2 = msg->telemetry_data[BB2_PT_2_I];
	    		Rocket_h.bb2State.pt3 = msg->telemetry_data[BB2_PT_3_I];
	    		Rocket_h.bb2State.pt4 = msg->telemetry_data[BB2_PT_4_I];
	    		Rocket_h.bb2State.pt5 = msg->telemetry_data[BB2_PT_5_I];
	    		Rocket_h.bb2State.pt6 = msg->telemetry_data[BB2_PT_6_I];
	    		Rocket_h.bb2State.pt7 = msg->telemetry_data[BB2_PT_7_I];
	    		Rocket_h.bb2State.pt8 = msg->telemetry_data[BB2_PT_8_I];
	    		Rocket_h.bb2State.pt9 = msg->telemetry_data[BB2_PT_9_I];
	    		Rocket_h.bb2State.pt10 = msg->telemetry_data[BB2_PT_10_I];
	    		Rocket_h.bb2State.tc1 = msg->telemetry_data[BB2_TC_1_I];
	    		Rocket_h.bb2State.tc2 = msg->telemetry_data[BB2_TC_2_I];
	    		Rocket_h.bb2State.tc3 = msg->telemetry_data[BB2_TC_3_I];
	    		Rocket_h.bb2State.tc4 = msg->telemetry_data[BB2_TC_4_I];
	    		Rocket_h.bb2State.tc5 = msg->telemetry_data[BB2_TC_5_I];
	    		Rocket_h.bb2State.tc6 = msg->telemetry_data[BB2_TC_6_I];
	    		Rocket_h.bb2State.imu1_A.XL_x = msg->telemetry_data[BB2_IMU1_X_I];
	    		Rocket_h.bb2State.imu1_A.XL_y = msg->telemetry_data[BB2_IMU1_Y_I];
	    		Rocket_h.bb2State.imu1_A.XL_z = msg->telemetry_data[BB2_IMU1_Z_I];
	    		Rocket_h.bb2State.imu1_W.G_y = msg->telemetry_data[BB2_IMU1_WP_I];
	    		Rocket_h.bb2State.imu1_W.G_x = msg->telemetry_data[BB2_IMU1_WR_I];
	    		Rocket_h.bb2State.imu1_W.G_z = msg->telemetry_data[BB2_IMU1_WY_I];
	    		Rocket_h.bb2State.imu2_A.XL_x = msg->telemetry_data[BB2_IMU2_X_I];
	    		Rocket_h.bb2State.imu2_A.XL_y = msg->telemetry_data[BB2_IMU2_Y_I];
	    		Rocket_h.bb2State.imu2_A.XL_z = msg->telemetry_data[BB2_IMU2_Z_I];
	    		Rocket_h.bb2State.imu2_W.G_y = msg->telemetry_data[BB2_IMU2_WP_I];
	    		Rocket_h.bb2State.imu2_W.G_x = msg->telemetry_data[BB2_IMU2_WR_I];
	    		Rocket_h.bb2State.imu2_W.G_z = msg->telemetry_data[BB2_IMU2_WY_I];
	    		Rocket_h.bb2State.bar1 = msg->telemetry_data[BB2_BAR_1_I];
	    		Rocket_h.bb2State.bar2 = msg->telemetry_data[BB2_BAR_2_I];
	    		Rocket_h.bb2State.bus24v_voltage = msg->telemetry_data[BB2_24V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus24v_current = msg->telemetry_data[BB2_24V_CURRENT_I];
	    		Rocket_h.bb2State.bus12v_voltage = msg->telemetry_data[BB2_12V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus12v_current = msg->telemetry_data[BB2_12V_CURRENT_I];
	    		Rocket_h.bb2State.bus5v_voltage = msg->telemetry_data[BB2_5V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus5v_current = msg->telemetry_data[BB2_5V_CURRENT_I];
	    		Rocket_h.bb2State.bus3v3_voltage = msg->telemetry_data[BB2_3V3_VOLTAGE_I];
	    		Rocket_h.bb2State.bus3v3_current = msg->telemetry_data[BB2_3V3_CURRENT_I];
	    		Rocket_h.bb2State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb2State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    case BOARD_BAY_3:
	    	if(xSemaphoreTake(Rocket_h.bb3State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb3State.vlv1_current = msg->telemetry_data[BB3_VLV1_CURRENT_I];
	    		Rocket_h.bb3State.vlv1_old = msg->telemetry_data[BB3_VLV1_OLD_I];
	    		Rocket_h.bb3State.vlv2_current = msg->telemetry_data[BB3_VLV2_CURRENT_I];
	    		Rocket_h.bb3State.vlv2_old = msg->telemetry_data[BB3_VLV2_OLD_I];
	    		Rocket_h.bb3State.vlv3_current = msg->telemetry_data[BB3_VLV3_CURRENT_I];
	    		Rocket_h.bb3State.vlv3_old = msg->telemetry_data[BB3_VLV3_OLD_I];
	    		Rocket_h.bb3State.vlv4_current = msg->telemetry_data[BB3_VLV4_CURRENT_I];
	    		Rocket_h.bb3State.vlv4_old = msg->telemetry_data[BB3_VLV4_OLD_I];
	    		Rocket_h.bb3State.vlv5_current = msg->telemetry_data[BB3_VLV5_CURRENT_I];
	    		Rocket_h.bb3State.vlv5_old = msg->telemetry_data[BB3_VLV5_OLD_I];
	    		Rocket_h.bb3State.vlv6_current = msg->telemetry_data[BB3_VLV6_CURRENT_I];
	    		Rocket_h.bb3State.vlv6_old = msg->telemetry_data[BB3_VLV6_OLD_I];
	    		Rocket_h.bb3State.vlv7_current = msg->telemetry_data[BB3_VLV7_CURRENT_I];
	    		Rocket_h.bb3State.vlv7_old = msg->telemetry_data[BB3_VLV7_OLD_I];
	    		Rocket_h.bb3State.pt1 = msg->telemetry_data[BB3_PT_1_I];
	    		Rocket_h.bb3State.pt2 = msg->telemetry_data[BB3_PT_2_I];
	    		Rocket_h.bb3State.pt3 = msg->telemetry_data[BB3_PT_3_I];
	    		Rocket_h.bb3State.pt4 = msg->telemetry_data[BB3_PT_4_I];
	    		Rocket_h.bb3State.pt5 = msg->telemetry_data[BB3_PT_5_I];
	    		Rocket_h.bb3State.pt6 = msg->telemetry_data[BB3_PT_6_I];
	    		Rocket_h.bb3State.pt7 = msg->telemetry_data[BB3_PT_7_I];
	    		Rocket_h.bb3State.pt8 = msg->telemetry_data[BB3_PT_8_I];
	    		Rocket_h.bb3State.pt9 = msg->telemetry_data[BB3_PT_9_I];
	    		Rocket_h.bb3State.pt10 = msg->telemetry_data[BB3_PT_10_I];
	    		Rocket_h.bb3State.tc1 = msg->telemetry_data[BB3_TC_1_I];
	    		Rocket_h.bb3State.tc2 = msg->telemetry_data[BB3_TC_2_I];
	    		Rocket_h.bb3State.tc3 = msg->telemetry_data[BB3_TC_3_I];
	    		Rocket_h.bb3State.tc4 = msg->telemetry_data[BB3_TC_4_I];
	    		Rocket_h.bb3State.tc5 = msg->telemetry_data[BB3_TC_5_I];
	    		Rocket_h.bb3State.tc6 = msg->telemetry_data[BB3_TC_6_I];
	    		Rocket_h.bb3State.imu1_A.XL_x = msg->telemetry_data[BB3_IMU1_X_I];
	    		Rocket_h.bb3State.imu1_A.XL_y = msg->telemetry_data[BB3_IMU1_Y_I];
	    		Rocket_h.bb3State.imu1_A.XL_z = msg->telemetry_data[BB3_IMU1_Z_I];
	    		Rocket_h.bb3State.imu1_W.G_y = msg->telemetry_data[BB3_IMU1_WP_I];
	    		Rocket_h.bb3State.imu1_W.G_x = msg->telemetry_data[BB3_IMU1_WR_I];
	    		Rocket_h.bb3State.imu1_W.G_z = msg->telemetry_data[BB3_IMU1_WY_I];
	    		Rocket_h.bb3State.imu2_A.XL_x = msg->telemetry_data[BB3_IMU2_X_I];
	    		Rocket_h.bb3State.imu2_A.XL_y = msg->telemetry_data[BB3_IMU2_Y_I];
	    		Rocket_h.bb3State.imu2_A.XL_z = msg->telemetry_data[BB3_IMU2_Z_I];
	    		Rocket_h.bb3State.imu2_W.G_y = msg->telemetry_data[BB3_IMU2_WP_I];
	    		Rocket_h.bb3State.imu2_W.G_x = msg->telemetry_data[BB3_IMU2_WR_I];
	    		Rocket_h.bb3State.imu2_W.G_z = msg->telemetry_data[BB3_IMU2_WY_I];
	    		Rocket_h.bb3State.bar1 = msg->telemetry_data[BB3_BAR_1_I];
	    		Rocket_h.bb3State.bar2 = msg->telemetry_data[BB3_BAR_2_I];
	    		Rocket_h.bb3State.bus24v_voltage = msg->telemetry_data[BB3_24V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus24v_current = msg->telemetry_data[BB3_24V_CURRENT_I];
	    		Rocket_h.bb3State.bus12v_voltage = msg->telemetry_data[BB3_12V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus12v_current = msg->telemetry_data[BB3_12V_CURRENT_I];
	    		Rocket_h.bb3State.bus5v_voltage = msg->telemetry_data[BB3_5V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus5v_current = msg->telemetry_data[BB3_5V_CURRENT_I];
	    		Rocket_h.bb3State.bus3v3_voltage = msg->telemetry_data[BB3_3V3_VOLTAGE_I];
	    		Rocket_h.bb3State.bus3v3_current = msg->telemetry_data[BB3_3V3_CURRENT_I];
	    		Rocket_h.bb3State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb3State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    default:
	    	return 1;
	        break;
	}

	return 0;
}

// Unpack flight recorder telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int unpack_fr_telemetry(TelemetryMessage *msg, uint8_t timeout_ticks) {
	if(xSemaphoreTake(Rocket_h.frState_access, timeout_ticks) == pdPASS) {
		Rocket_h.frState.imu1_A.XL_x = msg->telemetry_data[FR_IMU1_X_I];
		Rocket_h.frState.imu1_A.XL_y = msg->telemetry_data[FR_IMU1_Y_I];
		Rocket_h.frState.imu1_A.XL_z = msg->telemetry_data[FR_IMU1_Z_I];
		Rocket_h.frState.imu1_W.G_y = msg->telemetry_data[FR_IMU1_WP_I];
		Rocket_h.frState.imu1_W.G_x = msg->telemetry_data[FR_IMU1_WR_I];
		Rocket_h.frState.imu1_W.G_z = msg->telemetry_data[FR_IMU1_WY_I];
		Rocket_h.frState.imu2_A.XL_x = msg->telemetry_data[FR_IMU2_X_I];
		Rocket_h.frState.imu2_A.XL_y = msg->telemetry_data[FR_IMU2_Y_I];
		Rocket_h.frState.imu2_A.XL_z = msg->telemetry_data[FR_IMU2_Z_I];
		Rocket_h.frState.imu2_W.G_y = msg->telemetry_data[FR_IMU2_WP_I];
		Rocket_h.frState.imu2_W.G_x = msg->telemetry_data[FR_IMU2_WR_I];
		Rocket_h.frState.imu2_W.G_z = msg->telemetry_data[FR_IMU2_WY_I];
		Rocket_h.frState.bar1 = msg->telemetry_data[FR_BAR_1_I];
		Rocket_h.frState.bar2 = msg->telemetry_data[FR_BAR_2_I];
		Rocket_h.frState.timestamp = msg->timestamp;
		xSemaphoreGive(Rocket_h.frState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Send a LMP message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// buffersize is the maximum size that it will take to serialize the LMP message, if this is 0, it will use the maximum possible message size to ensure proper serialization
// returns 0 on success, -1 if the server is not up, -2 if there is no room in the txbuffer or space to allocate a buffer, -3 if the target device is not connected, and -4 on a LMP serialization error
int send_msg_to_device(Target_Device device, Message *msg, TickType_t wait, size_t buffersize) {
	if(buffersize == 0) {
		buffersize = MAX_MSG_LEN;
	}
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int devicefd = get_device_fd(device);
	if(devicefd < 0) {
		return -3; // Device not connected
	}
	uint8_t tempbuffer[buffersize];
	int buflen = serialize_message(msg, tempbuffer, buffersize);
	if(buflen == -1) {
		return -4; // Serialization error
	}
    uint8_t *buffer = malloc(buflen);
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	Raw_message rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;
    	rawmsg.connection_fd = devicefd;
    	int result = server_send(&rawmsg, wait);
    	if(result != 0) {
    		free(buffer);
    	}
    	return result;
    }
    return -2;
}

// Send a message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// returns 0 on success, -1 if the server is not up, -2 if there is no room in the txbuffer, -3 if the target device is not connected
// IMPORTANT: you are responsible for freeing the buffer in msg if this function returns something other than 0
int send_raw_msg_to_device(Target_Device device, Raw_message *msg, TickType_t wait) {
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int devicefd = get_device_fd(device);
	if(devicefd < 0) {
		return -3; // Device not connected
	}
	return server_send(msg, wait);
}

// Gets the valve index from the LMP valve id
// Be careful since the integer value of the Valve_Channel enum starts at 0
// Returns -1 on an invalid id
Valve_Channel get_valve(uint8_t valveId) {
	return (valveId % 10) - 1;
}

// Gets the board from a LMP valve id
// This will not check for an invalid id, call check_valve_id first
BoardId get_valve_board(uint8_t valveId) {
	return (BoardId) ((uint8_t)(valveId / 10));
}

// Generate the LMP valve id from the board and valve index
uint8_t generate_valve_id(BoardId board, Valve_Channel valve) {
	return (((uint8_t) board) * 10) + valve + 1;
}

// Check if a LMP valve id is valid. Returns 1 if the id is valid, 0 if it's not
uint8_t check_valve_id(uint8_t valveId) {
	if(valveId < 01 || valveId > 37) {
		return 0;
	}
	if(get_valve_board(valveId) == BOARD_FC) {
		if(get_valve(valveId) > 2) {
			return 0;
		}
	}
	else {
		if(get_valve(valveId) == -1 || get_valve(valveId) > 6) {
			return 0;
		}
	}
	return 1;
}

// Set a valve to a state and update its corresponding valve state
// Returns the state of the valve after setting it (this should be equal to desiredState, but in the case of an error, it will accurately reflect the current state of the valve)
Valve_State_t set_and_update_valve(Valve_Channel valve, Valve_State_t desiredState) {
	uint8_t vlverror = 0;
	 if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
		 if(desiredState == Valve_Energized) {
			 VLV_En(Rocket_h.fcValves[valve]);
			 Rocket_h.fcValveStates[valve] = Valve_Energized;
		 }
		 else if(desiredState == Valve_Deenergized) {
			 VLV_Den(Rocket_h.fcValves[valve]);
			 Rocket_h.fcValveStates[valve] = Valve_Deenergized;
		 }
		 else {
			 // Error invalid state
			 vlverror = 1;
		 }
		 xSemaphoreGive(Rocket_h.fcValve_access);
	 }
	 else {
		 vlverror = 1;
	 }
	 if(!vlverror) {
		 return desiredState;
	 }
	 else {
		 return HAL_GPIO_ReadPin(Rocket_h.fcValves[valve].VLV_EN_GPIO_Port, Rocket_h.fcValves[valve].VLV_EN_GPIO_Pin);
	 }
}

// Load board configuration from a buffer. Returns 0 on success, -1 on an eeprom error, -2 on invalid tc gains, and -3 on invalid valve configuration values
int load_eeprom_config(eeprom_t *eeprom, EEPROM_conf_t *conf) {
	uint8_t buffer[FC_EEPROM_LEN];
	eeprom_status_t read_stat = eeprom_read_mem(eeprom, 0, buffer, FC_EEPROM_LEN);
	if(read_stat != EEPROM_OK) {
		return -1;
	}
	memcpy(&(conf->pt1->zero_V), buffer, 4);
	memcpy(&(conf->pt1->pres_range), buffer + 4, 4);
	memcpy(&(conf->pt1->max_V), buffer + 8, 4);
	memcpy(&(conf->pt2->zero_V), buffer + 12, 4);
	memcpy(&(conf->pt2->pres_range), buffer + 16, 4);
	memcpy(&(conf->pt2->max_V), buffer + 20, 4);
	memcpy(&(conf->pt3->zero_V), buffer + 24, 4);
	memcpy(&(conf->pt3->pres_range), buffer + 28, 4);
	memcpy(&(conf->pt3->max_V), buffer + 32, 4);
	memcpy(&(conf->pt4->zero_V), buffer + 36, 4);
	memcpy(&(conf->pt4->pres_range), buffer + 40, 4);
	memcpy(&(conf->pt4->max_V), buffer + 44, 4);
	memcpy(&(conf->pt5->zero_V), buffer + 48, 4);
	memcpy(&(conf->pt5->pres_range), buffer + 52, 4);
	memcpy(&(conf->pt5->max_V), buffer + 56, 4);

	conf->tc1_gain = buffer[60] << 1;
	conf->tc2_gain = buffer[61] << 1;
	conf->tc3_gain = buffer[62] << 1;

	conf->vlv1_v = buffer[63];
	conf->vlv1_en = buffer[64];
	conf->vlv2_v = buffer[65];
	conf->vlv2_en = buffer[66];
	conf->vlv3_v = buffer[67];
	conf->vlv3_en = buffer[68];

	IP4_ADDR(&(conf->limewireIP), buffer[69], buffer[70], buffer[71], buffer[72]);
	IP4_ADDR(&(conf->flightcomputerIP), buffer[73], buffer[74], buffer[75], buffer[76]);
	IP4_ADDR(&(conf->bayboard1IP), buffer[77], buffer[78], buffer[79], buffer[80]);
	IP4_ADDR(&(conf->bayboard2IP), buffer[81], buffer[82], buffer[83], buffer[84]);
	IP4_ADDR(&(conf->bayboard3IP), buffer[85], buffer[86], buffer[87], buffer[88]);
	IP4_ADDR(&(conf->flightrecordIP), buffer[89], buffer[90], buffer[91], buffer[92]);

	if(conf->tc1_gain > 0x0E || conf->tc2_gain > 0x0E || conf->tc3_gain > 0x0E) {
		return -2;
	}

	if(conf->vlv1_v > 0x01 || conf->vlv1_en > 0x01 || conf->vlv2_v > 0x01 || conf->vlv2_en > 0x01 || conf->vlv3_v > 0x01 || conf->vlv3_en > 0x01) {
		return -3;
	}
	return 0;
}

void load_eeprom_defaults(EEPROM_conf_t *conf) {
	conf->pt1->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt1->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt1->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt2->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt2->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt2->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt3->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt3->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt3->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt4->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt4->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt4->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt5->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt5->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt5->max_V = FC_EEPROM_PT_MAX_DEFAULT;

	conf->tc1_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	conf->tc2_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	conf->tc3_gain = FC_EEPROM_TC_GAIN_DEFAULT;

	conf->vlv1_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv1_en = FC_EEPROM_VLV_EN_DEFAULT;
	conf->vlv2_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv2_en = FC_EEPROM_VLV_EN_DEFAULT;
	conf->vlv3_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv3_en = FC_EEPROM_VLV_EN_DEFAULT;

	IP4_ADDR(&(conf->limewireIP), FC_EEPROM_LIMEWIREIP_DEFAULT_1, FC_EEPROM_LIMEWIREIP_DEFAULT_2, FC_EEPROM_LIMEWIREIP_DEFAULT_3, FC_EEPROM_LIMEWIREIP_DEFAULT_4);
	IP4_ADDR(&(conf->flightcomputerIP), FC_EEPROM_FCIP_DEFAULT_1, FC_EEPROM_FCIP_DEFAULT_2, FC_EEPROM_FCIP_DEFAULT_3, FC_EEPROM_FCIP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard1IP), FC_EEPROM_BB1IP_DEFAULT_1, FC_EEPROM_BB1IP_DEFAULT_2, FC_EEPROM_BB1IP_DEFAULT_3, FC_EEPROM_BB1IP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard2IP), FC_EEPROM_BB2IP_DEFAULT_1, FC_EEPROM_BB2IP_DEFAULT_2, FC_EEPROM_BB2IP_DEFAULT_3, FC_EEPROM_BB2IP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard3IP), FC_EEPROM_BB3IP_DEFAULT_1, FC_EEPROM_BB3IP_DEFAULT_2, FC_EEPROM_BB3IP_DEFAULT_3, FC_EEPROM_BB3IP_DEFAULT_4);
	IP4_ADDR(&(conf->flightrecordIP), FC_EEPROM_FRIP_DEFAULT_1, FC_EEPROM_FRIP_DEFAULT_2, FC_EEPROM_FRIP_DEFAULT_3, FC_EEPROM_FRIP_DEFAULT_4);
}

// Writes text to flash, msgtext does not have to be null terminated and msglen should not include the null character if it is included
// type is the msg type, use the macros in main.h
// returns 0 on success, 1 if there is not enough space or the flash could not be accessed
uint8_t write_ascii_to_flash(const char *msgtext, size_t msglen, uint8_t type) {
	if(xSemaphoreTake(flash_mutex, 1) == pdPASS) {
		if(fc_get_bytes_remaining(&flash_h) < msglen + 2) {
			xSemaphoreGive(flash_mutex);
			return 1;
		}
		uint8_t *writebuf = (uint8_t *) malloc(msglen + 2);
		writebuf[0] = type;
		memcpy(&writebuf[1], msgtext, msglen);
		writebuf[msglen + 1] = '\n';
		fc_write_to_flash(&flash_h, writebuf, msglen + 2);
		free(writebuf);
		xSemaphoreGive(flash_mutex);
		return 0;
	}
	return 1;
}

// Returns 0 on success, 1 on access error, 2 if the flash is full
uint8_t write_raw_to_flash(uint8_t *writebuf, size_t msglen) {
	if(xSemaphoreTake(flash_mutex, 1) == pdPASS) {
		if(fc_get_bytes_remaining(&flash_h) < msglen) {
			xSemaphoreGive(flash_mutex);
			return 2;
		}
		fc_write_to_flash(&flash_h, writebuf, msglen);
		xSemaphoreGive(flash_mutex);
		return 0;
	}
	return 1;
}

/*
 * Log message to flash. This is the function to use throughout the rest of the code.
 * msgtext - Message text. This should include an error code if the message is an error
 * msgtype - Type of message, used for error message throttling. Pass -1 if this is a status message or if you don't want it to be throttled, otherwise pass the "category" of the error message
 * Returns 0 on success, 1 if a message of this type was sent too recently, 2 on general error, and 3 if there isn't enough memory available
 */
uint8_t log_message(const char *msgtext, int msgtype) {
	if(msgtype != -1) {
		if(msgtype >= ERROR_MSG_TYPES) {
			return 2;
		}
		if(xSemaphoreTake(errormsg_mutex, 2) == pdPASS) {
			uint8_t val = (msgtype % 2) == 0 ? errormsgtimers[msgtype / 2] & 0x0F : errormsgtimers[msgtype / 2] >> 4;
			if(val < ERROR_THROTTLE_MAX) {
				val++;
				if(msgtype % 2) {
					errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0x0F) | (val << 4);
				}
				else {
					errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0xF0) | val;
				}
			}
			else {
				xSemaphoreGive(errormsg_mutex);
				return 1;
			}
			/*if(val) {
				xSemaphoreGive(errormsg_mutex);
				return 1;
			}
			if(msgtype % 2) {
				errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0x0F) | (15 << 4);
			}
			else {
				errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0xF0) | 15;
			}*/
			xSemaphoreGive(errormsg_mutex);
		}
		else {
			return 2;
		}
	}
	// Flash entry type + timestamp + space + error code board number + message text + newline
	size_t msglen = 1 + 24 + 1 + 1 + strlen(msgtext) + 1;
	uint8_t *rawmsgbuf = (uint8_t *) malloc(msglen);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1]);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '1';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(list_push(errorMsgList, (void *)&fullmsg, 1)) {
			// No space for more messages
			free(rawmsgbuf);
			return 3;
		}
		return 0;
	}
	return 3;
}

// Same as log_message but intended for use with peripheral device errors
// Only difference is the message type timers last a lot longer ~ 5 seconds
// This is intended to reduce message spam in the case that we intentionally run boards with disconnected peripherals
uint8_t log_peri_message(const char *msgtext, int msgtype) {
	if(msgtype != -1) {
		if(msgtype >= PERI_ERROR_MSG_TYPES) {
			return 2;
		}
		if(xSemaphoreTake(perierrormsg_mutex, 2) == pdPASS) {
			if(perierrormsgtimers[msgtype]) {
				xSemaphoreGive(perierrormsg_mutex);
				return 1;
			}
			perierrormsgtimers[msgtype] = 20; // 20 * 250 = ~5000ms
			xSemaphoreGive(perierrormsg_mutex);
		}
		else {
			return 2;
		}
	}
	// Flash entry type + timestamp + space + error code board number + message text + newline
	size_t msglen = 1 + 24 + 1 + 1 + strlen(msgtext) + 1;
	uint8_t *rawmsgbuf = (uint8_t *) malloc(msglen);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1]);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '1';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(list_push(errorMsgList, (void *)&fullmsg, 1)) {
			// No space for more messages
			free(rawmsgbuf);
			return 3;
		}
		return 0;
	}
	return 3;
}

void send_flash_full() {
	if(xSemaphoreTake(errorudp_mutex, 5) == pdPASS) {
		if(errormsgudp) {
			struct netbuf *outbuf = netbuf_new();
			if(outbuf) {
				char *pkt_buf = (char *) netbuf_alloc(outbuf, 24 + 1 + 1 + sizeof(ERR_FLASH_FULL) - 1);
				if(pkt_buf) {
					get_iso_time(pkt_buf);
					pkt_buf[24] = ' ';
					pkt_buf[25] = '1';
					memcpy(&pkt_buf[26], ERR_FLASH_FULL, sizeof(ERR_FLASH_FULL) - 1);
					netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
				}
				netbuf_delete(outbuf);
			}
		}
		xSemaphoreGive(errorudp_mutex);
	}
}

// 0 success, 1 semaphore timeout, 2 flash full, 3 memory error
int log_lmp_packet(uint8_t *buf, size_t buflen) {
	size_t outlen;
	uint8_t *encoded = base64_encode(buf, buflen, &outlen, 1);
	if(encoded) {
		encoded[0] = FLASH_TELEM_MARK;
		encoded[outlen] = '\n';
		uint8_t stat = write_raw_to_flash(encoded, outlen + 1);
		free(encoded);
		return stat;
	}
	return 3;
}

void send_udp_online(ip4_addr_t * ip) {
	size_t msglen = 24 + 1 + 1 + sizeof(STAT_NETWORK_LOG_ONLINE) + 15;
	struct netbuf *outbuf = netbuf_new();
	if(outbuf) {
		uint8_t *pkt_buf = (uint8_t *) netbuf_alloc(outbuf, msglen);
		if(pkt_buf) {
			get_iso_time((char *) pkt_buf);
			pkt_buf[24] = ' ';
			pkt_buf[25] = '1';
			snprintf((char *) &pkt_buf[26], sizeof(STAT_NETWORK_LOG_ONLINE) + 15, STAT_NETWORK_LOG_ONLINE "%u.%u.%u.%u", ip4_addr1(ip), ip4_addr2(ip), ip4_addr3(ip), ip4_addr4(ip));
			netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
		}
		netbuf_delete(outbuf);
	}
}

void *ftp_open(const char *fname, const char *mode, u8_t write) {
	if(strcmp(fname, "eeprom.bin") == 0) {
		eeprom_cursor = 0;
		return (void *) 1; // 1 refers to eeprom
	}
	else if(strcmp(fname, "messages.txt") == 0) {
		if(!write) {
			xSemaphoreTake(flash_mutex, portMAX_DELAY);
			fc_finish_flash_write(&flash_h);
			fc_reset_flash_read_pointer(&flash_h);
			validflashbytes = 0;
			flashmsgtype = 0;
		}
		return write ? (void *) 0 : (void *) FLASH_MSG_MARK; // FLASH_MSG_MARK refers to status/error messages. Only reading is valid
	}
	else if(strcmp(fname, "telemetry.bin") == 0) {
		if(!write) {
			xSemaphoreTake(flash_mutex, portMAX_DELAY);
			fc_finish_flash_write(&flash_h);
			fc_reset_flash_read_pointer(&flash_h);
			validflashbytes = 0;
			flashmsgtype = 0;
		}
		return write ? (void *) 0 : (void *) FLASH_TELEM_MARK; // FLASH_TELEM_MARK refers to telemetry and valve state LMP dump. Only reading is valid
	}
	else {
		return (void *) 0;
	}
}

void ftp_close(void *handle) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		eeprom_cursor -= 4;
		if(eeprom_cursor < FC_EEPROM_LEN) {
			// too short
			log_message(ERR_TFTP_EERPOM_TOO_SHORT, -1);
			return;
		}
		// CRC check
		uint32_t sent_crc;
		eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor + FC_EEPROM_LEN, (uint8_t *) &sent_crc, 4);
		if(read_stat != EEPROM_OK) {
			// read error
			log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
			return;
		}
		HAL_CRC_Calculate(&hcrc, NULL, 0); // reset calculation
		uint32_t calc_crc;
		int cursor = 0;
		do {
			int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
			uint8_t buf[read_bytes];
			read_stat = eeprom_read_mem(&eeprom_h, FC_EEPROM_LEN + cursor, buf, read_bytes);
			if(read_stat != EEPROM_OK) {
				// read error
				log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
				return;
			}
			calc_crc = HAL_CRC_Accumulate(&hcrc, (uint32_t *) buf, read_bytes);
			cursor += read_bytes;
		} while(eeprom_cursor - cursor > 0);

		calc_crc ^= 0xFFFFFFFF; // Invert bits to follow crc32 standard

		if(sent_crc == calc_crc) {
			// Matches, copy contents into active section of eeprom
			cursor = 0;
			do {
				int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
				uint8_t buf[read_bytes];
				read_stat = eeprom_read_mem(&eeprom_h, FC_EEPROM_LEN + cursor, buf, read_bytes);
				if(read_stat != EEPROM_OK) {
					// copy read error
					log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
					return;
				}
				eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, cursor, buf, read_bytes);
				if(write_stat != EEPROM_OK) {
					// copy write error
					log_message(ERR_TFTP_EEPROM_WRITE_ERR, FC_ERR_TYPE_TFTP_EEPROM_WRITE);
					return;
				}
				cursor += read_bytes;
			} while(eeprom_cursor - cursor > 0);
			// eeprom successfully updated! restart board for new config to take effect
			log_message(STAT_EEPROM_CONFIG_CHANGED, -1);
		}
		else {
			// Mismatched CRC
			log_message(ERR_TFTP_EEPROM_BAD_CRC, -1);
		}
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		xSemaphoreGive(flash_mutex);
	}
}

int ftp_read(void *handle, void *buf, int bytes) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		// Read from EEPROM
		int bytes_to_read = (eeprom_cursor + bytes > FC_EEPROM_LEN) ? FC_EEPROM_LEN - eeprom_cursor : bytes;
		if(bytes_to_read == 0) {
			return 0;
		}
		eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor, buf, bytes_to_read);
		if(read_stat != EEPROM_OK) {
			// eeprom read error
			log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
			return -1;
		}
		eeprom_cursor += bytes_to_read;
		return bytes_to_read;
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		uint8_t *flashbuf = (uint8_t *) malloc(2048);
		if(!flashbuf) {
			return -1;
		}
		while(bytes > validflashbytes) {
			if(fc_flash_current_page(&flash_h) >= 4 * W25N01GV_NUM_PAGES) {
				break;
			}
			uint16_t cursor = 0;
			uint8_t empty = 1;
			fc_read_next_2KB_from_flash(&flash_h, flashbuf);
			if(flashbuf[0] != FLASH_MSG_MARK && flashbuf[0] != FLASH_TELEM_MARK) {
				// Load or "discard" partial message, be careful if the entire 2048 bytes are part of the partial message
				for(cursor = 0;cursor < 2048;cursor++) {
					if(flashbuf[cursor] != 0xFF) {
						empty = 0;
					}
					if(flashbuf[cursor] == '\n') {
						break;
					}
				}
				cursor += 1;
				if(cursor > 2047) {
					if(empty) {
						break;
					}
					// Load entire or none, then continue
					if(fd == flashmsgtype) {
						memcpy(&flashreadbuffer[validflashbytes], flashbuf, 2048);
						validflashbytes += 2048;
					}
					continue;
				}
				else {
					if(fd == flashmsgtype) {
						memcpy(&flashreadbuffer[validflashbytes], flashbuf, cursor);
						validflashbytes += cursor;
					}
				}
			}
			int start = -1;
			for(;cursor < 2048;cursor++) {
				if(flashbuf[cursor] == FLASH_MSG_MARK || flashbuf[cursor] == FLASH_TELEM_MARK) {
					flashmsgtype = flashbuf[cursor];
					start = cursor + 1;
				}
				else if(flashbuf[cursor] == '\n') {
					if(fd == flashmsgtype) {
						if(start != -1) {
							memcpy(&flashreadbuffer[validflashbytes], &flashbuf[start], (cursor - start) + 1);
							validflashbytes += (cursor - start) + 1;
						}
					}
					start = -1;
				}
			}
			if(start != -1) {
				if(fd == flashmsgtype) {
					memcpy(&flashreadbuffer[validflashbytes], &flashbuf[start], (2047 - start) + 1);
					validflashbytes += (2047 - start) + 1;
				}
			}
		}
		// Load validflashbytes into buf, keep in mind it could be less than 512 or even 0
		int readbytes = (bytes > validflashbytes) ? validflashbytes : bytes;
		memcpy(buf, flashreadbuffer, readbytes);
		memmove(&flashreadbuffer, &flashreadbuffer[readbytes], validflashbytes - readbytes);
		validflashbytes -= readbytes;
		free(flashbuf);
		return readbytes;
	}
	else {
		return -1;
	}
}

int ftp_write(void *handle, struct pbuf *p) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		// EEPROM
		struct pbuf *currbuf = p;
		do {
			eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, eeprom_cursor + FC_EEPROM_LEN, currbuf->payload, currbuf->len);
			if(write_stat != EEPROM_OK) {
				// eeprom write error
				log_message(ERR_TFTP_EEPROM_WRITE_ERR, FC_ERR_TYPE_TFTP_EEPROM_WRITE);
				return -1;
			}
			eeprom_cursor += currbuf->len;
			currbuf = currbuf->next;
		} while (currbuf != NULL);
		return p->tot_len;
	}
	else {
		return -1;
	}
}

const struct tftp_context my_tftp_ctx = {
    .open = ftp_open,
    .close = ftp_close,
    .read = ftp_read,
    .write = ftp_write
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // RTC init code - enable backup domain access because we're using HSE / 60 as RTC clock source
  HAL_PWR_EnableBkUpAccess();
  // Force reset RTC registers in case something got corrupted
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C5_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART10_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();
  MX_CRC_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  // Create RTC mutex
  RTC_mutex = xSemaphoreCreateMutex();
  flash_mutex = xSemaphoreCreateMutex();
  flash_clear_mutex = xSemaphoreCreateMutex();
  errormsg_mutex = xSemaphoreCreateMutex();
  perierrormsg_mutex = xSemaphoreCreateMutex();
  errorudp_mutex = xSemaphoreCreateMutex();

  xSemaphoreTake(flash_clear_mutex, 0); // Make sure the mutex is taken

  memset(errormsgtimers, 0, ERROR_MSG_TYPES / 2);
  memset(perierrormsgtimers, 0, PERI_ERROR_MSG_TYPES);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  errorMsgList = list_create(sizeof(errormsg_t), msgFreeCallback);

  // This next section is considered the "critical" portion of initialization. This portion has no access to logging and therefore needs to communicate status other ways. The section ends on the return of the MX_LWIP_Init function
  // The end of this section is marked by a UDP message and/or the on-board LED becoming solid and/or a short buzzer beep

  inittimers_t timers = {0};

  // Load EEPROM config
  loaded_config.pt1 = &PT1_h;
  loaded_config.pt2 = &PT2_h;
  loaded_config.pt3 = &PT3_h;
  loaded_config.pt4 = &PT4_h;
  loaded_config.pt5 = &PT5_h;
  if(eeprom_init(&eeprom_h, &hi2c1, EEPROM_WC_GPIO_Port, EEPROM_WC_Pin) == EEPROM_OK) {
#ifdef EEPROM_OVERRIDE
	  load_eeprom_defaults(&loaded_config);
	  log_message(FC_STAT_EEPROM_DEFAULT_LOADED, -1);
#else
	  switch(load_eeprom_config(&eeprom_h, &loaded_config)) {
	  	  case -1: {
	  		  // eeprom load error, defaults loaded
	  		  log_message(FC_ERR_EEPROM_LOAD_COMM_ERR, -1);
	  		  load_eeprom_defaults(&loaded_config);
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -2: {
	  		  // eeprom tc gain value error
	  		  log_message(FC_ERR_EEPROM_LOAD_TC_ERR, -1);
	  		  loaded_config.tc1_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  loaded_config.tc2_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  loaded_config.tc3_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -3: {
	  		  // eeprom valve conf error
	  		  log_message(FC_ERR_EEPROM_LOAD_VLV_ERR, -1);
	  		  loaded_config.vlv1_en = FC_EEPROM_VLV_EN_DEFAULT;
	  		  loaded_config.vlv1_v = FC_EEPROM_VLV_VOL_DEFAULT;
	  		  loaded_config.vlv2_en = FC_EEPROM_VLV_EN_DEFAULT;
	  		  loaded_config.vlv2_v = FC_EEPROM_VLV_VOL_DEFAULT;
	  		  loaded_config.vlv3_en = FC_EEPROM_VLV_EN_DEFAULT;
	  		  loaded_config.vlv3_v = FC_EEPROM_VLV_VOL_DEFAULT;
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  default: {
	  		  // eeprom conf loaded
	  		  log_message(FC_STAT_EEPROM_LOADED, -1);
	  		  break;
	  	  }
	  }
#endif
  }
  else {
	  // eeprom init error, defaults loaded
	  log_message(FC_ERR_EEPROM_INIT, -1);
	  load_eeprom_defaults(&loaded_config);
	  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
  }
  fc_addr = loaded_config.flightcomputerIP;

  // Init flash
  fc_init_flash(&flash_h, &hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  if(!fc_ping_flash(&flash_h)) {
	  // flash not connected
	  log_message(ERR_FLASH_INIT, -1);
	  timers.ledTimer = xTimerCreate("led", 500, pdTRUE, NULL, toggleLEDPins);
  }
  memset(&errormsgtimers, 0, ERROR_MSG_TYPES / 2);
  //fc_erase_flash(&flash_h);

  if(timers.buzzTimer) {
	  xTimerStart(timers.buzzTimer, 0);
  }
  if(timers.ledTimer) {
	  xTimerStart(timers.ledTimer, 0);
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startTask */
  startTaskHandle = osThreadNew(StartAndMonitor, (void *) &timers, &startTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  flashClearTaskHandle = osThreadNew(FlashClearTask, NULL, &flash_clear_task_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B0DCFB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C5_Init(void)
{

  /* USER CODE BEGIN I2C5_Init 0 */

  /* USER CODE END I2C5_Init 0 */

  /* USER CODE BEGIN I2C5_Init 1 */

  /* USER CODE END I2C5_Init 1 */
  hi2c5.Instance = I2C5;
  hi2c5.Init.Timing = 0x10B0DCFB;
  hi2c5.Init.OwnAddress1 = 0;
  hi2c5.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c5.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c5.Init.OwnAddress2 = 0;
  hi2c5.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c5.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c5.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c5, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C5_Init 2 */

  /* USER CODE END I2C5_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 6249;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief USART10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART10_UART_Init(void)
{

  /* USER CODE BEGIN USART10_Init 0 */

  /* USER CODE END USART10_Init 0 */

  /* USER CODE BEGIN USART10_Init 1 */

  /* USER CODE END USART10_Init 1 */
  huart10.Instance = USART10;
  huart10.Init.BaudRate = 38400;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  huart10.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart10.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart10.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart10, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart10, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART10_Init 2 */

  /* USER CODE END USART10_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPS_NRST_Pin|ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_DIO3_Pin|RF_DIO2_Pin|RF_DIO1_Pin|VLV1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_CS_GPIO_Port, RADIO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_NSRT_GPIO_Port, RF_NSRT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BAR2_CS_Pin|BAR1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PDB_DIO1_Pin|PDB_DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TC1_CS_Pin|TC2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VLV3_EN_GPIO_Port, VLV3_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VLV2_EN_Pin|BUFF_CLR_Pin|BUFF_CLK_Pin|VLV_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, FLASH_CS_Pin|EEPROM_WC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZ_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPS_NRST_Pin ADC_CS_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = GPS_NRST_Pin|ADC_CS_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_DIO3_Pin RF_DIO2_Pin RF_DIO1_Pin RF_NSRT_Pin
                           VLV1_EN_Pin */
  GPIO_InitStruct.Pin = RF_DIO3_Pin|RF_DIO2_Pin|RF_DIO1_Pin|RF_NSRT_Pin
                          |VLV1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_CS_Pin PDB_DIO1_Pin PDB_DIO2_Pin */
  GPIO_InitStruct.Pin = RADIO_CS_Pin|PDB_DIO1_Pin|PDB_DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_BUSY_Pin */
  GPIO_InitStruct.Pin = RF_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_MODE_SW_Pin VLV3_OLD_Pin VLV1_OLD_Pin */
  GPIO_InitStruct.Pin = RF_MODE_SW_Pin|VLV3_OLD_Pin|VLV1_OLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BAR2_CS_Pin BAR1_CS_Pin VLV2_EN_Pin */
  GPIO_InitStruct.Pin = BAR2_CS_Pin|BAR1_CS_Pin|VLV2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ETH_NRST_Pin BUZZ_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = ETH_NRST_Pin|BUZZ_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VLV2_OLD_Pin */
  GPIO_InitStruct.Pin = VLV2_OLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VLV2_OLD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TC1_CS_Pin TC2_CS_Pin */
  GPIO_InitStruct.Pin = TC1_CS_Pin|TC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VLV3_EN_Pin FLASH_CS_Pin EEPROM_WC_Pin */
  GPIO_InitStruct.Pin = VLV3_EN_Pin|FLASH_CS_Pin|EEPROM_WC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BUFF_CLR_Pin BUFF_CLK_Pin VLV_CTRL_Pin */
  GPIO_InitStruct.Pin = BUFF_CLR_Pin|BUFF_CLK_Pin|VLV_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU1_INT1_Pin IMU1_INT2_Pin IMU2_INT1_Pin IMU2_INT2_Pin */
  GPIO_InitStruct.Pin = IMU1_INT1_Pin|IMU1_INT2_Pin|IMU2_INT1_Pin|IMU2_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TelemetryTask(void *argument) {
	// started telemetry thread
	log_message(STAT_TELEM_TASK_STARTED, -1);
	for(;;) {
		uint32_t startTime = HAL_GetTick();
		// Read from sensors
		uint16_t adc_values[16] = {0};
		read_adc(&hspi4, &(sensors_h.adc_h), adc_values);
		if(adc_values[ADC_3V3_BUS_I] == 0) {
			// ADC read error
			log_peri_message(FC_ERR_SING_ADC_READ, FC_ERR_PERI_TYPE_ADC);
		}

  		Accel XL_readings1 = {0};
  		Accel XL_readings2 = {0};
  		AngRate angRate_readings1 = {0};
  		AngRate angRate_readings2 = {0};
  		int imu1_stat = IMU_getAccel(&(sensors_h.imu1_h), &XL_readings1);
  		if(imu1_stat == -1) {
  			// IMU 1 read error
			log_peri_message(ERR_IMU_READ "1", FC_ERR_PERI_TYPE_IMU1);
  		}
  		int imu2_stat = IMU_getAccel(&(sensors_h.imu2_h), &XL_readings2);
  		if(imu2_stat == -1) {
  			// IMU 2 read error
			log_peri_message(ERR_IMU_READ "2", FC_ERR_PERI_TYPE_IMU2);
  		}
  		IMU_getAngRate(&(sensors_h.imu1_h), &angRate_readings1);
  		IMU_getAngRate(&(sensors_h.imu2_h), &angRate_readings2);

  	  	float pres1 = 0.0;
  	  	float pres2 = 0.0;
  	  	int bar1_stat = MS5611_getPres(&(sensors_h.bar1_h), &pres1, &(sensors_h.prom1), OSR_1024);
  	  	if(bar1_stat) {
  	  		// BAR 1 read error
			log_peri_message(ERR_BAR_READ "1", FC_ERR_PERI_TYPE_BAR1);
  	  	}
  	  	int bar2_stat = MS5611_getPres(&(sensors_h.bar2_h), &pres2, &(sensors_h.prom2), OSR_1024);
  	  	if(bar2_stat) {
  	  		// BAR 2 read error
			log_peri_message(ERR_BAR_READ "2", FC_ERR_PERI_TYPE_BAR2);
  	  	}

  	  	float TCvalues[3];
  	  	int TC_stat = ADS_readAll(&(sensors_h.tc_main_h), TCvalues);
  	  	if(TC_stat || isnan(TCvalues[0]) || isnan(TCvalues[1]) || isnan(TCvalues[2])) {
  	  		// ADS read error
			log_peri_message(ERR_ADS_READ, FC_ERR_PERI_TYPE_ADS);
  	  	}

  	  	VLV_OpenLoad vlv1_old = 0;
  	  	VLV_OpenLoad vlv2_old = 0;
  	  	VLV_OpenLoad vlv3_old = 0;
  	  	uint8_t old_stat = 1;

  	  	if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
  	  		vlv1_old = VLV_isOpenLoad(Rocket_h.fcValves[0]);
  	  		vlv2_old = VLV_isOpenLoad(Rocket_h.fcValves[1]);
  	  		vlv3_old = VLV_isOpenLoad(Rocket_h.fcValves[2]);
  	  		old_stat = 0;
  	  		xSemaphoreGive(Rocket_h.fcValve_access);
  	  	}

  	  	uint64_t recordtime = get_rtc_time();

  	  	// Set global rocket state struct
  		if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
  			if(!bar1_stat) {
  				Rocket_h.fcState.bar1 = pres1;
  			}

  			if(!bar2_stat) {
  				Rocket_h.fcState.bar2 = pres2;
  			}

  			Rocket_h.fcState.imu1_A = XL_readings1;
  			Rocket_h.fcState.imu1_W = angRate_readings1;
  			Rocket_h.fcState.imu2_A = XL_readings2;
  			Rocket_h.fcState.imu2_W = angRate_readings2;

  			Rocket_h.fcState.pt1 = PT_calc(PT1_h, adc_values[ADC_PT1_I]);
  			Rocket_h.fcState.pt2 = PT_calc(PT2_h, adc_values[ADC_PT2_I]);
  			Rocket_h.fcState.pt3 = PT_calc(PT3_h, adc_values[ADC_PT3_I]);
  			Rocket_h.fcState.pt4 = PT_calc(PT4_h, adc_values[ADC_PT4_I]);
  			Rocket_h.fcState.pt5 = PT_calc(PT5_h, adc_values[ADC_PT5_I]);

  			Rocket_h.fcState.bus24v_voltage = bus_voltage_calc(adc_values[ADC_24V_BUS_I], POWER_24V_RES_A, POWER_24V_RES_B);
  			Rocket_h.fcState.bus12v_voltage = bus_voltage_calc(adc_values[ADC_12V_BUS_I], POWER_12V_RES_A, POWER_12V_RES_B);
  			Rocket_h.fcState.bus5v_voltage = bus_voltage_calc(adc_values[ADC_5V_BUS_I], POWER_5V_RES_A, POWER_5V_RES_B);
  			Rocket_h.fcState.bus3v3_voltage = bus_voltage_calc(adc_values[ADC_3V3_BUS_I], POWER_3V3_RES_A, POWER_3V3_RES_B);

  			Rocket_h.fcState.bus24v_current = current_sense_calc(adc_values[ADC_24V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Rocket_h.fcState.bus12v_current = current_sense_calc(adc_values[ADC_12V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Rocket_h.fcState.bus5v_current = current_sense_calc(adc_values[ADC_5V_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);
  			Rocket_h.fcState.bus3v3_current = current_sense_calc(adc_values[ADC_3V3_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);

  			Rocket_h.fcState.vlv1_current = current_sense_calc(adc_values[ADC_VLV1_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Rocket_h.fcState.vlv2_current = current_sense_calc(adc_values[ADC_VLV2_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Rocket_h.fcState.vlv3_current = current_sense_calc(adc_values[ADC_VLV3_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);

  			if(!TC_stat) {
  				if(!isnan(TCvalues[0])) {
  					Rocket_h.fcState.tc1 = TCvalues[0];
  				}
  				if(!isnan(TCvalues[1])) {
  					Rocket_h.fcState.tc2 = TCvalues[1];
  				}
  				if(!isnan(TCvalues[2])) {
  					Rocket_h.fcState.tc3 = TCvalues[2];
  				}
  			}

  			if(!old_stat) {
  				Rocket_h.fcState.vlv1_old = vlv1_old;
  				Rocket_h.fcState.vlv2_old = vlv2_old;
  				Rocket_h.fcState.vlv3_old = vlv3_old;
  			}

  			Rocket_h.fcState.timestamp = recordtime;

  			xSemaphoreGive(Rocket_h.fcState_access);
  		}
  		else {
  			// No telemetry updated
  			log_message(ERR_TELEM_NOT_UPDATED, FC_ERR_TYPE_TELEM_NUPDATED);
  		}

  		Message telemsg = {0};
  		telemsg.type = MSG_TELEMETRY;
  		if(!pack_fc_telemetry_msg(&(telemsg.data.telemetry), recordtime, 5)) {
  			if(send_msg_to_device(LimeWire_d, &telemsg, 5, 11 + (4 * FC_TELEMETRY_CHANNELS) + 5) == -2) {
  				// txbuffer full or memory error
  	  			log_message(ERR_TELEM_MEM_ERR, FC_ERR_TYPE_TELEM_MEM_ERR);
  			}
  		}

  		// Target TELEMETRY_HZ polling rate, but always delay for at least 1 tick, otherwise we risk not letting other tasks get CPU time. Actually I'm not sure this is true, but a 1ms delay is fine regardless

		uint32_t delta = HAL_GetTick() - startTime;
		if(delta > (1000 / TELEMETRY_HZ) * 2) {
			// telemetry collection overtime by more than 2x
  			log_message(ERR_TELEM_OVERTIME, FC_ERR_TYPE_TELEM_OVERTIME);
		}
		osDelay((1000 / TELEMETRY_HZ) > delta ? (1000 / TELEMETRY_HZ) - delta : 1);
	}

	// Reference code

	/*struct netbuf *outbuf = netbuf_new();
	void *pkt_buf = netbuf_alloc(outbuf, 3);

	memcpy(pkt_buf, "hi", 3);
	err_t send_err = netconn_sendto(sendudp, outbuf, &debug_addr, 1234);
	netbuf_delete(outbuf);

	HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, 0);
	osDelay(2000);
	HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, 1);
	gps_handler gps;
	gps.huart = &huart10;
	int status = init_gps(&gps);
	for(;;) {
		if(xSemaphoreTake(gps.semaphore, 2)) {
			char temp_rx[100];
			if(gps.active_rx_buffer == 1) {
				memcpy(temp_rx, gps.rx_buffer_2, 100);
			}
			else {
				memcpy(temp_rx, gps.rx_buffer_1, 100);
			}
			gps_data data;
			parse_gps_sentence(temp_rx, &data);
		}
		osDelay(100);
	}*/
}

void ProcessPackets(void *argument) {
	// Started processing thread
	log_message(STAT_PACKET_TASK_STARTED, -1);
	for(;;) {
		Raw_message msg = {0};
		int read_stat = server_read(&msg, 1000);
		if(read_stat == 0) {
			// The way the msg.bufferptr memory is handled past this point is that any result that doesn't relay msg to a different destination should NOT continue early, any result that does relay msg should continue early
			Message parsedmsg = {0};
			if(deserialize_message(msg.bufferptr, msg.packet_len, &parsedmsg) > 0) {
				switch(parsedmsg.type) {
				    case MSG_TELEMETRY: {
				        // Save and relay to Limewire
				    	if(parsedmsg.data.telemetry.board_id == BOARD_FR) {
				    		if(unpack_fr_telemetry(&(parsedmsg.data.telemetry), 5)) {
				    			// Failed to save data
				    			log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
				    		}
				    	}
				    	else {
					    	if(unpack_bb_telemetry(&(parsedmsg.data.telemetry), 5)) {
					    		// Failed to save data
					    		log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
					    	}
				    	}

				    	if(send_raw_msg_to_device(LimeWire_d, &msg, 5) == 0) {
				    		// Continue to prevent freeing memory we're still using
				    		continue;
				    	}
				    	else {
				    	  	// Server not up, target device not connected, or txbuffer is full
				    	}
				        break;
				    }
				    case MSG_VALVE_COMMAND: {
				    	if(check_valve_id(parsedmsg.data.valve_command.valve_id)) {
				    		if(get_valve_board(parsedmsg.data.valve_command.valve_id) == BOARD_FC) {
				    			// Do valve command and send state message
				    			Valve_State_t endState = set_and_update_valve(get_valve(parsedmsg.data.valve_command.valve_id), parsedmsg.data.valve_command.valve_state);
				    			Message returnMsg = {0};
				    			returnMsg.type = MSG_VALVE_STATE;
				    			returnMsg.data.valve_state.valve_state = endState;
				    			returnMsg.data.valve_state.valve_id = parsedmsg.data.valve_command.valve_id;
				    			returnMsg.data.valve_state.timestamp = get_rtc_time();
				      			if(send_msg_to_device(LimeWire_d, &returnMsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5) != 0) {
				      				// Server not up, target device not connected, or txbuffer is full
				      			}
				    		}
				    		else {
				    			// Relay to Bay Boards
				    			if(send_raw_msg_to_device(get_valve_board(parsedmsg.data.valve_command.valve_id), &msg, 5) == 0) {
						    		// Continue to prevent freeing memory we're still using
						    		continue;
				    			}
				    			else {
				    				// Server not up, target device not connected, or txbuffer is full
				    			}
				    		}
				    	}
				    	else {
				    		// Invalid valve id
				    		log_message(ERR_PROCESS_VLV_CMD_BADID, FC_ERR_TYPE_BAD_VLVID);
				    	}
				        break;
				    }
				    case MSG_VALVE_STATE: {
				        // Save and relay to Limewire
				    	if(check_valve_id(parsedmsg.data.valve_state.valve_id)) {
					    	switch(get_valve_board(parsedmsg.data.valve_state.valve_id)) {
					    	    case BOARD_BAY_1: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb1Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb1ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb1Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    case BOARD_BAY_2: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb2Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb2ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb2Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    case BOARD_BAY_3: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb3Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb3ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb3Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    default: {
					    	        // The flight computer should not receive valve state messages for its own valves
					    	        break;
					    	    }
					    	}

					    	if(send_raw_msg_to_device(LimeWire_d, &msg, 5) == 0) {
					    		// Continue to prevent freeing memory we're still using
					    		continue;
					    	}
					    	else {
					    	  	// Server not up, target device not connected, or txbuffer is full
					    	}
				    	}
				    	else {
				    		// Invalid valve id
				    		log_message(ERR_PROCESS_VLV_STATE_BADID, FC_ERR_TYPE_BAD_VLVID);
				    	}
				        break;
				    }
				    case MSG_DEVICE_COMMAND: {
				    	if(parsedmsg.data.device_command.board_id == BOARD_FC) {
				    		// Process device command
				    		switch(parsedmsg.data.device_command.cmd_id) {
				    			case DEVICE_CMD_RESET: {
				    				reset_board();
				    				break;
				    			}
				    			case DEVICE_CMD_CLEAR_FLASH: {
				    				clear_flash();
				    				break;
				    			}
				    			case DEVICE_CMD_QUERY_FLASH: {
				    				log_flash_storage();
				    				break;
				    			}
				    			default: {
				    				break;
				    			}
				    		}
				    	}
				    	else {
			    			// Relay to other boards
			    			if(send_raw_msg_to_device(parsedmsg.data.device_command.board_id, &msg, 5) == 0) {
					    		// Continue to prevent freeing memory we're still using
					    		continue;
			    			}
			    			else {
			    				// Server not up, target device not connected, or txbuffer is full
			    			}
				    	}
				    	break;
				    }
				    default: {
				        break;
				    }
				}
			}
			else {
				// Unknown message type
				log_message(ERR_UNKNOWN_LMP_PACKET, FC_ERR_TYPE_UNKNOWN_LMP);
			}
			free(msg.bufferptr);
		}
		else if(read_stat == -1) {
			// Server down, delay to prevent taking CPU time from other tasks
			osDelay(100);
		}
		else {
			// Timeout on waiting for messages
		}
	}
}

void FlashClearTask(void *argument) {
    for(;;) {
        if(xSemaphoreTake(flash_clear_mutex, portMAX_DELAY) == pdPASS) {
        	if(xSemaphoreTake(flash_mutex, 100) == pdPASS) {
        		fc_finish_flash_write(&flash_h); // Flush write buffer
            	fc_erase_flash(&flash_h); // Clear
        		xSemaphoreGive(flash_mutex);
        		log_message(STAT_CLEAR_FLASH, -1);
        	}
        }
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAndMonitor */
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAndMonitor */
void StartAndMonitor(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  	// Signal end of critical section
    inittimers_t * timers = (inittimers_t *) argument;

	if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		if(errormsgudp) {
			// No UDP error
		    if(!timers->ledTimer) {
		    	// No flash error, led constant on indicates end of critical section
		    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
		    	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
		    	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 1);
		    }
		}
		else {
			// UDP error
		    if(timers->ledTimer) {
		    	// Flash error, change delay since UDP error is more critical to address
		    	xTimerChangePeriod(timers->ledTimer, 250, 0);
		    }
		    else {
		    	// No flash error, start led flashing
		    	timers->ledTimer = xTimerCreate("led", 250, pdTRUE, NULL, toggleLEDPins);
		    	xTimerStart(timers->ledTimer, 0);
		    }
		}
		xSemaphoreGive(errorudp_mutex);
	}

    if(!timers->buzzTimer) {
    	// No eeprom error, short buzz indicates the end of the critical section
    	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 1);
    	timers->buzzTimer = xTimerCreate("sbuzz", 1000, pdFALSE, NULL, buzzerOff);
    	xTimerStart(timers->buzzTimer, 0);
    }

	// Setup TCP server
	if(server_create(loaded_config.limewireIP, loaded_config.bayboard1IP, loaded_config.bayboard2IP, loaded_config.bayboard3IP, loaded_config.flightrecordIP)) {
		// memory error creating TCP server, TCP server will not start
		log_message(FC_ERR_CREAT_TCP_MEM_ERR, -1);
	}

	// Init rocket state struct
	memset(&Rocket_h, 0, sizeof(Rocket_h)); // Reset contents
	Rocket_h.fcState_access = xSemaphoreCreateMutex();
	Rocket_h.fcValve_access = xSemaphoreCreateMutex();
	Rocket_h.bb1State_access = xSemaphoreCreateMutex();
	Rocket_h.bb1Valve_access = xSemaphoreCreateMutex();
	Rocket_h.bb2State_access = xSemaphoreCreateMutex();
	Rocket_h.bb2Valve_access = xSemaphoreCreateMutex();
	Rocket_h.bb3State_access = xSemaphoreCreateMutex();
	Rocket_h.bb3Valve_access = xSemaphoreCreateMutex();

	// Config SR
  	Shift_Reg reg = {0};
  	reg.VLV_CTR_GPIO_Port = VLV_CTRL_GPIO_Port;
  	reg.VLV_CTR_GPIO_Pin = VLV_CTRL_Pin;
  	reg.VLV_CLK_GPIO_Port = BUFF_CLK_GPIO_Port;
  	reg.VLV_CLK_GPIO_Pin = BUFF_CLK_Pin;
  	reg.VLV_CLR_GPIO_Port = BUFF_CLR_GPIO_Port;
  	reg.VLV_CLR_GPIO_Pin = BUFF_CLR_Pin;

  	VLV_Set_Conf(reg, loaded_config.vlv1_en, loaded_config.vlv1_v, loaded_config.vlv2_en, loaded_config.vlv2_v, loaded_config.vlv3_en, loaded_config.vlv3_v);

  	// Load valve pins
  	Rocket_h.fcValves[0].VLV_EN_GPIO_Port = VLV1_EN_GPIO_Port;
  	Rocket_h.fcValves[0].VLV_EN_GPIO_Pin = VLV1_EN_Pin;
  	Rocket_h.fcValves[0].VLV_OLD_GPIO_Port = VLV1_OLD_GPIO_Port;
  	Rocket_h.fcValves[0].VLV_OLD_GPIO_Pin = VLV1_OLD_Pin;

  	Rocket_h.fcValves[1].VLV_EN_GPIO_Port = VLV2_EN_GPIO_Port;
  	Rocket_h.fcValves[1].VLV_EN_GPIO_Pin = VLV2_EN_Pin;
  	Rocket_h.fcValves[1].VLV_OLD_GPIO_Port = VLV2_OLD_GPIO_Port;
  	Rocket_h.fcValves[1].VLV_OLD_GPIO_Pin = VLV2_OLD_Pin;

  	Rocket_h.fcValves[2].VLV_EN_GPIO_Port = VLV3_EN_GPIO_Port;
  	Rocket_h.fcValves[2].VLV_EN_GPIO_Pin = VLV3_EN_Pin;
  	Rocket_h.fcValves[2].VLV_OLD_GPIO_Port = VLV3_OLD_GPIO_Port;
  	Rocket_h.fcValves[2].VLV_OLD_GPIO_Pin = VLV3_OLD_Pin;

  	// These should always be 0 but in the case that code is added later than adds a default valve state I'm leaving this here
  	Rocket_h.fcValveStates[0] = VLV_State(Rocket_h.fcValves[0]);
  	Rocket_h.fcValveStates[1] = VLV_State(Rocket_h.fcValves[1]);
  	Rocket_h.fcValveStates[2] = VLV_State(Rocket_h.fcValves[2]);

	// Init sensors and peripherals
  	// ADC
  	sensors_h.adc_h.MAX11128_CS_PORT = ADC_CS_GPIO_Port;
  	sensors_h.adc_h.MAX11128_CS_ADDR = ADC_CS_Pin;
  	sensors_h.adc_h.HARDWARE_CONFIGURATION = NO_EOC_NOR_CNVST;
  	sensors_h.adc_h.NUM_CHANNELS = 16;

  	init_adc(&hspi4, &(sensors_h.adc_h));
  	// Check to make sure the chip is connected
	uint16_t adc_values[16] = {0};
	read_adc(&hspi4, &(sensors_h.adc_h), adc_values);
	if(adc_values[ADC_3V3_BUS_I] == 0) { // 3v3 bus voltage should always be greater than 0
		log_message(FC_ERR_SING_ADC_INIT, -1);
	}

  	// TC ADCs
  	ADS_configTC(&(sensors_h.TCs[0]), &hspi2, GPIOB, GPIO_PIN_14, 0x0005, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc1_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[1]), &hspi2, GPIOB, GPIO_PIN_14, 0x0005, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN2_AIN3, loaded_config.tc2_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[2]), &hspi2, GPIOB, GPIO_PIN_14, 0x0005, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc3_gain, ADS_DATA_RATE_600);

  	if(ADS_init(&(sensors_h.tc_main_h), sensors_h.TCs, 3)) {
  		// ADS thread start error
  		log_message(ERR_ADS_INIT_THREAD_ERR, -1);
  	}

  	// IMUs
  	sensors_h.imu1_h.hi2c = &hi2c5;
  	sensors_h.imu1_h.I2C_TIMEOUT = 5;
  	sensors_h.imu1_h.XL_x_offset = 0;
  	sensors_h.imu1_h.XL_y_offset = 0;
  	sensors_h.imu1_h.XL_z_offset = 0;
  	sensors_h.imu1_h.G_x_offset = 0;
  	sensors_h.imu1_h.G_y_offset = 0;
  	sensors_h.imu1_h.G_z_offset = 0;
  	sensors_h.imu1_h.SA0 = 0;

  	sensors_h.imu2_h.hi2c = &hi2c5;
  	sensors_h.imu2_h.I2C_TIMEOUT = 5;
  	sensors_h.imu2_h.XL_x_offset = 0;
  	sensors_h.imu2_h.XL_y_offset = 0;
  	sensors_h.imu2_h.XL_z_offset = 0;
  	sensors_h.imu2_h.G_x_offset = 0;
  	sensors_h.imu2_h.G_y_offset = 0;
  	sensors_h.imu2_h.G_z_offset = 0;
  	sensors_h.imu2_h.SA0 = 1;

  	if(IMU_init(&(sensors_h.imu1_h)) == -1) {
  		// IMU 1 init error
  		log_message(ERR_IMU_INIT "1", -1);
  	}
  	if(IMU_init(&(sensors_h.imu2_h)) == -1) {
  		// IMU 2 init error
  		log_message(ERR_IMU_INIT "2", -1);
  	}

  	// Barometers
  	sensors_h.bar1_h.hspi = &hspi6;
  	sensors_h.bar1_h.SPI_TIMEOUT = 5;
  	sensors_h.bar1_h.CS_GPIO_Port = BAR1_CS_GPIO_Port;
  	sensors_h.bar1_h.CS_GPIO_Pin = BAR1_CS_Pin;
  	sensors_h.bar1_h.pres_offset = 0;
  	sensors_h.bar1_h.alt_offset = 0;

  	sensors_h.bar2_h.hspi = &hspi6;
  	sensors_h.bar2_h.SPI_TIMEOUT = 5;
  	sensors_h.bar2_h.CS_GPIO_Port = BAR2_CS_GPIO_Port;
  	sensors_h.bar2_h.CS_GPIO_Pin = BAR2_CS_Pin;
  	sensors_h.bar2_h.pres_offset = 0;
  	sensors_h.bar2_h.alt_offset = 0;

  	if(MS5611_Reset(&(sensors_h.bar1_h))) {
  		// Bar 1 init error
  		log_message(ERR_BAR_INIT "1", -1);
  	}
  	MS5611_readPROM(&(sensors_h.bar1_h), &(sensors_h.prom1));
  	
  	if(MS5611_Reset(&(sensors_h.bar2_h))) {
  		// Bar 2 init error
  		log_message(ERR_BAR_INIT "2", -1);
  	}
  	MS5611_readPROM(&(sensors_h.bar2_h), &(sensors_h.prom2));

	// Start tasks

  	// Start telemetry task
  	telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetry_task_attr);

  	// Start packet handler
  	packetTaskHandle = osThreadNew(ProcessPackets, NULL, &packet_task_attr);

  	// Start TCP server only if the ethernet link is up
	if(netif_is_link_up(&gnetif)) {
		switch(server_init()) {
			case 0: {
				// TCP server started
				log_message(FC_STAT_TCP_SERV_RUNNING, -1);
				break;
			}
			case -1: {
				break;
			}
			case -2: {
				// one of the threads failed to start
				log_message(FC_ERR_INIT_TCP_THREAD_ERR, -1);
				break;
			}
			case -3: {
				break;
			}
			default: {
				break;
			}
		}
	}
	// log startup done
	log_message(STAT_STARTUP_DONE, -1);

	// Then, error handling and logging in telemetry and packet processing tasks, tftp, lwip link up and down
	// Then, hard fault handling and logging
	/*
	 * ALL VALVE STATES SHOULD BE SENT ON THE START OF CONNECTION WITH LIMEWIRE (FC FOR BBs) (WHEN THE FC CONNECTS TO LIMEWIRE SEND THE VALVE STATES FOR THE ENTIRE ROCKET)
	 *
	 * fc telemetry comes from struct, sent on a 50hz loop
	 * bb telemetry comes from tcp packets, relayed to limewire and stored for optional access in autosequences
	 * fc valve states are sent after valve commands are sent AND possibly every now and then, in case something gets disconnected, then stored for optional access
	 * bb valve states are relayed from tcp packets, and saved for optional access
	 * all valve commands are processed when received. fc relays commands from limewire to bb
	 *
	 * telemetry task - read from peripherals and send telemetry msg
	 * tcp process task - process incoming packets for relaying or valve stuff
	 */
	//const char sample[] = "Please work, I don't want to do more work";
	//write_ascii_to_flash(sample, sizeof(sample) - 1, FLASH_MSG_MARK);

	uint8_t telemcounter = 0;
	uint8_t statecounter = 0;

	uint8_t limewireconnected = 0;

	uint32_t startTick = HAL_GetTick();
	uint32_t lastflashfull = 0;
	/* Infinite loop */
	for(;;) {
		errormsg_t logmsg;
		if(!list_pop(errorMsgList, (void *)&logmsg, 5)) {
			uint8_t flashstat = write_raw_to_flash(logmsg.content, logmsg.len);
			if(flashstat == 2) {
				// Flash is full, send UDP message every 5 seconds
				if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
					send_flash_full();
					lastflashfull = HAL_GetTick();
				}
			}
			if(xSemaphoreTake(errorudp_mutex, 5) == pdPASS) {
				if(errormsgudp) {
					struct netbuf *outbuf = netbuf_new();
					if(outbuf) {
						void *pkt_buf = netbuf_alloc(outbuf, logmsg.len - 2);
						if(pkt_buf) {
							memcpy(pkt_buf, &(logmsg.content[1]), logmsg.len - 2);
							netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
						}
						netbuf_delete(outbuf);
					}
				}
				xSemaphoreGive(errorudp_mutex);
			}

			free(logmsg.content); // No memory leaks here hehe
		}
		if(HAL_GetTick() - startTick > 250) {
			startTick = HAL_GetTick();
			//size_t freemem = xPortGetFreeHeapSize();
			if(xSemaphoreTake(errormsg_mutex, 5) == pdPASS) {
				for(int i = 0;i < ERROR_MSG_TYPES;i++) {
					uint8_t val = (i % 2) == 0 ? errormsgtimers[i / 2] & 0x0F : errormsgtimers[i / 2] >> 4;
					if(val) {
						val--;
						if(i % 2) {
							errormsgtimers[i / 2] = (errormsgtimers[i / 2] & 0x0F) | (val << 4);
						}
						else {
							errormsgtimers[i / 2] = (errormsgtimers[i / 2] & 0xF0) | val;
						}
					}
				}
				xSemaphoreGive(errormsg_mutex);
			}

			if(xSemaphoreTake(perierrormsg_mutex, 5) == pdPASS) {
				for(int i = 0;i < PERI_ERROR_MSG_TYPES;i++) {
					if(perierrormsgtimers[i]) {
						perierrormsgtimers[i]--;
					}
				}
				xSemaphoreGive(perierrormsg_mutex);
			}

			int limefd = get_device_fd(LimeWire_d);
			if(limefd > -2) {
				if(limewireconnected == 0 && limefd > -1) {
					uint64_t valvetime = get_rtc_time();
			  	  	if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
			  	  		Valve_State_t vstates[3];
						for(int i = 0;i < 3;i++) {
							vstates[i] = Rocket_h.fcValveStates[i];
						}
			  	  		xSemaphoreGive(Rocket_h.fcValve_access);
						for(int i = 0;i < 3;i++) {
							Message statemsg = {0};
							statemsg.type = MSG_VALVE_STATE;
							statemsg.data.valve_state.timestamp = valvetime;
							statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_FC, i);
							statemsg.data.valve_state.valve_state = vstates[i];
							send_msg_to_device(LimeWire_d, &statemsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5);
						}
			  	  	}
			  	  	if(xSemaphoreTake(Rocket_h.bb1Valve_access, 5) == pdPASS) {
			  	  		Valve_State_t vstates[7];
						for(int i = 0;i < 7;i++) {
							vstates[i] = Rocket_h.bb1ValveStates[i];
						}
			  	  		xSemaphoreGive(Rocket_h.bb1Valve_access);
						for(int i = 0;i < 7;i++) {
							Message statemsg = {0};
							statemsg.type = MSG_VALVE_STATE;
							statemsg.data.valve_state.timestamp = valvetime;
							statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_BAY_1, i);
							statemsg.data.valve_state.valve_state = vstates[i];
							send_msg_to_device(LimeWire_d, &statemsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5);
						}
			  	  	}
			  	  	if(xSemaphoreTake(Rocket_h.bb2Valve_access, 5) == pdPASS) {
			  	  		Valve_State_t vstates[7];
						for(int i = 0;i < 7;i++) {
							vstates[i] = Rocket_h.bb2ValveStates[i];
						}
			  	  		xSemaphoreGive(Rocket_h.bb2Valve_access);
						for(int i = 0;i < 7;i++) {
							Message statemsg = {0};
							statemsg.type = MSG_VALVE_STATE;
							statemsg.data.valve_state.timestamp = valvetime;
							statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_BAY_2, i);
							statemsg.data.valve_state.valve_state = vstates[i];
							send_msg_to_device(LimeWire_d, &statemsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5);
						}
			  	  	}
			  	  	if(xSemaphoreTake(Rocket_h.bb3Valve_access, 5) == pdPASS) {
			  	  		Valve_State_t vstates[7];
						for(int i = 0;i < 7;i++) {
							vstates[i] = Rocket_h.bb3ValveStates[i];
						}
			  	  		xSemaphoreGive(Rocket_h.bb3Valve_access);
						for(int i = 0;i < 7;i++) {
							Message statemsg = {0};
							statemsg.type = MSG_VALVE_STATE;
							statemsg.data.valve_state.timestamp = valvetime;
							statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_BAY_3, i);
							statemsg.data.valve_state.valve_state = vstates[i];
							send_msg_to_device(LimeWire_d, &statemsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5);
						}
			  	  	}
				}
				if(limefd > -1) {
					limewireconnected = 1;
				}
				else {
					limewireconnected = 0;
				}
			}

			if(telemcounter == 0) {
		  		Message telemsg = {0};
		  		telemsg.type = MSG_TELEMETRY;
		  		if(!pack_fc_telemetry_msg(&(telemsg.data.telemetry), get_rtc_time(), 5)) {
		  			uint8_t tempbuffer[11 + (4 * FC_TELEMETRY_CHANNELS) + 5];
		  			int buflen = serialize_message(&telemsg, tempbuffer, 11 + (4 * FC_TELEMETRY_CHANNELS) + 5);
		  			if(buflen != -1) {
		  				if(log_lmp_packet(tempbuffer, buflen) == 2) {
		  					if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
		  						send_flash_full();
		  						lastflashfull = HAL_GetTick();
		  					}
		  				}
		  			}
		  		}
				telemcounter = 0; // 4hz
			}
			else {
				telemcounter--;
			}

			if(statecounter == 0) {
		  	  	if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
		  	  		Valve_State_t vstates[3];
					for(int i = 0;i < 3;i++) {
						vstates[i] = Rocket_h.fcValveStates[i];
					}
		  	  		xSemaphoreGive(Rocket_h.fcValve_access);
					uint64_t valvetime = get_rtc_time();
					for(int i = 0;i < 3;i++) {
						Message statemsg = {0};
						statemsg.type = MSG_VALVE_STATE;
						statemsg.data.valve_state.timestamp = valvetime;
						statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_FC, i);
						statemsg.data.valve_state.valve_state = vstates[i];
						uint8_t tempbuffer[MAX_VALVE_STATE_MSG_SIZE + 5];
						int buflen = serialize_message(&statemsg, tempbuffer, MAX_VALVE_STATE_MSG_SIZE + 5);
						if(buflen != -1) {
							if(log_lmp_packet(tempbuffer, buflen) == 2) {
								if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
									send_flash_full();
									lastflashfull = HAL_GetTick();
								}
							}
						}
					}
		  	  	}
				statecounter = 1; // 2hz
			}
			else {
				statecounter--;
			}
		}

	  osDelay(5);
	}
  /* USER CODE END 5 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000200;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
