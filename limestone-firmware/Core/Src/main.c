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

	Valve_State_t vlv1_state;
	Valve_State_t vlv2_state;
	Valve_State_t vlv3_state;
} Flight_Computer_State_t;

typedef struct {
	SemaphoreHandle_t fcState_access;
	Flight_Computer_State_t fcState;
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
	ip4_addr_t bayboard1IP;
	ip4_addr_t bayboard2IP;
	ip4_addr_t bayboard3IP;
	ip4_addr_t flightrecordIP;
} EEPROM_conf_t;
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
/* Definitions for setupTask */
osThreadId_t setupTaskHandle;
const osThreadAttr_t setupTask_attributes = {
  .name = "setupTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t RTC_mutex; // For safe concurrent access of the RTC

Rocket_State_t Rocket_h; // Main rocket state handle
eeprom_t eeprom_h = {0};

extern struct netif gnetif;

size_t eeprom_cursor = 0;
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
void StartAndMonitor(void *argument);
void setupAndStart(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t getTimestamp() {
	return HAL_GetTick();
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

	if(xSemaphoreTake(RTC_mutex, 2) == pdPASS) {
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
}


/**
 * Get Unix timestamp in nanoseconds
 * Returns 0 if RTC could not be accessed
 */
uint64_t get_rtc_time() {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(xSemaphoreTake(RTC_mutex, 2) == pdPASS) {
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

// Pack telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int pack_fc_telemetry_msg(TelemetryMessage *msg, uint64_t timestamp, uint8_t timeout_ticks) {
	msg->board_id = BOARD_FC;
	msg->num_channels = NUM_FC_CHANNELS;
	msg->timestamp = timestamp;
	if(xSemaphoreTake(Rocket_h.fcState_access, timeout_ticks) == pdPASS) {
		msg->telemetry_data[0] = Rocket_h.fcState.vlv1_current;
		msg->telemetry_data[1] = Rocket_h.fcState.vlv1_old;
		msg->telemetry_data[2] = Rocket_h.fcState.vlv2_current;
		msg->telemetry_data[3] = Rocket_h.fcState.vlv2_old;
		msg->telemetry_data[4] = Rocket_h.fcState.vlv3_current;
		msg->telemetry_data[5] = Rocket_h.fcState.vlv3_old;
		msg->telemetry_data[6] = Rocket_h.fcState.pt1;
		msg->telemetry_data[7] = Rocket_h.fcState.pt2;
		msg->telemetry_data[8] = Rocket_h.fcState.pt3;
		msg->telemetry_data[9] = Rocket_h.fcState.pt4;
		msg->telemetry_data[10] = Rocket_h.fcState.pt5;
		msg->telemetry_data[11] = Rocket_h.fcState.tc1;
		msg->telemetry_data[12] = Rocket_h.fcState.tc2;
		msg->telemetry_data[13] = Rocket_h.fcState.tc3;
		msg->telemetry_data[14] = Rocket_h.fcState.imu1_A.XL_x;
		msg->telemetry_data[15] = Rocket_h.fcState.imu1_A.XL_y;
		msg->telemetry_data[16] = Rocket_h.fcState.imu1_A.XL_z;
		msg->telemetry_data[17] = Rocket_h.fcState.imu1_W.G_y;
		msg->telemetry_data[18] = Rocket_h.fcState.imu1_W.G_x;
		msg->telemetry_data[19] = Rocket_h.fcState.imu1_W.G_z;
		msg->telemetry_data[20] = Rocket_h.fcState.imu2_A.XL_x;
		msg->telemetry_data[21] = Rocket_h.fcState.imu2_A.XL_y;
		msg->telemetry_data[22] = Rocket_h.fcState.imu2_A.XL_z;
		msg->telemetry_data[23] = Rocket_h.fcState.imu2_W.G_y;
		msg->telemetry_data[24] = Rocket_h.fcState.imu2_W.G_x;
		msg->telemetry_data[25] = Rocket_h.fcState.imu2_W.G_z;
		msg->telemetry_data[26] = Rocket_h.fcState.gps_lat;
		msg->telemetry_data[27] = Rocket_h.fcState.gps_long;
		msg->telemetry_data[28] = Rocket_h.fcState.gps_alt;
		msg->telemetry_data[29] = Rocket_h.fcState.bar1;
		msg->telemetry_data[30] = Rocket_h.fcState.bar2;
		msg->telemetry_data[31] = Rocket_h.fcState.bus24v_voltage;
		msg->telemetry_data[32] = Rocket_h.fcState.bus24v_current;
		msg->telemetry_data[33] = Rocket_h.fcState.bus12v_voltage;
		msg->telemetry_data[34] = Rocket_h.fcState.bus12v_current;
		msg->telemetry_data[35] = Rocket_h.fcState.bus5v_voltage;
		msg->telemetry_data[36] = Rocket_h.fcState.bus5v_current;
		msg->telemetry_data[37] = Rocket_h.fcState.bus3v3_voltage;
		msg->telemetry_data[38] = Rocket_h.fcState.bus3v3_current;
		xSemaphoreGive(Rocket_h.fcState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Load board configuration from a buffer. Returns 0 on success, -1 on an eeprom error or if a configuration value is outside its allowed range
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
	IP4_ADDR(&(conf->bayboard1IP), buffer[73], buffer[74], buffer[75], buffer[76]);
	IP4_ADDR(&(conf->bayboard2IP), buffer[77], buffer[78], buffer[79], buffer[80]);
	IP4_ADDR(&(conf->bayboard3IP), buffer[81], buffer[82], buffer[83], buffer[84]);
	IP4_ADDR(&(conf->flightrecordIP), buffer[85], buffer[86], buffer[87], buffer[88]);

	if(conf->tc1_gain > 0x0E || conf->tc2_gain > 0x0E || conf->tc3_gain > 0x0E) {
		return -1;
	}

	if(conf->vlv1_v > 0x01 || conf->vlv1_en > 0x01 || conf->vlv2_v > 0x01 || conf->vlv2_en > 0x01 || conf->vlv3_v > 0x01 || conf->vlv3_en > 0x01) {
		return -1;
	}
	return 0;
}

void *ftp_open(const char *fname, const char *mode, u8_t write) {
	if(strcmp(fname, "eeprom.bin") == 0) {
		eeprom_cursor = 0;
		return (void *) 1; // 1 refers to eeprom
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
			// TODO: Send too short error
			return;
		}
		// CRC check
		uint32_t sent_crc;
		eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor + FC_EEPROM_LEN, (uint8_t *) &sent_crc, 4);
		if(read_stat != EEPROM_OK) {
			// TODO: Send read error
			return;
		}
		HAL_CRC_Calculate(&hcrc, NULL, 0);
		uint32_t calc_crc;
		int cursor = 0;
		do {
			int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
			uint8_t buf[read_bytes];
			read_stat = eeprom_read_mem(&eeprom_h, FC_EEPROM_LEN + cursor, buf, read_bytes);
			if(read_stat != EEPROM_OK) {
				// TODO: Send read error
				return;
			}
			calc_crc = HAL_CRC_Accumulate(&hcrc, buf, read_bytes);
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
					// TODO: Send copy read error
					return;
				}
				eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, cursor, buf, read_bytes);
				if(write_stat != EEPROM_OK) {
					// TODO: Send copy write error
					return;
				}
				cursor += read_bytes;
			} while(eeprom_cursor - cursor > 0);
		}
		else {
			// TODO: Mismatched CRC
		}
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
			return -1;
		}
		eeprom_cursor += bytes_to_read;
		return bytes_to_read;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  // Create RTC mutex
  RTC_mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startTask */
  startTaskHandle = osThreadNew(StartAndMonitor, NULL, &startTask_attributes);

  /* creation of setupTask */
  setupTaskHandle = osThreadNew(setupAndStart, NULL, &setupTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
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

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU1_INT1_Pin IMU1_INT2_Pin IMU2_INT1_Pin IMU2_INT2_Pin */
  GPIO_InitStruct.Pin = IMU1_INT1_Pin|IMU1_INT2_Pin|IMU2_INT1_Pin|IMU2_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
    // Load EEPROM config
  	PT_t PT1_h = {0.5f, 5000.0f, 4.5f};
  	PT_t PT2_h = {0.5f, 5000.0f, 4.5f};
  	PT_t PT3_h = {0.5f, 5000.0f, 4.5f};
  	PT_t PT4_h = {0.5f, 5000.0f, 4.5f};
  	PT_t PT5_h = {0.5f, 5000.0f, 4.5f};
  	EEPROM_conf_t loaded_config = {0};
  	loaded_config.pt1 = &PT1_h;
  	loaded_config.pt2 = &PT2_h;
  	loaded_config.pt3 = &PT3_h;
  	loaded_config.pt4 = &PT4_h;
  	loaded_config.pt5 = &PT5_h;
    eeprom_init(&eeprom_h, &hi2c1, EEPROM_WC_GPIO_Port, EEPROM_WC_Pin);
    load_eeprom_config(&eeprom_h, &loaded_config);

	// Setup NTP listener
	sntp_setoperatingmode(SNTP_OPMODE_LISTENONLY);

	// Setup TCP server
	ip4_addr_t limewire;
	ip4_addr_t noIP = {0};
	IP4_ADDR(&limewire, 192, 168, 0, 5);
	server_create(limewire, noIP, noIP, noIP, noIP);

	// Init rocket state struct
	memset(&Rocket_h, 0, sizeof(Rocket_h)); // Reset contents
	Rocket_h.fcState_access = xSemaphoreCreateMutex();
	// TODO Init peripherals and start tasks

	if(netif_is_link_up(&gnetif)) {
		server_init();
		sntp_init();
		tftp_init(&my_tftp_ctx);
	}

	// DONE: Add TCP server support to disconnected clients
	//			Test TCP keep-alive
	//			If select returns -1, still go through connections since that probably means a connection was dropped
	//			If recv returns 0, connection is closed so remove it from the list, MAKE SURE TO TAKE MUTEX BEFORE EDITING CONN LIST
	//			If send returns -1, connection is closed, but select will handle it so don't handle it

	// DONE: Figure out what happens when the board starts without being plugged in to ethernet - you must shutdown all active connections/servers and then start them as appropriate in ethernet_link_status_updated - for shutting down the tcp server, give a flag (semaphore) to the 3 tasks, each task will free it's own memory and close sockets and then delete itself, then free the TS lists
	//			For the connection thread, close listener socket. Use a binary semaphore to signal to the threads to stop, then use a counting semaphore and 3 waits to wait until all 3 threads have shut down. Then close all connections and free the memory from the TS queues. No need to free mutex since the threads should all unlock it before exiting
	//			On uplink, use a different init function that does all the same thing except creating the mutex
	//			FIGURE OUT A WAY TO TELL IF THE SERVER WAS STARTED OR NOT. ethernet_link_status_updated SHOULD NOT DO ANYTHING IF THE SERVER HAS NOT BEEN STARTED IN THIS THREAD. ALSO DON'T START THE SERVER UNLESS IT HAS BEEN STOPPED AND VICE VERSA
	//			FIGURE OUT WHAT HAPPENS IF THE PROGRAM IS STARTED UNLINKED. DOES ethernet_link_status_updated STILL TRIGGER WHEN THE LINK IS CREATED, IF SO, DELAY ANY NETWORK INIT (INCLUDING SNTP) UNTIL A LINK IS CREATED.

	// TODO: Do proper errno handling for tcp server, for example send(), select(), accept(), recv()
	// TODO: Possibly add code to default timestamp if rtc isn't set
	// DONE: globals for ip addresses for all boards and limewire, and create functions to get the fd based on which board/ip addr - something like get_fd(BoardId, board), also maybe function where you give the boardId, and it gets the fd and checks if the connection is open and the server is running DONT SEND OR TRY TO RECEIVE ANYTHING UNLESS THE SERVER IS UP
	// DONE: Figure out EEPROM ordering and what goes in there. PT constants, IP addresses, TC gains, valve voltage and enabling, maybe more
	// DONE: FTP server (add at least 2 tcp connects to the limit and tcp pcb mem limit) for EEPROM, when a client connects and tries to write to a file, say eeprom.bin, lock a mutex, write eeprom, and then reload config. If multiple clients try to write to the same thing (e.g. eeprom) ignore all attempts besides the first, DON'T WAIT FOR THE FIRST TO FINISH
	// DONE: Add eeprom readme crc32 info so it can be recreated
	// DONE: Figure out why the ADS1120 driver is giving 0 volt readings sometimes, especially very regularly on the cjc chip
	// TODO: Use the limewire google colab notebook to generate a header file for channel order
	// TODO: All TC config should use 600sps
	// TODO: Add support for commands in LMP and messages.c, then add reset command to reset board. In reset command, close all connections and stop any important things, then reset
	// TODO: AT THE END, add feedback (LED, buzzer) on startup and during normal operations to communicate status, also add while(1); loops on startup if any issues are encountered that the board CANNOT run without, anything that the board can run without should have defaults
	// DONE: heartbeat is dead, figure out keep-alive parameters, talk with rohan and jack about it. Also log info about it readme
	// TODO: Flash and udp message logging, look in design document about it. Figure out how much flash space I have to work with, it should not get deleted, if the space fills up, send a udp error message about it every time an error is attemped to be logged
	// TODO: Log valve states to flash every second, look at design document for more info
	/*
	 * ALL VALVE STATES SHOULD BE SENT ON THE START OF CONNECTION WITH LIMEWIRE (FC FOR BBs)
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
  	ADS_Main_t main_handle = {0};
  	ADS_TC_t multiTCs[3];

  	ADS_configTC(&multiTCs[0], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN0_AIN1, ADS_PGA_GAIN_128, ADS_DATA_RATE_600);
  	ADS_configTC(&multiTCs[1], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN2_AIN3, ADS_PGA_GAIN_1, ADS_DATA_RATE_600);
  	ADS_configTC(&multiTCs[2], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN0_AIN1, ADS_PGA_GAIN_128, ADS_DATA_RATE_600);
  	ADS_init(&main_handle, multiTCs, 3);

  	IMU IMU1;
  	IMU1.hi2c = &hi2c5;
  	IMU1.I2C_TIMEOUT = 9999;
  	IMU1.XL_x_offset = 0;
  	IMU1.XL_y_offset = 0;
  	IMU1.XL_z_offset = 0;
  	IMU1.G_x_offset = 0;
  	IMU1.G_y_offset = 0;
  	IMU1.G_z_offset = 0;
  	IMU1.SA0 = 1;

  	IMU_init(&IMU1);
	/* Infinite loop */
	for(;;) {
  		ADS_Reading_t values[3];
  		int status = ADS_readAllwTimestamps(&main_handle, values);
  		Accel XL_readings1 = {0};
  		IMU_getAccel(&IMU1, &XL_readings1);

  		if(xSemaphoreTake(Rocket_h.fcState_access, 1) == pdPASS) {
  			Rocket_h.fcState.tc1 = values[0].temp_c;
  			Rocket_h.fcState.tc2 = values[1].temp_c;
  			Rocket_h.fcState.tc3 = values[2].temp_c;
  			Rocket_h.fcState.imu1_A = XL_readings1;
  			xSemaphoreGive(Rocket_h.fcState_access);
  			if(is_server_running()) {
  				int limewirefd = get_device_fd(LimeWire_d);
  				if(limewirefd != -1) {
  	  	  			TelemetryMessage telemsg;
  	  	  			if(!pack_fc_telemetry_msg(&telemsg, get_rtc_time(), 1)) {
  	  	  				Message genmsg = {MSG_TELEMETRY, telemsg};
  	  	  				Raw_message msg = {0};
  	  	  				int buflen = serialize_message(&genmsg, msg.buffer, MAX_MSG_LEN);
  	  	  				if(buflen != -1) {
  	  	  	  				msg.packet_len = buflen;
  	  	  	  				msg.connection_fd = limewirefd;
  	  	  	  				server_send(&msg, 10);
  	  	  				}
  	  	  			}
  				}
  			}
  		}

	  // TODO Make sure things are still running and restart anything that stops
	  osDelay(20);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_setupAndStart */
/**
* @brief Function implementing the setupTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_setupAndStart */
void setupAndStart(void *argument)
{
  /* USER CODE BEGIN setupAndStart */
	// REMOVE THIS TASK
	  for(;;) {
		  size_t freemem = xPortGetFreeHeapSize();
		  osDelay(1000);
	  }
	  struct netconn *sendudp = netconn_new(NETCONN_UDP);
	  ip_addr_t debug_addr;
	  IP4_ADDR(&debug_addr, 192, 168, 0, 5);

	  for(;;) {
		  struct netbuf *outbuf = netbuf_new();
		  void *pkt_buf = netbuf_alloc(outbuf, 9);
		  uint64_t cur_time = get_rtc_time();
		  //uint64_t cur_time = 0;
		  uint8_t outarr[8];

		  for (int i = 0; i < 8; i++) {
			  outarr[i] = (uint8_t)((cur_time >> (8 * (7 - i))) & 0xFF);
		  }

		  //netbuf_ref(outbuf, &cur_time, 8);
		  memcpy(pkt_buf, outarr, 8);
		  if (RTC->ISR & RTC_ISR_RSF) {
		      // Shadow registers are synchronized and up-to-date
			  *(((uint8_t *)pkt_buf) + 8) = 0x45;
		  } else {
		      // Shadow registers are not yet synchronized
			  *(((uint8_t *)pkt_buf) + 8) = 0x67;
		  }

		  err_t send_err = netconn_sendto(sendudp, outbuf, &debug_addr, 1234);

		  osDelay(1000);
		  netbuf_delete(outbuf);
	  }

	  /*for(;;) {
		  uint64_t ns = get_rtc_time();
		  osDelay(250);
	  }*/
	  /*HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, 0);
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

	  	/*IMU IMU1;
	  	IMU1.hi2c = &hi2c5;
	  	IMU1.I2C_TIMEOUT = 9999;
	  	IMU1.XL_x_offset = 0;
	  	IMU1.XL_y_offset = 0;
	  	IMU1.XL_z_offset = 0;
	  	IMU1.G_x_offset = 0;
	  	IMU1.G_y_offset = 0;
	  	IMU1.G_z_offset = 0;
	  	IMU1.SA0 = 0;

	  	IMU IMU2;
	  	IMU2.hi2c = &hi2c5;
	  	IMU2.I2C_TIMEOUT = 9999;
	  	IMU2.XL_x_offset = 0;
	  	IMU2.XL_y_offset = 0;
	  	IMU2.XL_z_offset = 0;
	  	IMU2.G_x_offset = 0;
	  	IMU2.G_y_offset = 0;
	  	IMU2.G_z_offset = 0;
	  	IMU2.SA0 = 1;

	  	IMU_init(&IMU1);
	  	IMU_init(&IMU2);

	  	for(;;) {
	  		Accel XL_readings1 = {0};
	  		Accel XL_readings2 = {0};
	  		IMU_getAccel(&IMU1, &XL_readings1);
	  		IMU_getAccel(&IMU2, &XL_readings2);

	  		osDelay(1000);
	  	}*/
	  	/*Shift_Reg reg = {0};
	  	reg.VLV_CTR_GPIO_Port = GPIOA;
	  	reg.VLV_CTR_GPIO_Pin = VLV_CTRL_Pin;
	  	reg.VLV_CLK_GPIO_Port = GPIOA;
	  	reg.VLV_CLK_GPIO_Pin = BUFF_CLK_Pin;
	  	reg.VLV_CLR_GPIO_Port = GPIOA;
	  	reg.VLV_CLR_GPIO_Pin = BUFF_CLR_Pin;

	  	Valve VLV1 = {0};
	  	VLV1.VLV_EN_GPIO_Port = VLV1_EN_GPIO_Port;
	  	VLV1.VLV_EN_GPIO_Pin = VLV1_EN_Pin;

	  	VLV_Set_Voltage(reg, 0b00000010);
	  	for (;;) {
	  	    VLV_Toggle(VLV1);
	  	    HAL_GPIO_TogglePin(GPIOE, LED_BLUE_Pin);
	  	    osDelay(3000);
	  	}*/
	  	GPIO_MAX11128_Pinfo adc_pins;
	  	adc_pins.MAX11128_CS_PORT 		= ADC_CS_GPIO_Port;
	  	adc_pins.MAX11128_CS_ADDR 		= ADC_CS_Pin;
	  	adc_pins.HARDWARE_CONFIGURATION = NO_EOC_NOR_CNVST;
	  	adc_pins.NUM_CHANNELS = 16;


	  	init_adc(&hspi4, &adc_pins);
	  	//configure_read_adc_all(&adc_pins);
	  	Shift_Reg reg = {0};
	  	reg.VLV_CTR_GPIO_Port = GPIOA;
	  	reg.VLV_CTR_GPIO_Pin = VLV_CTRL_Pin;
	  	reg.VLV_CLK_GPIO_Port = GPIOA;
	  	reg.VLV_CLK_GPIO_Pin = BUFF_CLK_Pin;
	  	reg.VLV_CLR_GPIO_Port = GPIOA;
	  	reg.VLV_CLR_GPIO_Pin = BUFF_CLR_Pin;

	  	Valve VLV1 = {0};
	  	VLV1.VLV_EN_GPIO_Port = VLV1_EN_GPIO_Port;
	  	VLV1.VLV_EN_GPIO_Pin = VLV1_EN_Pin;
	  	VLV1.VLV_OLD_GPIO_Port = VLV1_OLD_GPIO_Port;
	  	VLV1.VLV_OLD_GPIO_Pin = VLV1_OLD_Pin;
	  	Valve VLV2 = {0};
	  	VLV2.VLV_EN_GPIO_Port = VLV2_EN_GPIO_Port;
	  	VLV2.VLV_EN_GPIO_Pin = VLV2_EN_Pin;

	  	VLV_Set_Conf(reg, 1, VLV_24V, 1, VLV_12V, 0, VLV_12V);
	  	/*for(;;) {
	  		uint16_t adc_values[16] = {0};
	  		read_adc(&hspi4, &adc_pins, adc_values);
	  		uint16_t raw_valve1 = adc_values[1];
	  		float valve_current = (((raw_valve1 / 4095.0) * 3.3) * (5.0/3.0)) / (50 * 0.02);
	  	    VLV_Toggle(VLV1);
	  	    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  	    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	  	    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

	  	    osDelay(20);
	  	    GPIO_PinState open_state = HAL_GPIO_ReadPin(VLV_OLD1_GPIO_Port, VLV_OLD1_Pin);

	  		osDelay(2000);
	  	}*/
	  	ADS_Main_t main_handle = {0};
	  	ADS_TC_t multiTCs[3];

	  	ADS_configTC(&multiTCs[0], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN0_AIN1, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
	  	ADS_configTC(&multiTCs[1], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN2_AIN3, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
	  	ADS_configTC(&multiTCs[2], &hspi2, GPIOB, GPIO_PIN_14, 0xffff, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN0_AIN1, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
	  	ADS_init(&main_handle, multiTCs, 3);

	  	MS5611 bar2;
	  	bar2.hspi = &hspi6;
	  	bar2.SPI_TIMEOUT = 100;
	  	bar2.CS_GPIO_Port = BAR1_CS_GPIO_Port; // PA3
	  	bar2.CS_GPIO_Pin = BAR1_CS_Pin;
	  	bar2.pres_offset = 0;
	  	bar2.alt_offset = 0;

	  	MS5611_PROM_t prom;
	  	prom.constants.C1 = 0;
	  	prom.constants.C2 = 0;
	  	prom.constants.C3 = 0;
	  	prom.constants.C4 = 0;
	  	prom.constants.C5 = 0;
	  	prom.constants.C6 = 0;

	  	MS5611_Reset(&bar2);
	  	MS5611_readPROM(&bar2, &prom);

	  	float pres = 0.0;

	  	for(;;) {
	  		ADS_Reading_t values[3];
	  		int status = ADS_readAllwTimestamps(&main_handle, values);

	  		MS5611_getPres(&bar2, &pres, &prom, OSR_256);

	  		uint16_t adc_values[16] = {0};
	  		read_adc(&hspi4, &adc_pins, adc_values);
	  		uint16_t raw_valve1 = adc_values[1];
	  		float valve_current = (((raw_valve1 / 4095.0) * 3.3) * (5.0/3.0)) / (50 * 0.02);
	  		VLV_Toggle(VLV1);
	  		//VLV_Toggle(VLV2);
	  		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	  		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

	  		osDelay(20);
	  		VLV_OpenLoad open_state = VLV_isOpenLoad(VLV1);

	  		osDelay(1500);
	  	}
	      /*MS5611 baro = {0};
	  	baro.hspi = &hspi6;
	  	baro.CS_GPIO_Pin = BAR2_CS_Pin;
	  	baro.CS_GPIO_Port = BAR2_CS_GPIO_Port;
	  	baro.SPI_TIMEOUT = 1000;
	  	baro.pres_offset = 0;
	  	baro.alt_offset = 0;

	  	MS5611_PROM_t prom = {0};
	  	float pres = 0;

	  	MS5611_Reset(&baro);
	  	MS5611_readPROM(&baro, &prom);
	  	MS5611_getPres(&baro, &pres, OSR_256);*/
	  	for (;;) {
	  		HAL_GPIO_WritePin(VLV1_EN_GPIO_Port, VLV1_EN_Pin, 1);
	  		osDelay(10);
	  		HAL_GPIO_WritePin(VLV1_EN_GPIO_Port, VLV1_EN_Pin, 0);
	  		osDelay(10);
	  	}
  /* USER CODE END setupAndStart */
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
