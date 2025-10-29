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
#include "lwip/udp.h"
#include "timers.h"
#include "log_errors.h"
#include "semphr.h"
#include "time-sync.h"
#include "logging.h"
//#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
extern struct netif gnetif;

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

// use_bat 1 to use battery source, 0 to use GSE
void PDB_source(uint8_t use_bat) {
	HAL_GPIO_WritePin(PDB_DIO1_GPIO_Port, PDB_DIO1_Pin, use_bat);
	if(use_bat) {
		log_message(FC_STAT_PDB_SWITCH_BAT, -1);
	}
	else {
		log_message(FC_STAT_PDB_SWITCH_GSE, -1);
	}
}

void COTS_supply(uint8_t enabled) {
	HAL_GPIO_WritePin(PDB_DIO2_GPIO_Port, PDB_DIO2_Pin, enabled);
	if(enabled) {
		log_message(FC_STAT_PDB_COTS_SUPPLY_ON, -1);
	}
	else {
		log_message(FC_STAT_PDB_COTS_SUPPLY_OFF, -1);
	}
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
// IMPORTANT: you are responsible for freeing the buffer in msg if this function returns something other than 0. This function does not
// take "ownership" of the message object
int send_raw_msg_to_device(Target_Device device, Raw_message *msg, TickType_t wait) {
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int devicefd = get_device_fd(device);
	if(devicefd < 0) {
		return -3; // Device not connected
	}
	msg->connection_fd = devicefd;
	return server_send(msg, wait);
}

void *ftp_open(const char *fname, const char *mode, u8_t write) {
	if(strcmp(fname, "eeprom.bin") == 0) {
		prepare_eeprom_config();
		return (void *) 1; // 1 refers to eeprom
	}
	else if(strcmp(fname, "messages.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) FLASH_MSG_MARK; // FLASH_MSG_MARK refers to status/error messages. Only reading is valid
	}
	else if(strcmp(fname, "telemetry.bin") == 0) {
		if(!write) {
			prepare_flash_dump();
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
		close_and_validate_config(&hcrc);
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		finish_flash_dump();
	}
}

int ftp_read(void *handle, void *buf, int bytes) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		return eeprom_config_dump(buf, bytes);
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		return dump_flash(fd, buf, bytes);
	}
	else {
		return -1;
	}
}

int ftp_write(void *handle, struct pbuf *p) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		return eeprom_config_write(p);
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
  timesync_setup();
  logging_setup();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  // This next section is considered the "critical" portion of initialization. This portion has no access to logging and therefore needs to communicate status other ways. The section ends on the return of the MX_LWIP_Init function
  // The end of this section is marked by a UDP message and/or the on-board LED becoming solid and/or a short buzzer beep

  inittimers_t timers = {0};

  // Load EEPROM config
  loaded_config.pt1 = &PT1_h;
  loaded_config.pt2 = &PT2_h;
  loaded_config.pt3 = &PT3_h;
  loaded_config.pt4 = &PT4_h;
  loaded_config.pt5 = &PT5_h;
  if(setup_eeprom(&hi2c1, EEPROM_WC_GPIO_Port, EEPROM_WC_Pin)) {
#ifdef EEPROM_OVERRIDE
	  load_eeprom_defaults(&loaded_config);
	  log_message(STAT_EEPROM_DEFAULT_LOADED, -1);
#else
	  switch(load_eeprom_config(&eeprom_h, &loaded_config)) {
	  	  case -1: {
	  		  // eeprom load error, defaults loaded
	  		  log_message(ERR_EEPROM_LOAD_COMM_ERR, -1);
	  		  load_eeprom_defaults(&loaded_config);
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -2: {
	  		  // eeprom tc gain value error
	  		  log_message(ERR_EEPROM_LOAD_TC_ERR, -1);
	  		  loaded_config.tc1_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  loaded_config.tc2_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  loaded_config.tc3_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -3: {
	  		  // eeprom valve conf error
	  		  log_message(ERR_EEPROM_LOAD_VLV_ERR, -1);
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
	  		  log_message(STAT_EEPROM_LOADED, -1);
	  		  break;
	  	  }
	  }
#endif
  }
  else {
	  // eeprom init error, defaults loaded
	  log_message(ERR_EEPROM_INIT, -1);
	  load_eeprom_defaults(&loaded_config);
	  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
  }
  fc_addr = loaded_config.flightcomputerIP;

  // Init flash
  if(init_flash_logging(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_GPIO_Pin)) {
	  timers.ledTimer = xTimerCreate("led", 500, pdTRUE, NULL, toggleLEDPins);
  }
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
		int read_stat = server_read(&msg, 50);
		if(read_stat >= 0) {
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

				    	if(send_raw_msg_to_device(LimeWire_d, &msg, 2) == 0) {
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
				    			case DEVICE_CMD_PDB_SRC_GSE: {
				    				PDB_source(0);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_SRC_BAT: {
				    				PDB_source(1);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_OFF: {
				    				COTS_supply(0);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_ON: {
				    				COTS_supply(1);
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
    	handle_flash_clearing();
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

	if(is_net_logging_up()) {
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

	setup_system_state();

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
  	setup_valve(0, VLV1_EN_GPIO_Port, VLV1_EN_Pin, VLV1_OLD_GPIO_Port, VLV1_OLD_Pin);
  	setup_valve(1, VLV2_EN_GPIO_Port, VLV2_EN_Pin, VLV2_OLD_GPIO_Port, VLV2_OLD_Pin);
  	setup_valve(2, VLV3_EN_GPIO_Port, VLV3_EN_Pin, VLV3_OLD_GPIO_Port, VLV3_OLD_Pin);

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
  	ADS_configTC(&(sensors_h.TCs[0]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc1_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[1]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN2_AIN3, loaded_config.tc2_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[2]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc3_gain, ADS_DATA_RATE_600);

  	if(ADS_init(&(sensors_h.tc_main_h), sensors_h.TCs, 3)) {
  		// ADS thread start error
  		log_message(ERR_ADS_INIT_THREAD_ERR, -1);
  	}

  	// IMUs
  	sensors_h.imu1_h.hi2c = &hi2c5;
  	sensors_h.imu1_h.I2C_TIMEOUT = PERIPHERAL_TIMEOUT;
  	sensors_h.imu1_h.XL_x_offset = 0;
  	sensors_h.imu1_h.XL_y_offset = 0;
  	sensors_h.imu1_h.XL_z_offset = 0;
  	sensors_h.imu1_h.G_x_offset = 0;
  	sensors_h.imu1_h.G_y_offset = 0;
  	sensors_h.imu1_h.G_z_offset = 0;
  	sensors_h.imu1_h.SA0 = 0;

  	sensors_h.imu2_h.hi2c = &hi2c5;
  	sensors_h.imu2_h.I2C_TIMEOUT = PERIPHERAL_TIMEOUT;
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
  	sensors_h.bar1_h.SPI_TIMEOUT = PERIPHERAL_TIMEOUT;
  	sensors_h.bar1_h.CS_GPIO_Port = BAR1_CS_GPIO_Port;
  	sensors_h.bar1_h.CS_GPIO_Pin = BAR1_CS_Pin;
  	sensors_h.bar1_h.pres_offset = 0;
  	sensors_h.bar1_h.alt_offset = 0;

  	sensors_h.bar2_h.hspi = &hspi6;
  	sensors_h.bar2_h.SPI_TIMEOUT = PERIPHERAL_TIMEOUT;
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

	/*
	 * ALL VALVE STATES SHOULD BE SENT ON THE START OF CONNECTION WITH LIMEWIRE (FC FOR BBs) (WHEN THE FC CONNECTS TO LIMEWIRE SEND THE VALVE STATES FOR THE ENTIRE ROCKET)
	 *
	 * fc telemetry comes from struct, sent on a 50hz loop
	 * bb telemetry comes from tcp packets, relayed to limewire and stored for optional access in autosequences
	 * fc valve states are sent after valve commands are sent, then stored for optional access
	 * bb valve states are relayed from tcp packets, and saved for optional access
	 * all valve commands are processed when received. fc relays commands from limewire to bb
	 *
	 * telemetry task - read from peripherals and send telemetry msg
	 * tcp process task - process incoming packets for relaying or valve stuff
	 */

	uint8_t telemcounter = 0;
	uint8_t statecounter = 0;

	uint8_t limewireconnected = 0;

	uint32_t startTick = HAL_GetTick();
	/* Infinite loop */
	for(;;) {
		handle_logging();

		// Send all valve states on connection with limewire
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
				log_telemetry();
				telemcounter = 0; // 4hz
			}
			else {
				telemcounter--;
			}

			if(statecounter == 0) {
				log_valve_states();
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
