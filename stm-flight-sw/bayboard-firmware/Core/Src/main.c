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
#include "api.h"
#include "string.h"
#include "client.h"
#include "messages.h"
#include "M24256E.h"
#include "utils.h"
#include "lmp_channels.h"
#include "lwip/udp.h"
#include "timers.h"
#include "log_errors.h"
#include "queue.h"
#include "eeprom-config.h"
#include "telemetry.h"
#include "network-processing.h"
#include "board-state.h"
#include "logging.h"
#include "time-sync.h"
//#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

/* Definitions for startTask */
osThreadId_t startTaskHandle;
const osThreadAttr_t startTask_attributes = {
  .name = "startTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
extern struct netif gnetif;

ip4_addr_t bb_addr;
uint8_t bb_num;

EEPROM_conf_t loaded_config = {0};
PT_t PT1_h = {0.5f, 5000.0f, 4.5f}; // Default
PT_t PT2_h = {0.5f, 5000.0f, 4.5f};
PT_t PT3_h = {0.5f, 5000.0f, 4.5f};
PT_t PT4_h = {0.5f, 5000.0f, 4.5f};
PT_t PT5_h = {0.5f, 5000.0f, 4.5f};
PT_t PT6_h = {0.5f, 5000.0f, 4.5f};
PT_t PT7_h = {0.5f, 5000.0f, 4.5f};
PT_t PT8_h = {0.5f, 5000.0f, 4.5f};
PT_t PT9_h = {0.5f, 5000.0f, 4.5f};
PT_t PT10_h = {0.5f, 5000.0f, 4.5f};

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
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI6_Init(void);
void StartAndMonitor(void *argument);

/* USER CODE BEGIN PFP */
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
  MX_RTC_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_I2C5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  // Create RTC mutex
  timesync_setup(&hrtc);
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
  loaded_config.pt6 = &PT6_h;
  loaded_config.pt7 = &PT7_h;
  loaded_config.pt8 = &PT8_h;
  loaded_config.pt9 = &PT9_h;
  loaded_config.pt10 = &PT10_h;
  if(setup_eeprom(&hi2c1, EEPROM_WC_GPIO_Port, EEPROM_WC_Pin)) {
#ifdef EEPROM_OVERRIDE
	  load_eeprom_defaults(&loaded_config, &bb_num);
	  log_message(STAT_EEPROM_DEFAULT_LOADED, -1);
#else
	  switch(load_eeprom_config(&loaded_config, &bb_num)) {
	  	  case -1: {
	  		  // eeprom load error, defaults loaded
	  		  load_eeprom_defaults(&loaded_config, &bb_num);
	  		  log_message(ERR_EEPROM_LOAD_COMM_ERR, -1);
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -2: {
	  		  // eeprom tc gain value error
	  		  log_message(ERR_EEPROM_LOAD_TC_ERR, -1);
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -3: {
	  		  // eeprom valve conf error
	  		  log_message(ERR_EEPROM_LOAD_VLV_ERR, -1);
	  		  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
	  		  break;
	  	  }
	  	  case -4: {
	  		  // invalid bb number
	  		  log_message(BB_ERR_EEPROM_LOAD_BB_NUM_ERR, -1);
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
	  load_eeprom_defaults(&loaded_config, &bb_num);
	  log_message(ERR_EEPROM_INIT, -1);
	  timers.buzzTimer = xTimerCreate("buzz", 500, pdTRUE, NULL, toggleBuzzer);
  }
  bb_addr = loaded_config.bayboardIP;
  char logmsg[sizeof(BB_STAT_STARTING_IDENTIFY) + 1];
  snprintf(logmsg, sizeof(logmsg), BB_STAT_STARTING_IDENTIFY "%d", bb_num);
  log_message(logmsg, -1);

  log_message(STAT_VERSION_INFO, -1);

  // Init flash
  if(init_flash_logging(&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin)) {
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ADC1_CS_Pin|ADC2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BAR2_CS_Pin|BAR1_CS_Pin|TC1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ETH_NRST_Pin|TC2_CS_Pin|EEPROM_WC_Pin|TC3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, VLV5_EN_Pin|VLV4_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VLV3_EN_Pin|VLV2_EN_Pin|VLV1_EN_Pin|BUZZ_Pin
                          |LED_BLUE_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUFF_CLK_Pin|VLV_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, HBRIDGE_EN_Pin|HBRIDGE_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC1_CS_Pin ADC2_CS_Pin HBRIDGE_EN_Pin HBRIDGE_DIR_Pin */
  GPIO_InitStruct.Pin = ADC1_CS_Pin|ADC2_CS_Pin|HBRIDGE_EN_Pin|HBRIDGE_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CS_Pin */
  GPIO_InitStruct.Pin = ENCODER_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENCODER_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : BAR2_CS_Pin BAR1_CS_Pin TC1_CS_Pin */
  GPIO_InitStruct.Pin = BAR2_CS_Pin|BAR1_CS_Pin|TC1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ETH_NRST_Pin TC2_CS_Pin EEPROM_WC_Pin TC3_CS_Pin */
  GPIO_InitStruct.Pin = ETH_NRST_Pin|TC2_CS_Pin|EEPROM_WC_Pin|TC3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : VLV5_OLD_Pin VLV4_OLD_Pin VLV3_OLD_Pin VLV2_OLD_Pin
                           IMU1_INT1_Pin IMU1_INT2_Pin IMU2_INT1_Pin IMU2_INT2_Pin */
  GPIO_InitStruct.Pin = VLV5_OLD_Pin|VLV4_OLD_Pin|VLV3_OLD_Pin|VLV2_OLD_Pin
                          |IMU1_INT1_Pin|IMU1_INT2_Pin|IMU2_INT1_Pin|IMU2_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VLV1_OLD_Pin */
  GPIO_InitStruct.Pin = VLV1_OLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VLV1_OLD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VLV5_EN_Pin VLV4_EN_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = VLV5_EN_Pin|VLV4_EN_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : VLV3_EN_Pin VLV2_EN_Pin VLV1_EN_Pin BUZZ_Pin
                           LED_BLUE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = VLV3_EN_Pin|VLV2_EN_Pin|VLV1_EN_Pin|BUZZ_Pin
                          |LED_BLUE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUFF_CLK_Pin VLV_CTRL_Pin */
  GPIO_InitStruct.Pin = BUFF_CLK_Pin|VLV_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

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

    setup_system_state();

	// Config SR
  	Shift_Reg reg = {0};
  	reg.VLV_CTR_GPIO_Port = VLV_CTRL_GPIO_Port;
  	reg.VLV_CTR_GPIO_Pin = VLV_CTRL_Pin;
  	reg.VLV_CLK_GPIO_Port = BUFF_CLK_GPIO_Port;
  	reg.VLV_CLK_GPIO_Pin = BUFF_CLK_Pin;

  	VLV_Set_Conf(reg, loaded_config.vlv1_en, loaded_config.vlv1_v,
  				loaded_config.vlv2_en, loaded_config.vlv2_v,
				loaded_config.vlv3_en, loaded_config.vlv3_v,
				loaded_config.vlv4_en, loaded_config.vlv4_v,
				loaded_config.vlv5_en, loaded_config.vlv5_v);

  	// Load valve pins
  	setup_valve(0, VLV1_EN_GPIO_Port, VLV1_EN_Pin, VLV1_OLD_GPIO_Port, VLV1_OLD_Pin);
  	setup_valve(1, VLV2_EN_GPIO_Port, VLV2_EN_Pin, VLV2_OLD_GPIO_Port, VLV2_OLD_Pin);
  	setup_valve(2, VLV3_EN_GPIO_Port, VLV3_EN_Pin, VLV3_OLD_GPIO_Port, VLV3_OLD_Pin);
  	setup_valve(3, VLV4_EN_GPIO_Port, VLV4_EN_Pin, VLV4_OLD_GPIO_Port, VLV4_OLD_Pin);
  	setup_valve(4, VLV5_EN_GPIO_Port, VLV5_EN_Pin, VLV5_OLD_GPIO_Port, VLV5_OLD_Pin);

	// Init sensors and peripherals
  	// ADC
  	sensors_h.adc1_h.MAX11128_CS_PORT = ADC1_CS_GPIO_Port;
  	sensors_h.adc1_h.MAX11128_CS_ADDR = ADC1_CS_Pin;
  	sensors_h.adc1_h.HARDWARE_CONFIGURATION = NO_EOC_NOR_CNVST;
  	sensors_h.adc1_h.NUM_CHANNELS = 16;

  	sensors_h.adc2_h.MAX11128_CS_PORT = ADC2_CS_GPIO_Port;
  	sensors_h.adc2_h.MAX11128_CS_ADDR = ADC2_CS_Pin;
  	sensors_h.adc2_h.HARDWARE_CONFIGURATION = NO_EOC_NOR_CNVST;
  	sensors_h.adc2_h.NUM_CHANNELS = 16;

  	init_adc(&hspi4, &(sensors_h.adc1_h));
  	// Check to make sure the chip is connected
	uint16_t adc_values[16] = {0};
	read_adc(&hspi4, &(sensors_h.adc1_h), adc_values);
	if(adc_values[ADC1_3V3_BUS_I] == 0) { // 3v3 bus voltage should always be greater than 0
		// ADC1 error
		log_message(BB_ERR_ADC1_INIT, -1);
	}

  	init_adc(&hspi4, &(sensors_h.adc2_h));

  	// TC ADCs
  	ADS_configTC(&(sensors_h.TCs[0]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC3_CS_GPIO_Port, TC3_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc1_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[1]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC3_CS_GPIO_Port, TC3_CS_Pin, ADS_MUX_AIN3_AIN2, loaded_config.tc2_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[2]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc3_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[3]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC2_CS_GPIO_Port, TC2_CS_Pin, ADS_MUX_AIN3_AIN2, loaded_config.tc4_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[4]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN0_AIN1, loaded_config.tc5_gain, ADS_DATA_RATE_600);
  	ADS_configTC(&(sensors_h.TCs[5]), &hspi2, GPIOB, GPIO_PIN_14, PERIPHERAL_TIMEOUT, TC1_CS_GPIO_Port, TC1_CS_Pin, ADS_MUX_AIN3_AIN2, loaded_config.tc6_gain, ADS_DATA_RATE_600);

  	if(ADS_init(&(sensors_h.tc_main_h), sensors_h.TCs, NUM_TCS)) {
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

  	// Start TCP client
  	switch(client_init(&loaded_config.flightcomputerIP, netif_is_link_up(&gnetif))) {
  		case 2: {
  			// threads didn't start
  			log_message(BB_ERR_TCP_CLIENT_INIT_THREAD, -1);
  			break;
  		}
  		case 1: {
  			// memory error
  			log_message(BB_ERR_TCP_CLIENT_INIT_MEM_ERR, -1);
  			break;
  		}
  		default: {
  			// client initialized
  			log_message(BB_STAT_TCP_CLIENT_INIT, -1);
  			break;
  		}
  	}

	// log startup done
	log_message(STAT_STARTUP_DONE, -1);

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

	uint8_t telemcounter = 0;
	uint8_t statecounter = 0;

	uint8_t fcconnected = 0;

	uint32_t startTick = HAL_GetTick();

	/* Infinite loop */
	for(;;) {
		handle_logging(); // Log to UDP and flash

		if(HAL_GetTick() - startTick > 250) {
			startTick = HAL_GetTick();
			//size_t freemem = xPortGetFreeHeapSize();
			refresh_log_timers();

			int constat = is_client_connected();
			if(constat > -1) {
				if(fcconnected == 0 && constat == 1) {
					uint64_t valvetime = get_rtc_time();
			  	  	if(xSemaphoreTake(Board_h.bbValve_access, 5) == pdPASS) {
			  	  		Valve_State_t vstates[NUM_VALVE_CHS];
						for(int i = 0;i < NUM_VALVE_CHS;i++) {
							vstates[i] = Board_h.bbValveStates[i];
						}
			  	  		xSemaphoreGive(Board_h.bbValve_access);
						for(int i = 0;i < NUM_VALVE_CHS;i++) {
							Message statemsg = {0};
							statemsg.type = MSG_VALVE_STATE;
							statemsg.data.valve_state.timestamp = valvetime;
							statemsg.data.valve_state.valve_id = generate_valve_id(bb_num, i);
							statemsg.data.valve_state.valve_state = vstates[i];
							send_msg_to_device(&statemsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5);
						}
			  	  	}
				}
				fcconnected = constat;
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
