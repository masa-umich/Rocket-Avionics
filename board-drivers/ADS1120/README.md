# ADS1120 
* [ADS1120 Datasheet](https://www.ti.com/lit/ds/symlink/ads1120.pdf?ts=1729446367184&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1120%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-wwe%2526utm_content%253DADS1120%2526ds_k%253DADS1120+Datasheet%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DEAIaIQobChMIlsjR3sGdiQMVqUb_AR32OS1xEAAYASAAEgIGMvD_BwE%2526gclsrc%253Daw.ds)
* Type: Temperature
* Interface: SPI2
* Driver Requirements:
    * Init
    * Send Command
    * Read Register
    * Write Register
    * Get Temperature
    * Read All TCs and Attach Timestamps

<br/><br/>

# Contents
 - [SPI Setup](#spisetup)
 - [Example Usage](#usage)
 - [Tips](#tips)
 - [Reference](#ref)
    - [Mux Constants](#mux)
    - [Gain Constants](#gain)
    - [Datarate Constants](#rate)
    - Structs
       - [ADS_Main_t](#mainhand)
       - [ADS_TC_t](#tchand)
       - [ADS_Reading_t](#readingst)
       - [ADS_Chip](#chiphand)
    - Functions
       - [ADS_configTC()](#confTC)
       - [ADS_init()](#init)
       - [ADS_readIndividual()](#read1)
       - [ADS_readIndividualwTimestamp()](#read1t)
       - [ADS_readAll()](#readall)
       - [ADS_readAllwTimestamps()](#readallt)
       - [ADS_readInternalTemp()](#readInt)
       - [ADS_convertRawToMicrovolts()](#convertraw)
       - [ADS_polyMicrovoltsToTemp()](#polymtt)
       - [ADS_polyTempToMicrovolts()](#polyttm)
       - [vTCTask()](#task)
       - [ADS_configure()](#configChip)
       - [ADS_configureChip()](#configChipd)

<a id="spisetup"></a>
# SPI Setup

* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First
* Clock Polarity: Low
* Clock Phase: 2 Edge

<a id="usage"></a>
# Example Usage

This example sets up 4 thermocouples connected to 2 chips. Both chips are connected to SPI 1 and the MISO pin is set as pin PA_6.

Must define `getTimestamp()` somewhere earlier. This example sets up the timestamp to be the time since startup in milliseconds:
```c
uint64_t getTimestamp() {
	return HAL_GetTick();
}
```

```c
ADS_Main_t main_handle = {0};
ADS_TC_t multiTCs[4]; // Set up 4 thermocouples

// Configure the 4 thermocouple connections.
ADS_configTC(&multiTCs[0], &hspi1, GPIOA, GPIO_PIN_6, 0xffff, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, ADS_MUX_AIN2_AIN3, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
ADS_configTC(&multiTCs[1], &hspi1, GPIOA, GPIO_PIN_6, 0xffff, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, ADS_MUX_AIN1_AIN0, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
ADS_configTC(&multiTCs[2], &hspi1, GPIOA, GPIO_PIN_6, 0xffff, SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, ADS_MUX_AIN2_AIN3, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);
ADS_configTC(&multiTCs[3], &hspi1, GPIOA, GPIO_PIN_6, 0xffff, SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, ADS_MUX_AIN1_AIN0, ADS_PGA_GAIN_1, ADS_DATA_RATE_20);

// Initialize and start the RTOS task
ADS_init(&main_handle, multiTCs, 4);

// Loop indefinitely, reading from every thermocouple twice a second
for(;;) {
    ADS_Reading_t readings[4];

    int status = ADS_readAllwTimestamps(&main_handle, readings);
    if(!status) {
        // Iterate through the readings. The order will match the order of multiTCs.
        for(int i = 0;i < 4;i++) {
            if(!readings[i].error) {
                // Do something with readings[i].temp_c and readings[i].timestamp
                // Again, the thermocouple that this corresponds to is whichever is index i in multiTCs
            }
            else {
                // Failed to retrieve this thermocouple temperature
            }
        }
    }
    else {
        // Failed to retrieve internal temperature, all readings are invalid
    }

	osDelay(500);
}
```

<a id="tips"></a>
## Tips

### Picking a datarate/polling rate:
Unfortunately, since the ADS1120 cannot reading from multiple connects at once, this driver has to switch between the connected thermocouples on each chip, causing a lot of delay in between readings. This means that the configured datarate for each thermocouple is not actually how often the reading gets updated in the driver. I'd recommend using a datarate of a little over 2x your desired polling rate. For example, if you need a polling rate of 50Hz, use a datarate of 90Hz if the polling rate isn't super important, 175Hz if the polling rate is very important.

### Understand errors:
If a read function fails to retrieve a reading, it doesn't mean that a new reading isn't ready. Also, you can call the read functions as often as you want, 
if a new reading isn't ready it just returns the previous reading. Basically, whether or not a read function fails has nothing to do with if data is ready on the ADS1120 side.

<a id="ref"></a>
# Reference

<a id="mux"></a>
## Mux Constants
`ADS_MUX_AIN0_AIN1` - Positive lead connected to AIN0, negative lead connected to AIN1

`ADS_MUX_AIN0_AIN2` - Positive lead connected to AIN0, negative lead connected to AIN2

`ADS_MUX_AIN0_AIN3` - Positive lead connected to AIN0, negative lead connected to AIN3

`ADS_MUX_AIN1_AIN2` - Positive lead connected to AIN1, negative lead connected to AIN2

`ADS_MUX_AIN1_AIN3` - Positive lead connected to AIN1, negative lead connected to AIN3

`ADS_MUX_AIN2_AIN3` - Positive lead connected to AIN2, negative lead connected to AIN3

`ADS_MUX_AIN1_AIN0` - Positive lead connected to AIN1, negative lead connected to AIN0

`ADS_MUX_AIN3_AIN2` - Positive lead connected to AIN3, negative lead connected to AIN2

<a id="gain"></a>
## Gain Constants
`ADS_PGA_GAIN_1` - 1x Gain

`ADS_PGA_GAIN_2` - 2x Gain

`ADS_PGA_GAIN_4` - 4x Gain

`ADS_PGA_GAIN_8` - 8x Gain

`ADS_PGA_GAIN_16` - 16x Gain

`ADS_PGA_GAIN_32` - 32x Gain

`ADS_PGA_GAIN_64` - 64x Gain

`ADS_PGA_GAIN_128` - 128x Gain

<a id="rate"></a>
## Datarate Constants
`ADS_DATA_RATE_20` - 20Hz Polling rate

`ADS_DATA_RATE_45` - 45Hz Polling rate

`ADS_DATA_RATE_90` - 90Hz Polling rate

`ADS_DATA_RATE_175` - 175Hz Polling rate

`ADS_DATA_RATE_330` - 330Hz Polling rate

`ADS_DATA_RATE_600` - 600Hz Polling rate

`ADS_DATA_RATE_1000` - 1000Hz Polling rate


<a id="mainhand"></a>
## struct ADS_Main_t

ADS_Main_t is the main handle for everything.
It stores the RTOS task handle for the task that collects thermocouple measurements, pointers to all of the individual thermocouple connections, and the current cold junction temperature.


```c
typedef struct {
	TaskHandle_t tc_task;
	ADS_TC_t *TCs;
	uint8_t TC_count;


	int16_t raw_temp;
	SemaphoreHandle_t temp_semaphore;
	uint32_t last_temp;
} ADS_Main_t;
```

<a id="tchand"></a>
## struct ADS_TC_t

ADS_TC_t stores information about a specific thermocouple connection.
This includes information about the specific ADS1120 chip that the thermocouple is connected to (SPI bus, CS pin), which ADS1120 "channel" it's connected to, and configuration information like the gain and datarate.
It also stores the latest reading and associated timestamp from the thermocouple.

```c
typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef *DOUT_GPIO_Port;
	uint16_t DOUT_GPIO_Pin;
	uint16_t SPI_TIMEOUT;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_GPIO_Pin;
	uint8_t mux;

	uint8_t gain;
	uint8_t rate;
	double step_size;

	int16_t current_raw;
	uint64_t timestamp;
	SemaphoreHandle_t reading_semaphore;
} ADS_TC_t;
```

<a id="readingst"></a>
## struct ADS_Reading_t

ADS_Reading_t packages a single thermocouple reading along with the timestamp when it was read, and an error flag.

`temp_c` is the cold junction compensated temperature in Celsius timestamp is the time when the reading was received from the ADS1120.
It is in whatever format was given by the external function `getTimestamp()`.

If `error` is 0, then `temp_c` and `timestamp` are valid, otherwise there was an issue reading the latest measurement - `temp_c` and `timestamp` will not be valid and could be anything.

```c
typedef struct {
	float temp_c;
	uint64_t timestamp;
	uint8_t error;
} ADS_Reading_t;
```

**Fields:**
 - `temp_c` - The temperature of this reading in Celsius. This is cold junction compensated
 - `timestamp` - The timestamp of this reading. This timestamp corresponds to the time when the reading is received from the ADS1120
 - `error` - The error flag. If this is not 0, then both `temp_c` and `timestamp` are not valid

<a id="chiphand"></a>
## struct ADS_Chip

ADS_Chip contains information for each ADS1120 chip.
It stores SPI bus information, pointers to the thermocouples that are connected to the chip, and timing information related to reading from the chip and switching between thermocouple connections.


```c
typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef *DOUT_GPIO_Port;
	uint16_t DOUT_GPIO_Pin;
	uint16_t SPI_TIMEOUT;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_GPIO_Pin;

	ADS_TC_t* muxes[2];
	uint8_t last_state;
	uint8_t mux_count;
	uint8_t current_mux;
	uint32_t last_tick;
	uint32_t timer_start;
} ADS_Chip;
```

<a id="confTC"></a>
## void ADS_configTC(ADS_TC_t *ADSTC, SPI_HandleTypeDef *hspi, GPIO_TypeDef *DOUT_GPIO_Port, uint16_t DOUT_GPIO_Pin, uint16_t SPI_TIMEOUT, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t mux, uint8_t gain, uint8_t rate)

Configure a thermocouple with connection information, gain, and datarate.

The DOUT (MISO) pin of the SPI bus is needed since it's used as a GPIO pin 
between SPI communications to determine if data is ready.

NOTE: this does not actually communicate with the chip, it just sets up the struct.

```c
void ADS_configTC(ADS_TC_t *ADSTC, SPI_HandleTypeDef *hspi, GPIO_TypeDef *DOUT_GPIO_Port, uint16_t DOUT_GPIO_Pin, uint16_t SPI_TIMEOUT, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t mux, uint8_t gain, uint8_t rate)
```

**Params:**

- `ADSTC` - The thermocouple to configure
- `hspi` - The SPI handle that the corresponding ADS1120 is connected to
- `DOUT_GPIO_Port` - The GPIO port that the MISO pin of the SPI bus is connected to
- `DOUT_GPIO_Pin` - The GPIO pin that the MISO pin of the SPI bus is connected to
- `SPI_TIMEOUT` - The timeout to use for SPI communications (milliseconds)
- `CS_GPIO_Port` - The GPIO port that the chip select pin of the ADS1120 is connected to
- `CS_GPIO_Pin` - The GPIO pin that the chip select pin of the ADS1120 is connected to
- `mux` - The combination of pins on the ADS1120 that the thermocouple is connected to.
- `gain` - The gain to use for this thermocouple
- `rate` - The datarate to use for this thermocouple

<a id="init"></a>
## int ADS_init(ADS_Main_t *ADSMain, ADS_TC_t *TCs, uint8_t num_TCs)

Configure the main struct and initialize the RTOS task.


```c
int ADS_init(ADS_Main_t *ADSMain, ADS_TC_t *TCs, uint8_t num_TCs)
```

**Params:**

- `ADSMain` - The main handle where the RTOS task handle and all thermocouple pointers should be stored
- `TCs` - The thermocouples to be monitored by the RTOS task. This should be a pointer to an array of thermocouples
- `num_TCs` - The number of thermocouples to be monitored by the RTOS task

**Returns:**

- The status of the created RTOS task. Returns 0 if the task started successfully, returns 1 if the task failed to start

<a id="read1"></a>
## int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading)

Get the latest temperature of one thermocouple.
This reading is cold junction compensated using one of the connected ADS1120s.


```c
int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading)
```

**Params:**

- `ADSMain` - The main driver handle
- `TC_index` - The index of the thermocouple to be read. This is 0 indexed and is based on the order of the array that was originally passed to `ADS_init`
- `reading` - The pointer to where the reading should be stored. This is a temperature measurement in Celsius

**Returns:**

- Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved

<a id="read1t"></a>
## int ADS_readIndividualwTimestamp(ADS_Main_t *ADSMain, uint8_t TC_index, ADS_Reading_t *reading)

Get the latest temperature of one thermocouple, including the timestamp when it was collected.
This reading is cold junction compensated using one of the connected ADS1120s.

The error flag in the passed ADS_Reading_t reflects the success of this function, and either it or the return value can be used to determine whether or not the function successfully retrieved the reading.

The timestamp of the reading is taken when the reading is received from the ADS1120 NOT when this function is called. This means it can be 
used to determine if everything is working and readings are being updated. For example, if the datarate is set at 20Hz, and the timestamp is significantly greater than 50ms in the past, the reading is probably not getting updated.


```c
int ADS_readIndividualwTimestamp(ADS_Main_t *ADSMain, uint8_t TC_index, ADS_Reading_t *reading)
```

**Params:**

- `ADSMain` - The main driver handle
- `TC_index` - The index of the thermocouple to be read. This is 0 indexed and is based on the order of the array that was originally passed to `ADS_init`
- `reading` - The pointer to where the reading should be stored. This is a temperature measurement in Celsius

**Returns:**

- Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved

<a id="readall"></a>
## int ADS_readAll(ADS_Main_t *ADSMain, float *readings)

Get the latest temperature of all thermocouples.
These readings are cold junction compensated using one of the connected ADS1120s.

Readings are in the order that their corresponding thermocouple was passed to `ADS_init`.

If a reading couldn't be retrieved, its reading will be NAN.
Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect if they weren't successful (NAN). If this function returns 1, then all readings are not valid.


```c
int ADS_readAll(ADS_Main_t *ADSMain, float *readings)
```

**Params:**

- `ADSMain` - The main driver handle
- `readings` - The pointer to the array where the readings should be stored. This should be at least as long as the number of thermocouples that were passed to `ADS_init`

**Returns:**

- Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be retrieved

<a id="readallt"></a>
## int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings)

Get the latest temperature of all thermocouples.
These readings are cold junction compensated using one of the connected ADS1120s.

Readings are in the order that their corresponding thermocouple was passed to `ADS_init`.

If a reading couldn't be retrieved, its error flag will be 1.
Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect if they weren't successful (error flag = 1). If this function returns 1, then all readings are not valid, although their error flags might not reflect this.

Just as in `ADS_readIndividualwTimestamp()`, the timestamps are taken when the readings are received from the ADS1120s, so they can be used to determine if readings are being updated.


```c
int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings)
```

**Params:**

- `ADSMain` - The main driver handle
- `readings` - The pointer to the array where the readings and timestamps should be stored. This should be at least as long as the number of thermocouples that were passed to `ADS_init`

**Returns:**

- Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be retrieved

<a id="readInt"></a>
## int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp)

Get the latest internal temperature from one of the ADS1120s.
By default this is updated twice a second, but it can be changed by changing the `ADS_INTERNAL_TEMP_DELAY` macro.
This is the temperature used for cold junction compensation in all read functions. The specific ADS1120 that this is retrieved from is whichever ADS1120 is connected to the first thermocouple passed to `ADS_init`.


```c
int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp)
```

**Params:**

- `ADSMain` - The main driver handle
- `temp` - The pointer to where the temperature should be stored. This temperature is in Celsius

**Returns:**

- Returns 0 if retrieving the temperature was successful, returns 1 if it could not be retrieved

<a id="convertraw"></a>
## double ADS_convertRawToMicrovolts(ADS_TC_t *ADS, int16_t raw)

Convert a raw 16 bit value from a ADS1120 to a microvolt reading.


```c
double ADS_convertRawToMicrovolts(ADS_TC_t *ADS, int16_t raw)
```

**Params:**

- `ADS` - The thermocouple that this reading came from
- `raw` - The 16 bit value from the ADS1120

**Returns:**

- The voltage measured by the ADS1120 in microvolts

<a id="polymtt"></a>
## float ADS_polyMicrovoltsToTemp(double microvolts)

Convert a voltage of a type T thermocouple to its corresponding temperature.


```c
float ADS_polyMicrovoltsToTemp(double microvolts)
```

**Params:**

- `microvolts` - The voltage of the thermocouple in microvolts

**Returns:**

- The calculated temperature of the thermocouple in Celsius

<a id="polyttm"></a>
## double ADS_polyTempToMicrovolts(float temp)

Convert the temperature of a type T thermocouple to the voltage it would read.


```c
double ADS_polyTempToMicrovolts(float temp)
```

**Params:**

- `temp` - The temperature of the thermocouple in Celsius

**Returns:**

- The calculated voltage in microvolts that the thermocouple should generate if it was at the given temperature

<a id="task"></a>
## void vTCTask(void *pvParameters)

The main function for the RTOS task.
Collects and stores readings from all thermocouples, including a cold junction temperature from one of the connected ADS1120s.


```c
void vTCTask(void *pvParameters)
```

**Params:**

- `pvParameters` - The pointer to the main ADS_Main_t handle

<a id="configChip"></a>
## int ADS_configure(ADS_Chip *ADS, uint8_t TC_index, int check)

Configure an ADS1120 to read from one of its connected thermocouples.
This can also be used to configure an ADS1120 to read from its internal temperature sensor.


```c
int ADS_configure(ADS_Chip *ADS, uint8_t TC_index, int check)
```

**Params:**

- `ADS` - The specific ADS1120 chip to configure
- `TC_index` - The index of the thermocouple to set up the ADS1120 to read from. This is specific to how the ADS chip struct was set up, and since there can be up to 2 thermocouples connected to each chip, this parameter can only be 0 or 1. If this parameter is 2, the ADS1120 is configured to read from its internal temperature sensor
- `check` - If this is 1, the configuration of the ADS1120 will be checked after it's written to make sure the new configuration was saved. If this is 0, no verification is done

**Returns:**

- The result of the verification. Returns 0 if the verification passes and the configuration on the ADS1120 matches the passed configuration and returns 1 if the configuration fails and the ADS1120 configuration remains the same as before this function was called. If check is 0, then this always returns 0

<a id="configChipd"></a>
## int ADS_configureChip(ADS_Chip *ADS, int check)

Configure an ADS1120 chip with the default configuration


```c
int ADS_configureChip(ADS_Chip *ADS, int check)
```

**Params:**

- `ADS` - The specific ADS1120 chip to configure
- `check` - If this is 1, the configuration of the ADS1120 will be checked after it's written to make sure the new configuration was saved. If this is 0, no verification is done

**Returns:**

- The result of the verification. Returns 0 if the verification passes and the configuration on the ADS1120 matches the passed configuration and returns 1 if the configuration fails and the ADS1120 configuration remains the same as before this function was called. If check is 0, then this always returns 0