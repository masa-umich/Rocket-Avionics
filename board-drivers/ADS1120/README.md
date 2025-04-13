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


# Example Usage

# Tips

# Reference

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

## int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading)

Get the latest temperature of one thermocouple.
This reading is cold junction compensated using one of the connected ADS1120s.


```c
int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading)
```

**Params:**

- `ADSMain` - The main driver handle
- `TC_index` - The index of the thermocouple to be read. This is 0 indexed and is based on the order of the array that was originally passed to ADS_init
- `reading` - The pointer to where the reading should be stored. This is a temperature measurement in Celsius

**Returns:**

- Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved

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
- `TC_index` - The index of the thermocouple to be read. This is 0 indexed and is based on the order of the array that was originally passed to ADS_init
- `reading` - The pointer to where the reading should be stored. This is a temperature measurement in Celsius

**Returns:**

- Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved

## int ADS_readAll(ADS_Main_t *ADSMain, float *readings)

Get the latest temperature of all thermocouples.
These readings are cold junction compensated using one of the connected ADS1120s.

Readings are in the order that their corresponding thermocouple was passed to ADS_init.

If a reading couldn't be retrieved, its reading will be NAN.
Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect if they weren't successful (NAN). If this function returns 1, then all readings are not valid.


```c
int ADS_readAll(ADS_Main_t *ADSMain, float *readings)
```

**Params:**

- `ADSMain` - The main driver handle
- `readings` - The pointer to the array where the readings should be stored. This should be at least as long as the number of thermocouples that were passed to ADS_init

**Returns:**

- Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be retrieved

## int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings)

Get the latest temperature of all thermocouples.
These readings are cold junction compensated using one of the connected ADS1120s.

Readings are in the order that their corresponding thermocouple was passed to ADS_init.

If a reading couldn't be retrieved, its error flag will be 1.
Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect if they weren't successful (error flag = 1). If this function returns 1, then all readings are not valid, although their error flags might not reflect this.

Just as in ADS_readIndividualwTimestamp(), the timestamps are taken when the readings are received from the ADS1120s, so they can be used to determine if readings are being updated.


```c
int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings)
```

**Params:**

- `ADSMain` - The main driver handle
- `readings` - The pointer to the array where the readings and timestamps should be stored. This should be at least as long as the number of thermocouples that were passed to ADS_init

**Returns:**

- Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be retrieved

## int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp)

Get the latest internal temperature from one of the ADS1120s.
By default this is updated twice a second, but it can be changed by changing the ADS_INTERNAL_TEMP_DELAY macro.
This is the temperature used for cold junction compensation in all read functions. The specific ADS1120 that this is retrieved from is whichever ADS1120 is connected to the first thermocouple passed to ADS_init.


```c
int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp)
```

**Params:**

- `ADSMain` - The main driver handle
- `temp` - The pointer to where the temperature should be stored. This temperature is in Celsius

**Returns:**

- Returns 0 if retrieving the temperature was successful, returns 1 if it could not be retrieved

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

## float ADS_polyMicrovoltsToTemp(double microvolts)

Convert a voltage of a type T thermocouple to its corresponding temperature.


```c
float ADS_polyMicrovoltsToTemp(double microvolts)
```

**Params:**

- `microvolts` - The voltage of the thermocouple in microvolts

**Returns:**

- The calculated temperature of the thermocouple in Celsius

## double ADS_polyTempToMicrovolts(float temp)

Convert the temperature of a type T thermocouple to the voltage it would read.


```c
double ADS_polyTempToMicrovolts(float temp)
```

**Params:**

- `temp` - The temperature of the thermocouple in Celsius

**Returns:**

- The calculated voltage in microvolts that the thermocouple should generate if it was at the given temperature

## void vTCTask(void *pvParameters)

The main function for the RTOS task.
Collects and stores readings from all thermocouples, including a cold junction temperature from one of the connected ADS1120s.


```c
void vTCTask(void *pvParameters)
```

**Params:**

- `pvParameters` - The pointer to the main ADS_Main_t handle

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