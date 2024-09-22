# UPDATE: Feburary 16th, 2024
* RTOS-ified this (added like 4 lines to pause interupts)
* Pending testing
# User Guide (LAST UPDATED: July 7, 2021)

* [MAX11128 Technical Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX11120-MAX11128.pdf)
* [MAX11131BOB Breakout Board Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX11131BOB.pdf) (Left because MAX11131 is very similar to MAX11128 but does not support not having EOC nor CNVST)

Important note for MAX11131BOB breakout board users: the MAX11131BOB dev board 
connects analog input 15 to REF- by default. To configure it to use AIN15, cut the
jumper J15 and apply analog input at header H2. This can be found on the datasheet
on the [MAX11131BOB datasheet.](https://datasheets.maximintegrated.com/en/ds/MAX11131BOB.pdf)

## Pin Terminology

* CNVST   - ADC pin that is used for starting conversions on the MAX11131 chip
* EOC     - ADC pin set by the MAX11131 chip that tells the microcontroller when
            conversions are ready to be read

### Microcontroller Clock Settings

To guarantee proper functionality, the clock speed on the microcontroller should 
not be set higher than 180 MHz. This is because this library using a noop instruction
to delay execution of the CNVST pin cycle for at least 5ns. This delay is necessary
to guarantee that the ADC properly registers that the CNVST pin has been cycled.

### Configuring SPI settings

The maximum SPI frequency allowed for this ADC is 40 MHz. In the STM32CubeIde
SPI configuration, the configuration settings should be identical to those below. Note: the prescaler is not noted below but should be adjusted accordingly

* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First
* Clock Polarity: High
* Clock Phase: 2 Edge

### Configuring ADC pins

For this ADC, we need to configure three pins on the microcontroller: EOC, CNVST, and CS. These pins may be on any GPIO pin, but they must be configured exactly as shown below.

CS: (Output pin)
* GPIO output level: High
* GPIO mode: Output Push Pull
* GPIO Pull-up/Pull-down: None
* Maximum output speed: High

CNVST: (Output pin)
* GPIO output level: High
* GPIO mode: Output Push Pull
* GPIO Pull-up/Pull-down: None
* Maximum output speed: High

EOC: (Input pin)
* GPIO mode: Input mode
* GPIO Pull-up/Pull-down: None

### Configuring Hardware Pinouts
1. Initialize `GPIO_MAX11128_Pinfo` struct, this is your main interface for 
    configuring ADC hardware pins and which channels to read from
2. Manually assign pins in `GPIO_MAX11128_Pinfo` to hardware pins. A snippet of 
    the underlying data structure can be found below. For instance, MAX11131_CS_ADDR
    indicates that it is CS pin addr for an ADC.
3. To initialize multiple ADCs, create an array of `GPIO_MAX11128_Pinfo` structs
    and manually assign each struct's pinouts.
4. For future reference, you must provide the pin data for each ADC everytime 
    you want to interface with the ADC.
5. Set `HARDWARE_CONFIGURATION` to `EOC_AND_CNVST`, `EOC_ONLY`, or `NO_EOC_NOR_CNVST` in `GPIO_MAX11128_Pinfo` 

```
typedef struct GPIO_MAX11128_Pinfo {
    GPIO_TypeDef* MAX11128_CS_PORT;     // PORT belonging to CS pin
    GPIO_TypeDef* MAX11128_EOC_PORT;    // PORT belonging to EOC pin
    GPIO_TypeDef* MAX11128_CNVST_PORT;  // PORT belonging to CNVST pin
    uint16_t MAX11128_CS_ADDR;          // PIN belonging to CS pin
    uint16_t MAX11128_EOC_ADDR;         // PIN belonging to EOC pin
    uint16_t MAX11128_CNVST_ADDR;       // PIN belonging to CNVST pin

    uint8_t NUM_CHANNELS;               // Number of channels to read from
    uint8_t MAX11128_CHANNELS[16];      // Channel Identification Numbers

    uint8_t HARDWARE_CONFIGURATION;     // EOC and CNVST pin configuration
} GPIO_MAX11128_Pinfo;
```
3. Once ADC settings are configured, call `init_adc(SPI_HandleTypeDef* SPI_BUS, GPIO_MAX11128_Pinfo *pins)` on each ADC struct, making sure to pass the correct `SPI BUS`, `GPIO_MAX11128_Pinfo` configuration variable. This initializes, by default, 
channels 0-13,15. Channel 14 is not initialized because it will be used for the
CNVST pin in the Custom Internal Mode. Therefore, channel 14 will always read 0. If using MAX11128 in NO_EOC_NOR_CNVST, then all channels will be read.

4. To read current ADC values, call `read_adc(SPI_HandleTypeDef *SPI_BUS, GPIO_MAX11128_Pinfo *pinfo, uint16_t* adc_out)`. The adc_out array that you pass 
in must be of size 16 to guarantee proper behavior. Keep in mind that because
this function requires more than one SPI communication, it is should only be 
called as needed. It requires ~1ms to complete after testing.

5. If you want to change what channels are included in the conversion sequence when using EOC and/or CNVST,
simply modify the `GPIO_MAX11128_Pinfo` configuration struct with the desired 
pinouts and call`void set_read_adc_range(SPI_HandleTypeDef *SPI_BUS, GPIO_MAX11128_Pinfo *pinfo)`. This function also requires more than one SPI communication, and 
should be used on the fly. If unique sets of adc channels are required often,
consider implementing the SampleSet or Manual ADC read procedures. Information
on how to do so is specified in the MAX11128 datasheet linked at the top of this
document. If using no EOC nor CNVST pins, then the loop in read_adc will have to be changed to send the ADC the right channels. See Manual ADC read procedures for more information.

### Sample ADC Initialization Code

Here is all of the sample code needed to read from channels 0-13, 15 on an adc
that is connected to SPI 1. The pin assignment names are macros assigned 
by STM32CubeIDE to the actual micro pins. The `hspi1` is the default name that
STM32CubeIDE declared the SPI 1 bus as. It is officially declared in main.c as `SPI_HandleTypeDef hspi1` and should be auto generated for you after
configuring SPI_x. More information on how to configure SPI can be found [here.](https://www.youtube.com/watch?v=eFKeNPJq50g&feature=emb_title)

```
MX_SPI1_Init();                 // initializes SPI1
uint16_t adc_values[16] = {0};  // array for receiving adc conversions

GPIO_MAX11131_Pinfo adc_pins;
adc_pins.MAX11131_CS_PORT 		= SPI_ADC0_CS_GPIO_Port;
adc_pins.MAX11131_EOC_PORT		= SPI_EOC_GPIO_Port;
adc_pins.MAX11131_CNVST_PORT	= SPI_CNVST_GPIO_Port;
adc_pins.MAX11131_CS_ADDR 		= SPI_ADC0_CS_Pin;
adc_pins.MAX11131_EOC_ADDR		= SPI_EOC_Pin;
adc_pins.MAX11131_CNVST_ADDR	= SPI_CNVST_Pin;
adc_pins.HARDWARE_CONFIGURATION = EOC_ONLY;

init_adc(&hspi1, &adc_pins);

read_adc(&hspi1, &adc_pins, adc_values);
```

# Developer Guide

### Test Procedure (From MAX11131, needs adjustment for MAX11128)

To verify that adc channels 0-13, 15 are working correctly in Custom Internal mode, 
repeat the following procedure below. It is important to note that the MAX11131BOB 
connects analog input 15 to REF- by default. To configure it to use AIN15, cut the
jumper J15 and apply analog input at header H2. This can be found on the datasheet
on the [MAX11131BOB datasheet.](https://datasheets.maximintegrated.com/en/ds/MAX11131BOB.pdf)

1. Setup a timer based interrupt that counts up and is triggered every 1-2 seconds.
2. In the timer interrupt handler, toggle as many output pins from the main
    microcontroller to each ADC_in pin. Ideally, adjacent pins should be configured
    to always have opposite states. In addition, I voltage divided some of the 
    pins and determined roughly what counts I should be expecting for those pins.
3. Initialize the microcontroller to read from channels 0-13, 15 on the ADC.
    Additionally, it is helpful to initialize the ADC values array as a global var 
    in order to view from the live expressions tab in STM32CubeIDE. More information 
    on how to setup live expressions can be found at this [link.](https://www.youtube.com/watch?v=Nyml66k_Ppk)
4. Observe that each adc value in the array match what is expected. Note: the adc
    values are currently in counts, so some additional conversions may be needed.

For a brief example of what this implementation looks like on the Nucleo F446RE 
dev board, I have created an example program below that demonstrates the procedure
on adc pins 0-2. It is at this [Github Repo.](https://github.com/KingArthurZ3/MASA-firmware-dev) (Commit number: 2a15a8)

### Modifying the existing firmware

Most of the documentation for the exact details on how to configure each read mode should be documented in the firmware code. The README should only be for documenting project level configurations. In addition, detailed documentation
