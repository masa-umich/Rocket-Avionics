/** max11128.h
 *
 * Header file for communicating with MAX11128 adc
 * datasheet:https://datasheets.maximintegrated.com/en/ds/MAX11120-MAX11128.pdf
 *
 * Important note for MAX11131BOB breakout board users: the MAX11131BOB dev board
 * connects analog input 15 to REF- by default. To configure it to use AIN15, cut the
 * jumper J15 and apply analog input at header H2. This can be found on the datasheet
 * on the [MAX11131BOB datasheet.](https://datasheets.maximintegrated.com/en/ds/MAX11131BOB.pdf)
 *
 * Pin Terminology
 * CNVST   - ADC pin that is used for starting conversions on the MAX11128 chip
 * EOC     - ADC pin set by the MAX11128 chip that tells the microcontroller when
 *          conversions are ready to be read
 *
 * Public Interface Functions
 *
 *  init_adc()  -   Automatically initializes ADC to read from channels 0-13, 15
 *                  using CUSTOM INT scan method for hardware configurations using
 *                  EOC and/or CNVST pins. More explanation on scan
 *                  method differences can be found below. In addition, this
 *                  function configures ADC registers to save the average of 4
 *                  ADC conversion results for each channel, denoted by the
 *                     'SET_MAX11128_AVGON' bit.
 *
 *  read_adc()  -   Reads ADC conversions for configured channels on ADC. Reads
 *                  adc conversions on 'adc_out'. 'adc_out' should be an array
 *                  of size 16 to avoid index out of bound errors. Each index in
 *                  'adc_out' corresponds to the channel it's for.
 *
 *  set_read_adc_range()-   Configures ADC registers to read from custom
 *                          range of ADC channels. Prior to calling this
 *                          function, the user must set the contents of the
 *                          'MAX11128_CHANNELS' array inside of the
 *                          'GPIO_MAX11128_Pinfo' to the channel numbers to read
 *                          from. In addition, the user must also set the
 *                          'NUM_CHANNELS' variables in the struct to the number
 *                          of channels. This function can only be called after
 *                          init_adc().
 *
 * Scan Modes
 *
 *  Overview: There are several different scan modes available to this ADC. Of
 *  the different modes, only CUSTOM_INT is currently implemented because it
 *  handles the majority of this library's use cases. In addition, this mode
 *  also increases the speed at which ADC conversions can be retrieved.
 *
 *  CUSTOM_INT:
 *  This mode indicates that the ADC will use its internal clock when carrying
 *  out ADC conversions, it also gives the user the ability to set what ADC
 *  channels should be converted.
 *
 *  MANUAL:
 *  The second scan mode of use is MANUAL. MANUAL mode uses the
 *  external clock for carrying out ADC conversions. In addition, to retrieve
 *  ADC conversions, the user must send the channel ID to the ADC and wait for
 *  the conversion result on the SPI MISO line. This is implemented for no EOC
 *  nor CNVSt configurationss, and currently scans all channels of the ADC.
 *
 *
 * Implementation Details
 *
 * Developers using this library can select the hardware configuration of their choice
 * by setting the HARDWARE_CONFIGURATION of GPIO_MAX11128_Pinfo to EOC_AND_CNVST,
 * EOC_ONLY, or NO_EOC_NOR_CNVST depending on the hardware configuration of the board.
 *
 * For the CUSTOM_INT mode, the developer can choose to enable the SWCNV bit.
 * If the SWCNV bit is enabled, the ADC samples all the channels once after
 * the user is done requesting data, and does not sample again until the user
 * finishes requesting data again. This means that the longer you wait between
 * requesting ADC conversions, the more the results will lag reality. However,
 * the benefit is that this method can be faster than disabling the SWCNV bit.
 *
 * By disabling the SWCNV bit, the user must cycle the CNVST (Conversions Start)
 * pin to tell the ADC to sample conversions before getting conversion data.
 * This guarantees that conversion data will be as recent as possible at the
 * expense of longer conversion times.
 *
 * One advantage of enabling the SWCNV bit is that it frees up all channels on
 * the ADC to read data from.
 *
 * Conversion Times for SWCNV (10000 ADC samples) (timings from 11131 docs)
 * SWCNV enabled    -    9023 milliseconds
 * SWCNV disabled   -    9052 milliseconds
 *
 *
 */
#ifndef MAX11128_H
#define MAX11128_H
 /* Includes */
#include "stdint.h"
#include "stdlib.h"

#include "main.h" // includes hal library for whatever microcontroller is being used

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

/* DEBUG ENABLER */
#define MAX11128_DEBUG_EN    (uint16_t) 0x0001 // set to 1 to enable debugging

/* Hardware Configurations */
#define EOC_AND_CNVST 0
#define EOC_ONLY 1
#define NO_EOC_NOR_CNVST 2

/* Register Definitions */

// Register Identification Code
// datasheet definitions between on pg21
#define MAX11128_MODE_CNTL   (uint16_t) 0x0000   // 0b0          followed by 0s
#define MAX11128_MODE_MANUAL (uint16_t) 0x0800   // 0b00001      followed by 0s
#define MAX11128_CONFIG      (uint16_t) 0x8000   // 0b1000       followed by 0s
#define MAX11128_UNIPOLAR    (uint16_t) 0x8800   // 0b10001000   followed by 0s
#define MAX11128_BIPOLAR     (uint16_t) 0x9000   // 0b1001       followed by 0s
#define MAX11128_RANGE       (uint16_t) 0x9800   // 0b10011000   followed by 0

/* Offset bits mapping adc channel numbers to bit number */
#define MAX11128_CUSTOM_SCAN0_SUB   (uint8_t) 5
#define MAX11128_CUSTOM_SCAN1_ADD   (uint8_t) 3

/* Register bits for adc Mode Control registers */
#define SET_MAX11128_SWCNV          (uint16_t) 0x0002   // 0b10  preceded by 0s
#define SET_MAX11128_CHAN_ID        (uint16_t) 0x0004   // 0b100 preceded by 0s

/* Register bits for adc configuration registers */
#define SET_MAX11128_AVGON          (uint16_t) 0x0200   // 0b1000000000
#define SET_MAX11128_ECHO_ON        (uint16_t) 0x0004   // 0b0100

/* Register bits for adc scan registers */
#define MAX11128_CUSTOM_SCAN0       (uint16_t) 0xA000   // 0b10100
#define MAX11128_CUSTOM_SCAN1       (uint16_t) 0xA800   // 0b10101
#define MAX11128_CUSTOM_SCAN_ALL_0  (uint16_t) 0x05F8   // 0b10111111000
#define MAX11128_CUSTOM_SCAN_ALL_1  (uint16_t) 0x07F8   // 0b11111111000

/* Channel size in FIFO register */
#define MAX11128_CHANNEL_SZ	        (uint8_t)   0x0002

/* Global Var Definitions */
#define MAX11128_MAX_CHANNELS       (uint8_t)   0x0010  // 15 Channels(0-13, 15)
#define MAX11128_EOC_WAIT_TIME      (uint16_t)  0x0014  // Max wait time for EOC
                                                        // to finish conversions

// GPIO pinout memory addresses
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

// Mode Control Scan Registers
// SCAN STATE specifications are defined in pg22
enum SCAN_STATES {
    HOLD,       // Maintains same ADC conversion procedure as before
    MANUAL,     // Requires transmission of Channel ID each ADC conversion
    REPEAT,     // Repeats scanning channel N for number of times
    STD_INT,    // Scans all channels 0-15 in ascending order
    STD_EXT,    // Same as STD_INT except uses external clock
    UPPER_INT,  // Scans cahnnels N through 15/11/7/3 in ascending order
    UPPER_EXT,  // Same as UPPER_INT, except uses external clock
    CUSTOM_INT, // Scans specified channels in ascending order
    CUSTOM_EXT  // Same as CUSTOM_INT except uses external clock
};

/* Public Function Prototypes */
#ifdef HAL_SPI_MODULE_ENABLED
/**
 *  Initialize ADC hardware component
 *
 *  detailed documentation starts on datasheet pg21
 *  @param SPI_BUS      <SPI_HandleTypeDef*> SPI object ADC is on
 *  @param pins         <GPIO_MAX11128_Pinfo*> contains ADC pin defs,
 *                                              refer to def above
 *
 *  Note: assumes 8bit data framing on SPI
 */
void init_adc(SPI_HandleTypeDef* SPI_BUS, GPIO_MAX11128_Pinfo *pins);

/**
 *  Sets range to read from adc. This function takes a NONTRIVIAL amount
 *  of time for completion, roughly 1ms, as it requires more than one SPI
 *  communication transmission.
 *
 *  general documentation starts on datasheet pg 21
 *  @param SPI_BUS      <SPI_HandleTypeDef*>    SPI object adc is on
 *  @param pinfo        <GPIO_MAX11128_Pinfo*>  contains ADC pin defs
 *
 */
void set_read_adc_range(SPI_HandleTypeDef *SPI_BUS, GPIO_MAX11128_Pinfo *pinfo);

/**
 *  Convenience function for reading from configured channels on adcx
 *  This function takes a NONTRIVIAL amount of time for completion, roughly 1ms,
 *  as it requires more than one SPI communication transmission. This function
 *  also assumes the adc range to read from has already been set by
 *  set_read_adc_range()
 *
 *  general documentation starts on datasheet pg 21
 *  @param SPI_BUS      <SPI_HandleTypeDef*> SPI object adc is on
 *  @param pinfo        <GPIO_MAX11128_Pinfo*>   contains ADC pin defs
 *	@param adc_out      <uint16_t*> raw adc counts for each channel (0-4096)
 *
 *	Note: adc_out should be initialized to size 16 to guarantee safe operations
 *      if channel x, y, z are selected, then indices x, y, z will be filled
 *      in adc_out
 */
void read_adc(SPI_HandleTypeDef *SPI_BUS, GPIO_MAX11128_Pinfo *pinfo,
                                                        uint16_t *adc_out);
#endif

#endif /* end header include protection */
