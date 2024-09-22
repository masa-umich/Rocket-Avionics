## ADC Drivers:
* Name: FCBBADC (Flight Computer & Bay Board Analog to Digital Converter)
* Read in batch
* Convert to raw in batch
* Convert to voltages in batch (float32 array)
* Driver Functions:
    * `readADCRaw()`
		* Get Raw Values
		* Parameters: 
			* `ADC_HandleTypeDef*` for your ADC type handler
			* `int` for the amount of channels you are using
			* `int*` for the array you want values in
			* `int` for the maximum delay per channel in milliseconds
    * `readADCVolt()`
		* Get Raw Voltages
		* Parameters: 
			* `ADC_HandleTypeDef*` for your ADC type handler
			* `int` for the amount of channels you are using
			* `float*` for the array you want values in
			* `int` for the maximum delay per channel in milliseconds
    * `readADC()`
		* Get Voltages & Timestamps
		* Parameters: 
			* `ADC_HandleTypeDef*` for your ADC type handler
			* `int` for the amount of channels you are using
			* `float*` for the array you want ADC values in & timestamp
			* `int` for the maximum delay per channel in milliseconds
## Important Usage Information
To setup multiple channels to read, in your CubeIDE IOC file, you must enable `Scan Conversion Mode` by increasing `Number of Conversion` under `ADC_Regular_ConversionMode` to the amount of channels you will be using. You must also enable `Discontinuous Conversion Mode` and set `Number of Discontinuous Conversions` to the same number of ranks you configured.

Due to how multi-channel ADCs work, you must increase `Sampling Time` for each rank you create to a value equal to or more than `64.5`. If using standard clocks, this means that each channel takes about **3 microseconds** to read, so keep that in mind when using this.

This driver assumes there is a function called `getTimestamp()` that returns a uint64_t somewhere else in the program. An example of this can be [found below](#example-gettimestamp-function) 
## Example Usage
```c
TickType_t xLastWakeTime;
const TickType_t xFrequency = 750; // Milliseconds
// Initialize the xLastWakeTime variable with the current time.
xLastWakeTime = xTaskGetTickCount();
int channels = 4;
// Must be one larger than the number of channels to include the timestamp
// Can be equal to amount of channels if using readADCRaw() or readADCVolt()
float adcValues[5];

for(;;) {
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	readADC(&hadc1, channels, adcValues, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED
}
```
***
## Example getTimestamp() Function
In the future, we will need a timestamp function with sub-millisecond timing but an external `getTimestamp()` function is required, so here is an example.
```c
uint64_t getTimestamp(void) {
	uint32_t timestamp = HAL_GetTick();
	return (uint64_t)timestamp;
}
```