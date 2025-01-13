# Flight Computer & Bay Board Drivers

## Current Members:
Jack Hammerberg, Felix Foreman, Ryan Salehi

## Summary:
This directory contains all of the relevant drivers used on the rocket avionics firmware. Not every board will use every driver, these drivers are designed to be copy-pasted into each application or project as needed. Most if not all require FreeRTOS to work properly for non-blocking opperation.

## Drivers:
* Name: FCBBADC (Flight Computer & Bay Board Analog to Digital Converter)
* Init ADCs in batch
* Read in batch
* Convert to raw in batch
* Convert to voltages in batch (float32 array)
* Driver Functions:
    * Init
    * Get Raw Values
    * Get Raw Voltages
    * Get Voltages & Timestamps

* MAX11128
    * [MAX11128 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX11120-MAX11128.pdf)
    * Type: ADC
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Voltage

* LPS22HBTR 
    * [LPS22HBTR Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/bf/c1/4f/23/61/17/44/8a/DM00140895.pdf/files/DM00140895.pdf/jcr:content/translations/en.DM00140895.pdf)
    * Type: Barometer/Altimeter 
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Altitude
        * Get Pressure

* LSM6DSO32XTR
    * [LSM6DSO32XTR Datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32x.pdf)
    * Type: IMU
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Accelleration
        * Get Angular Rate
        * Get Temperature

* M5611
    * [M5611 Datasheet](https://www.mouser.com/datasheet/2/418/6/ENG_DS_MS5611_01BA03_B3-1134567.pdf)
    * Type: Barometer/Altimeter
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Pressure Convert
        * Temperature Convert
        * Read Pressure
        * Read Temperature

* M24256E
    * [M24256E Datasheet](https://www.st.com/resource/en/datasheet/m24256e-f.pdf)
    * Type: EEPROM 
    * Interace: I2C
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Write Flash
        * Read Flash

* NEO-M9N-00B
    * [NEO-M9N-00B](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf)
    * Type: GPS
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Lat/Long/Alt

* SX1280 
    * [SX1280 Datasheet](https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo)
    * Type: LoRa 2.4GHz Radio 
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Transmit Data
        * Recieve Data

* W25N01GVZEIG
    * [W25N01GV Datasheet](https://www.winbond.com/resource-files/w25n01gv%20revl%20050918%20unsecured.pdf)
    * Type: Flash 
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Write Flash
        * Read Flash

* ADS1120 
    * [ADS1120 Datasheet](https://www.ti.com/lit/ds/symlink/ads1120.pdf)
    * Type: Thermocouple ADC
    * Interface: SPI
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Temperature
        * Read All TCs and Attach Timestamps

* Valves
    * Type: Solenoid Valves
    * Interface: GPIO & Shift Register
    * Driver Requirements:
        * Valve Voltage Config (Shift Register)
        * Open Valve
        * Close Valve

* TCP
    * Type: TCP/IP
    * Interface: Ethernet
    * Driver Requirements:
        * Packet Parsing
        * Connection Thread Manager
        * TCP Server
        * TCP Client