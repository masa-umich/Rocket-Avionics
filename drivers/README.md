# Flight Computer & Bay Board Drivers

## Current Members:
Evan Eidt, Jack Hammerberg, Luke Weaver

## ADC Drivers:
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

* Name: SPIADC (its a adc, but spi)
* Chip: [MAX11128](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX11120-MAX11128.pdf)
* Everything the FCBBADC driver does, but you give it a spi address

## Chips:
* LAN8742A-CZ
    * [LAN8742A-CZ Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/DS_LAN8742_00001989A.pdf)
    * Type: Ethernet
    * Interface: Ethernet-MAC interface for STM32H7 Chip
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Transmit Data
        * Recieve Data

* LPS22HBTR 
    * [LPS22HBTR Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/bf/c1/4f/23/61/17/44/8a/DM00140895.pdf/files/DM00140895.pdf/jcr:content/translations/en.DM00140895.pdf)
    * Type: Barometer/Altimeter 
    * Interface: SPI1
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
    * Interface: SPI1
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Accelleration
        * Get Angular Rate
        * Get Temperature

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
    * Interface: SPI3
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Lat/Long/Alt

* SX1280 
    * [SX1280 Datasheet](https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo)
    * Type: LoRa 2.4GHz Radio 
    * Interface: SPI2
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Transmit Data
        * Recieve Data

* W25N01GVZEIG*
    * [W25N01GV Datasheet](https://www.winbond.com/resource-files/w25n01gv%20revl%20050918%20unsecured.pdf)
    * Type: Flash 
    * Interface: SPI2
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Write Flash
        * Read Flash

*This driver has been copied from one that was written in 2020, pending testing

* ??? - 
    * Type: Temperature 
    * Interface: SPI1
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Temperature