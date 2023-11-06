# Flight Computer & Bay Board Drivers

## Current Members:
Jack Hammerberg, Luke Weaver, Evan Eidt

## Chips:
* LSM6DSO32XTR - IMU - SPI1
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

* LPS22HBTR 
    * Type: Barometer/Altimeter 
    * Interface: SPI1
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Altitude
        * Get Pressure

* W25N01GVZEIG*
    * Type: Flash 
    * Interface: SPI2
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Write Flash
        * Read Flash

*The flash chip apparently already has drivers written but I don't know where they are.

* M24256E 
    * Type: EEPROM 
    * Interace: I2C
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Write Flash
        * Read Flash
    
* SX1280 
    * Type: LoRa 2.4GHz Radio 
    * Interface: SPI2
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Transmit Data
        * Recieve Data

* ??? 
    * Type: Ethernet
    * Interface: ???
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Transmit Data
        * Recieve Data

* ??? - 
    * Type: Temperature 
    * Interface: SPI (# unknown)
    * Driver Requirements:
        * Init
        * Send Command
        * Read Register
        * Write Register
        * Get Temperature