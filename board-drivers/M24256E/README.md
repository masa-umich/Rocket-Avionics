# M24256E
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

## Dev Board Pinout

NOTE: The SO8N package for this IC doesn't have a
physical indicator for where Pin 1 is. I assumed that
the writing in the drawing in Section 10.1 of the
datasheet is the same orientation as the writing on
the IC itself.

| Pin # | Function |
|-------|----------|
| 1     | NC       |
| 2     | NC       |
| 3     | NC       |
| 4     | VSS      |
| 5     | SDA      |
| 6     | SCL      |
| 7     | !WC      |
| 8     | VCC      |
