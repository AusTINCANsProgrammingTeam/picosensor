# picosensor
# 2023 the ausTIN CANs FRC#2158
# License: MIT
CircuitPython to read distance sensor data on a Raspberry Pi Pico in order to send the data to a roboRIO.

Use Adafruit VL53L0X and VL6180X distance sensors on I2C bus (STEMMA/QT connections). I2C addresses multiplexing is performed by
an Adafruit PCA9546A board.

The roboRIO can provide Ground, 5v, and the communication signal to the device.
UART Rx on the roboRIO is pin 10 (bottom row, 5th in from the right) and connects to pin 1 (UART0 TX) of the Pi Pico.

Adafruit PiCowbell Proto for Pico is used to provide the STEMMA QT / Qwiic connector for fast I2C, using GP4 and GP5
