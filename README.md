# picosensor
CircuitPython to read distance sensor data on a Raspberry Pi Pico in order to send the data to a roboRIO.

Use Adafruit VL53L0X and VL6180X distance sensors on I2C bus (STEMMA/QT connections)

The roboRIO can provide Ground, 5v, and the communication signal to the device.

5v on the roboRIO is MXP pin 1 (Top right pin) and connects to pin 39 (VSYS) of the Pi Pico.
GND on the roboRIO is pin 8 (bottom row, 4th in from the right) and connects to either Pin 3 or Pin 38 (GND) of the Pi Pico.
UART Rx on the roboRIO is pin 10 (bottom row, 5th in from the right) and connects to pin 1 (UART0 TX) of the Pi Pico.

Adafruit PiCowbell Proto for Pico is used to provide the STEMMA QT / Qwiic connector for fast I2C
