# picosensor
CircuitPython to read sensor data on a Raspberry Pico in order to send the data to a roboRIO


Inspired by:  https://github.com/ThadHouse/picocolorsensor

The roboRIO can provide Ground, 5v, and the communication signal to the device.

5v on the roboRIO is MXP pin 1 (Top right pin) and connects to pin 39 (VSYS) of the Pi Pico.
GND on the roboRIO is pin 8 (bottom row, 4th in from the right) and connects to either Pin 3 or Pin 38 (GND) of the Pi Pico.
UART Rx on the roboRIO is pin 10 (bottom row, 5th in from the right) and connects to pin 1 (UART0 TX) of the Pi Pico.
