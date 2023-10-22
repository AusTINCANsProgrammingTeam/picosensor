# SPDX-FileCopyrightText: 2023 the ausTIN CANs FRC#2158
# SPDX-License-Identifier: MIT

# Read VL53L0X and VL6180X distance sensors.
# Print the sensed range/distance and light Lux every second.
import time

import busio
import board
import digitalio

import adafruit_vl53l0x
import adafruit_vl6180x

# Raspberry Pi Pico onboard LED
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Initialize I2C bus and sensor.
# VL53L0X - default I2C address is 0x29 and changed to 0x30
# 50 - 1200 mm of range distance
#
# instantiate the VL53L0X sensor on the I2C bus
i2c1 = busio.I2C(board.SCL, board.SDA)
vl53l0 = adafruit_vl53l0x.VL53L0X(i2c1)
vl53l0.set_address(0x30)   # address assigned should NOT be already in use

# VL6180X - I2C address is 0x29 and cannot be changed
# 5 - 200 mm of range distance
#
# instantiate the VL6180X sensor on the I2C bus
i2c2 = busio.I2C(board.SCL, board.SDA)
vl6180 = adafruit_vl6180x.VL6180X(i2c2)

# Optionally add an offset to distance measurements here (e.g. calibration)
# Swapping for the following would add a +10 millimeter offset to measurements:
# vl6180 = adafruit_vl6180x.VL6180X(i2c2, offset=10)

# Optionally adjust the measurement timing budget to change speed and accuracy.
# See the example here for more details:
#   https://github.com/pololu/vl53l0x-arduino/blob/master/examples/Single/Single.ino
# For example a higher speed but less accurate timing budget of 20ms:
# vl53.measurement_timing_budget = 20000
# Or a slower but more accurate timing budget of 200ms:
# vl53.measurement_timing_budget = 200000
# The default timing budget is 33ms, a good compromise of speed and accuracy.

# Main loop will read the ranges (mm) and light (lux) and print them every second.
while True:
    led.value = True
    light_lux = vl6180.read_lux(adafruit_vl6180x.ALS_GAIN_1)
    print("{0}, {0}, {0}".format(vl53l0.range, vl6180.range, light_lux))
    led.value = False
    time.sleep(1.0)  # Sleep for one second.
