# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the VL53L0X distance sensor.
# Will print the sensed range/distance every second.
import time

import busio
import board

import adafruit_vl53l0x

# Initialize I2C bus and sensor.
i2c1 = busio.I2C(board.GP5, board.GP4)
vl53_1 = adafruit_vl53l0x.VL53L0X(i2c1)

i2c2 = busio.I2C(board.GP7, board.GP6)
vl53_2 = adafruit_vl53l0x.VL53L0X(i2c2)

# Optionally adjust the measurement timing budget to change speed and accuracy.
# See the example here for more details:
#   https://github.com/pololu/vl53l0x-arduino/blob/master/examples/Single/Single.ino
# For example a higher speed but less accurate timing budget of 20ms:
# vl53.measurement_timing_budget = 20000
# Or a slower but more accurate timing budget of 200ms:
# vl53.measurement_timing_budget = 200000
# The default timing budget is 33ms, a good compromise of speed and accuracy.

# Main loop will read the range and print it every second.
while True:
    print("{0}, {0}".format(vl53_1.range, vl53_2.range))
    time.sleep(1.0)