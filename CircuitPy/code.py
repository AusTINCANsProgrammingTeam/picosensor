# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the VL53L0X distance sensor.
# Will print the sensed range/distance every second.
import time

import busio
import board


import adafruit_vl53l0x

# Initialize I2C bus and sensor.
sensor1Range = -2.0
sensor2Range = -2.0

uart = busio.UART(board.GP0, board.GP1, baudrate=115200, bits=8, parity=None )
    
print("new version - uart")
try:
    i2c1 = busio.I2C(board.GP5, board.GP4)
    vl53_1 = adafruit_vl53l0x.VL53L0X(i2c1)
except:
    print("No 1st sensor connected")

try:
    i2c2 = busio.I2C(board.GP7, board.GP6)
    vl53_2 = adafruit_vl53l0x.VL53L0X(i2c2)
except:
    print("No 2nd sensor connected")
    

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
    try:
        sensor1Range = vl53_1.range
    except:
        sensor1Range = -1
        
        
    try:
        sensor2Range = vl53_2.range
        
    except:
        sensor2Range = -1
    print("{0}, {1},".format(sensor1Range, sensor2Range))
    uart.write(bytearray("{0}, {1},\r\n".format(sensor1Range, sensor2Range)))

    time.sleep(0.005)
