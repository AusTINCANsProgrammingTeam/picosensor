# SPDX-FileCopyrightText: 2023 the ausTIN CANs FRC#2158
# SPDX-License-Identifier: MIT

# Read VL53L0X and VL6180X distance sensors.
# Print the sensed range/distance and light Lux every second.
import time

import board
import busio

import digitalio

import adafruit_vl53l0x
import adafruit_vl6180x
import adafruit_tca9548a

# Raspberry Pi Pico onboard LED
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Initialize I2C bus and sensor.
# Adafruit
i2c0 = busio.I2C(board.GP5, board.GP4)

# Create the PCA9546A object and give it the I2C bus
try:
    # Port 0 is connected to VL6180
    # Port 1 is connected to VL53L0
    # Port 2 and 3 are free for others sensors
    mux = adafruit_tca9548a.PCA9546A(i2c0)
    # VL6180X - I2C address is 0x29 and cannot be changed
    # 5 - 200 mm of range distance
    # instantiate the VL6180X sensor using the PCA9546A channel instead
    # of the I2C object
    vl6180 = adafruit_vl6180x.VL6180X(mux[0])

    # VL53L0X - default I2C address is 0x29
    # 50 - 1200 mm of range distance
    #
    # instantiate the VL53L0X sensor using the PCA9546A channel instead
    # of the I2C object
    vl53l0 = adafruit_vl53l0x.VL53L0X(mux[1])

    # Main loop will read the ranges (mm) and light (lux) and print them every second.
    while True:
        led.value = True
        light_lux = vl6180.read_lux(adafruit_vl6180x.ALS_GAIN_1)
        # Range are millimeters, illuminance in Lux 
        rangeVL6180 = vl6180.range
        rangeVL53L0 = vl53l0.range
        # VL6180 is more precise, so use it for less than 255
        if rangeVL6180 >= 255:
            range = rangeVL53L0
        else:
            range = rangeVL6180
        print(range, light_lux)
        led.value = False
        time.sleep(1.0)  # Sleep for one second.
except (OSError) as e:
    print("{}. PCA9546A Not found at address 0x70".format(e))
except (ValueError) as ve:
    print("{}.".format(ve))


