# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the VL53L0X distance sensor.
# Will print the sensed range/distance every second.
import time

import busio
import board

import adafruit_vl53l0x

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.GP5, board.GP4)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

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
    print("Range: {0}mm".format(vl53.range))
    time.sleep(1.0)
""" 
Using the tutorial here:
https://learn.adafruit.com/circuitpython-basics-i2c-and-spi/i2c-devices

Try and buid a python copy of ThadHouse's excellent help during 2022 FRC
season that reads sensor data from a REV robotics V3 color sensor and writes
to a serial port:
https://github.com/ThadHouse/picocolorsensor

REV Robotics docs:
https://docs.revrobotics.com/color-sensor/

Broadcom chip docs with internal information about the color sensor:
https://docs.broadcom.com/doc/APDS-9151-DS

Some notes:
Thad's example uses different i2c busses on the pico.
- This example is written to use a multiplexer
Thad's example transmits information on the UART.
- This example transmits across the USB serial port.
( boot.py enables USB serial communication )
Thad's example uses i2c_write_timeout_us from here:
  https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__i2c.html#ga0abb49ca0282530c2655f188b72eb653
- This shows a 25000usec timeout.
- Some writes also do not send a stop byte. ( for initial configuration of the sensor. )
- The library that we use does not allow witholding the stop byte unless you read immediately afterwards.
  https://docs.circuitpython.org/en/latest/shared-bindings/busio/#busio.I2C


Why this effort?
CircuitPython doesn't require setting up a build environment, and can be modified
from any team member's workstation.

"""

GainFactor = {
    "k1x" : 0,
    "k3x" : 1,
    "k6x" : 2,
    "k9x" : 3,
    "k18x" : 4 }

LEDPulseFrequency = {
    "k60kHz" : 0x18,
    "k70kHz" : 0x40,
    "k80kHz" : 0x28,
    "k90kHz" : 0x30,
    "k100kHz" : 0x38 }

LEDCurrent = {
    "kPulse2mA" : 0,
    "kPulse5mA" : 1,
    "kPulse10mA" : 2,
    "kPulse25mA" : 3,
    "kPulse50mA" : 4,
    "kPulse75mA" : 5,
    "kPulse100mA" : 6,
    "kPulse125mA" : 7 }

ProximityResolution = {
    "k8bit" : 0x00,
    "k9bit" : 0x08,
    "k10bit" : 0x10,
    "k11bit" : 0x18 }

ProximityMeasurementRate = {
    "k6ms" : 1,
    "k12ms" : 2,
    "k25ms" : 3,
    "k50ms" : 4,
    "k100ms" : 5,
    "k200ms" : 6,
    "k400ms" : 7 }

ColorResolution = {
    "k20bit" : 0x00,
    "k19bit" : 0x10,
    "k18bit" : 0x20,
    "k17bit" : 0x30,
    "k16bit" : 0x40,
    "k13bit" : 0x50 }

ColorMeasurementRate = {
    "k25ms" : 0,
    "k50ms" : 1,
    "k100ms" : 2,
    "k200ms" : 3,
    "k500ms" : 4,
    "k1000ms" : 5,
    "k2000ms" : 7 }

Register = {
    'kMainCtrl' : 0x00,
    'kProximitySensorLED' : 0x01,
    'kProximitySensorPulses' : 0x02,
    'kProximitySensorRate' : 0x03,
    'kLightSensorMeasurementRate' : 0x04,
    'kLightSensorGain' : 0x05,
    'kPartID' : 0x06,
    'kMainStatus' : 0x07,
    'kProximityData' : 0x08,
    'kDataInfrared' : 0x0A,
    'kDataGreen' : 0x0D,
    'kDataBlue' : 0x10,
    'kDataRed' : 0x13
}

MainCtrlFields = {
    "kProximitySensorEnable" : 0x01,
    "kLightSensorEnable" : 0x02,
    "kRGBMode" : 0x04,
    "kAllEnable" : 0x07 }

kAddress = 0x52  # Address of the Rev3 Color sensor
kExpectedPartID = 0xc2

# REV v3 color sensors only use address 0x52.
# in order to use more than one, you either need to use a multiplexer
# or multiple i2c busses.  This example is written for a multiplexer.
kMuxAddress = 0x70
kSensorCount = 3  # How many sensors are attached to the mux

"""
Defaults for the rev v3 color sensor in hardware:

kDefaultGain = GainFactor.k3x
kDefaultPulseFreq = LEDPulseFrequency.k60kHz
LEDCurrent kDefaultLEDCurrent = LEDCurrent.kPulse100mA
kDefaultPulses = 32;
kDefaultProxRes = ProximityResolution.k11bit
kDefaultProxRate = ProximityMeasurementRate.k50ms
kDefaultColorRes = ColorResolution.k18bit
kDefaultColorRate = ColorMeasurementRate.k100ms
"""

def select_channel(mux, device):
    # Write a command to an attached multiplexer to select a device
    channel = 1 << device
    with mux:
        mux.write(bytes([channel]))

def init_device(dev):
    # Send commands to initialize a rev color sensor.
    # if defaults need to be changed, they should be changed here.
    with dev:
        try:
            # Enable Proximity, light sensor and RGBmode.
            dev.write(bytes([Register["kMainCtrl"], MainCtrlFields["kAllEnable"]]))
            dev.write(bytes([Register["kProximitySensorRate"], ProximityResolution["k11bit"] |
                      ProximityMeasurementRate["k100ms"]]))
            dev.write(bytes([Register["kProximitySensorPulses"], 32]))
            return True
        except Exception:
            return False

# Last number is the frequency for the i2c bus.
i2c = busio.I2C(GP5, GP4, frequency = 400000 )

mux = I2CDevice(i2c, kMuxAddress)
select_channel(mux, 0) # Need to have at least one sensor available or else init of color_sensor fails
color_sensor = I2CDevice(i2c, kAddress)

sensor_valid = [False] * kSensorCount
for c in range(kSensorCount):
    result = bytearray(1)
    select_channel(mux, c)
    sensor_valid[c] = init_device(color_sensor)
    if ( sensor_valid[c] ):
      with color_sensor:
          color_sensor.write_then_readinto(bytes([Register["kMainStatus"]]), result)

values = [0] * kSensorCount * 5
while (True):
    for c in range(kSensorCount):
        select_channel(mux, c)
        result = bytearray(15)  # zero fill the array every time.
        try:
            with color_sensor:
                color_sensor.write_then_readinto(bytes([Register["kMainStatus"]]), result)
                # Uncomment to print out raw byte string
                # print(result)

            if (( result[0] & 0x20 ) != 0 ):
                # First byte is status, and 0x20 is a failure, we should not capture any sensor information
                raise Exception("Failed status byte") 
            
            values[(c*5)+4] = ((result[1] & 0xFF) | ((result[2] & 0xFF) << 8)) & 0x7FF # Proximity Sensor Data
            values[(c*5)+3] = ((result[3] & 0xFF) | ((result[4] & 0xFF) << 8) | ((result[5] & 0xFF) << 16)) & 0x03FFFF # Light Sensor IR 
            values[(c*5)+2] = ((result[6] & 0xFF) | ((result[7] & 0xFF) << 8) | ((result[8] & 0xFF) << 16)) & 0x03FFFF # Light Sensor Green
            values[(c*5)+1] = ((result[9] & 0xFF) | ((result[10] & 0xFF) << 8) | ((result[11] & 0xFF) << 16)) & 0x03FFFF # Light Sensor Blue
            values[(c*5)] = ((result[12] & 0xFF) | ((result[13] & 0xFF) << 8) | ((result[14] & 0xFF) << 16)) & 0x03FFFF # Light Sensor Red
            sensor_valid[c] = True
        except Exception as error:
            # uncomment to see caught error
            # print(error)
            sensor_valid[c] = False
            init_device(color_sensor)


    for c in range(kSensorCount):
        print(1 if sensor_valid[c] else 0, end=",")

    for v in range(kSensorCount*5):
        print(values[v], end=",")

    print("end")
    colorSum = values[0] + values[1] + values[2]
    if (colorSum > 0):
        redRatio = values[0] / colorSum
        blueRatio = values[1] / colorSum
        greenRatio = values[2] / colorSum
    if (colorSum > 0):
        print("redRatio: " + str(redRatio))
        print("blueRatio: " + str(blueRatio))
        print("greenRatio: " + str(greenRatio))
    
    if (values[4] > 1560):
        print("DISTANCE: Less than 10 mm")
    elif(values[4] > 670 and values[4] <= 1560):
        print("DISTANCE: Between 10 and 15 mm")
    elif(values[4] > 500 and values[4] <= 670):
        print("DISTANCE: Between 15 and 20 mm")
    elif(values[4] > 430 and values[4] <= 500):
        print("DISTANCE: Between 20 and 25 mm")
    elif(values[4] > 320 and values[4] <= 430):
        print("DISTANCE: Between 25 and 30 mm")
    elif(values[4] > 245 and values[4] <= 320):
        print("DISTANCE: Between 30 and 35 mm")
    elif(values[4] > 220 and values[4] <= 245):
        print("DISTANCE: Between 35 and 40 mm")
    elif(values[4] > 160 and values[4] <= 220):
        print("DISTANCE: Between 40 and 45 mm")
    elif(values[4] > 145 and values[4] <= 160):
        print("DISTANCE: Between 45 and 50 mm")
    else:
        print("DISTANCE: More than 50 mm")
        
    print("PROXIMITY: " + str(values[4]))
    
    if(redRatio > .29 and redRatio < .37 and blueRatio > .08 and blueRatio < .13 and greenRatio > .51 and greenRatio < .59):
        print("COLOR: Yellow")
    elif(redRatio > .49 and redRatio < .63 and blueRatio > .03 and blueRatio < .12 and greenRatio > .32 and greenRatio < .43):
        print("COLOR: Orange")
    elif(redRatio > .25 and redRatio < .30 and blueRatio > .23 and blueRatio < .30 and greenRatio > .42 and greenRatio < .47):
        print("COLOR: White")
    else:
        if (values[0] > values[1]):
            if (values[0] > values[2]):
                print("Color: Red")
            else:
                print("Color: Green")
        
        else:
            if (values[1] > values[2]):
                print("Color: Blue")
            else:
                print("Color: Green")



    

        

    # Sleep for 100ms + time in sensor calls.
    sleep(.1)

