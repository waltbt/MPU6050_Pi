![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
[![Licence](https://img.shields.io/github/license/Ileriayo/markdown-badges?style=for-the-badge)](./LICENSE)
# MPU6050 - 6 Axis Gyroscope and Accelerometer (IMU)  

The MPU6050 is an i2c, 16 Axis Gyroscope and Accelerometer.  

## Python code for the Raspberry Pi
This is a very basic program to allow you to use the MPU6050 with a Raspberry Pi. It does not have any special features, but can easily be modified to include them. It was written for Python 3.x, but should work for Python 2.x with minimal changes.  

## Operation Notes:
Calibration: May need to wait up to 40 seconds after startup for the signals to settle.

## Calculating offsets
I am unable to locate the code I used for calibration, but this appears to be the updated version of it.  
https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
https://wired.chillibasket.com/2015/01/calibrating-mpu6050/

## A explanation of how IMU work
https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

## Why is Z acceleration positive?
The device is not measuring the acceleration due to gravity (which is negative), but the force of the table or hand that it rests on pushing up against it - which is in the +Z direction.
If it was not, what would happen to Z if you dropped the device?
 - If it is measuring the force of gravity, it should read -9.8 as it falls
 - If it is measuring the force applied, it will read 0 - which is what we see

## SMBus
This program uses smbus.  Any recent version is likely to work as only basic functions are used.  

This project is licensed under the terms of the MIT license.  
