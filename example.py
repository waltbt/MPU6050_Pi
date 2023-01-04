#!/usr/bin/env python3

import MPU6050 as MPU
import time

mpu = MPU.MPU6050()

mpu.set_offsets([10,12,100,4,1,0]) # Example calibration offsets

"""
Read all 7 outputs in a loop
"""
while(1):
	reading = mpu.read_imu7()
	print(f"Accel X: {reading[0]} Accel Y: {reading[1]} Accel Z: {reading[2]}")
	print(f"Gyro X: {reading[3]} Gyro Y: {reading[4]} Gyro Z: {reading[5]}")
	print(f"Temperature: {reading[6]}")
	time.sleep(.2)
