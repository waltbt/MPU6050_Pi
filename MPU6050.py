#!/usr/bin/env python3
"""
MPU6050.py
Function: A simple class to operate an MPU6050 IMU on an RPi with Python
Author: Benjamin Walt
Date: 1/3/2023
Version: 0.1
Copyright (c) Benjamin Thomas Walt
Licensed under the MIT license.
"""



import smbus
import time
import numpy as np

_MPU6050_I2CADDR_DEFAULT = 0x68 # Default address
# Key Registers
_MPU6050_SMPLRT_DIV = 0x19  # sample rate dividor register
_MPU6050_CONFIG = 0x1A # Configuration register
_MPU6050_GYRO_CONFIG = 0x1B # Gyro configuration register
_MPU6050_ACCEL_CONFIG = 0x1C # Accelerometer configration register
_MPU6050_ACCEL_XOUT_H = 0x3B  # Start address for accel reads and full reads
_MPU6050_TEMP_OUT_H = 0x41  # Start address for temp reads
_MPU6050_GYRO_XOUT_H = 0x43  # Start address for gyro reads
_MPU6050_SIGNAL_PATH_RESET = 0x68 # Signal path reset register
_MPU6050_PWR_MGMT_1 = 0x6B		# Power Management 1 register
_MPU6050_WHO_AM_I = 0x75 # Who Am I register (Contains I2C Address)

# Settings
_MPU6050_STANDARD_GRAVITY = 9.80665
# Index of values in the _MPU6050_ACCEL_SCALE
_MPU6050_ACCEL_2G = 0
_MPU6050_ACCEL_4G = 1
_MPU6050_ACCEL_8G = 2
_MPU6050_ACCEL_16G = 3
# Index of values in the _MPU6050_GYRO_SCALE
_MPU6050_GYRO_250 = 0
_MPU6050_GYRO_500 = 1
_MPU6050_GYRO_1000 = 2
_MPU6050_GYRO_2000 = 3

_MPU6050_GYRO_SCALE = [131.0, 65.5, 32.8, 16.4]
_MPU6050_ACCEL_SCALE = [16384.0, 8192.0, 4096.0, 2048.0]

_MPU6050_ACCEL_X_OFFSET_HIGH_ADDRESS = 0x06
_MPU6050_ACCEL_Y_OFFSET_HIGH_ADDRESS = 0x08
_MPU6050_ACCEL_Z_OFFSET_HIGH_ADDRESS = 0x0A
_MPU6050_GYRO_X_OFFSET_HIGH_ADDRESS = 0x13
_MPU6050_GYRO_Y_OFFSET_HIGH_ADDRESS = 0x15
_MPU6050_GYRO_Z_OFFSET_HIGH_ADDRESS = 0x17

class MPU6050:

	def __init__(self, address=_MPU6050_I2CADDR_DEFAULT):
		self._bus = smbus.SMBus(1) # Channel = 1
		self._address = address
		if(self._read_reg(_MPU6050_WHO_AM_I) != _MPU6050_I2CADDR_DEFAULT):
			raise RuntimeError("Error loading MPU-6050 IMU")

		self._cur_gyro_setting = _MPU6050_GYRO_250
		self._cur_accel_setting = _MPU6050_ACCEL_2G

		self._reset()
		self.set_sample_rate_divisor(0) # 0 is No divisor
		self.set_filter_bandwidth(0) # 0 is No filter
		self.set_gyro_range(self._cur_gyro_setting)
		self.set_accelerometer_range(self._cur_accel_setting)
		self.set_clock(1) # PLL with X axis gyroscope reference
		time.sleep(0.1)

	def _write_reg(self, reg, value):
		"""Write 1 byte to a given register"""
		self._bus.write_byte_data(self._address, reg, value)

	def _read_reg(self, reg):
		"""Read 1 byte from a given register"""
		return self._bus.read_byte_data(self._address, reg)

	def _reset(self):
		"""Reset the device to return all settings to the startup state
		Finally wakes up the device."""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_PWR_MGMT_1)
		# Set bit 7, the reset bit, to 1
		new_register_value = self._set_bit(cur_reg_reading, 7)
		# Write the new register
		self._write_reg(_MPU6050_PWR_MGMT_1, new_register_value)
		# wait
		while(self._test_bit(self._read_reg(_MPU6050_PWR_MGMT_1),7)):
			time.sleep(0.001)
		# Reset the gyro (bit2), accel (bit1) and temp (bit0)
		self._write_reg(_MPU6050_SIGNAL_PATH_RESET, 0x07) # 0x07 =b00000111
		# wait
		time.sleep(0.1) # Not sure how long is needed
		# Wake up IMU
		self._sleep_device(False)

	def _sleep_device(self, value):
		"""
		_sleep_devices or wakes up the device
		True - puts the device to _sleep_device
		False - wakes the device up
		"""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_PWR_MGMT_1)
		if(value == True): # _sleep_device
			# Set bit 6, the _sleep_device bit, to 1
			new_register_value = self._set_bit(cur_reg_reading, 6)
		elif(value == False): # Wake up
			# Set bit 6, the _sleep_device bit, to 0
			new_register_value = self._clear_bit(cur_reg_reading, 6)
		else:
			print("__sleep_device command failed, True or False only")
			return -1
		# Write the new register
		self._write_reg(_MPU6050_PWR_MGMT_1, new_register_value)
		# wait
		time.sleep(0.1) # Not sure how long is needed

	def set_sample_rate_divisor(self, value):
		"""
		Sets the clock rate with the following formula:
		rate = clock/(1 + value)
		"""
		if value > 255:
			value = 255
		if value < 0:
			value = 0
		self._write_reg(_MPU6050_SMPLRT_DIV, value)

	def set_filter_bandwidth(self, value):
		"""
		Sets the DLPF for the gyro
		Digital Low Pass Filter (DLPF) values are the bottom 3 bits of the CONFIG register
		See page 13 of Register Map for explaination of values
		0 should be no filter
		"""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_CONFIG)
		# Set the new register value
		new_register_value = cur_reg_reading | (value & 0x07)
		# Write the new register
		self._write_reg(_MPU6050_CONFIG, new_register_value)

	def set_gyro_range(self, value):
		"""
		Sets the full scale range of the guro in deg/s
		Full Scale range is bits 3:4 in GYRO_CONFIG register
		0 - 250 deg/s
		1 - 500 deg/s
		2 - 1000 deg/s
		3 - 2000 deg/s
		"""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_GYRO_CONFIG)
		# Set the new register value
		new_value = cur_reg_reading | ((value & 0x03) << 3)
		# Write the new register
		self._write_reg(_MPU6050_GYRO_CONFIG, new_value)

	def set_accelerometer_range(self, value):
		"""
		Sets the full scale range of the accelerometer in g
		Full Scale range is bits 3:4 in ACCEL_CONFIG register
		0 - 2g
		1 - 4g
		2 - 8g
		3 - 16g
		"""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_ACCEL_CONFIG)
		# Set the new register value
		new_value = cur_reg_reading | ((value & 0x03) << 3)
		# Write the new register
		self._write_reg(_MPU6050_ACCEL_CONFIG, new_value)

	def set_clock(self, value):
		"""
		Sets the source of the clock for the device
		Bits 0:2 of _MPU6050_PWR_MGMT_1
		See page 40 of Register Map for explaination of values
		1 - PLL with X axis gyroscope reference
		A gyroscope is recommended over internal clock for stability
		"""
		# Read the current value
		cur_reg_reading = self._read_reg(_MPU6050_PWR_MGMT_1)
		# Set bits0:2, to value
		new_register_value = cur_reg_reading | (value & 0x07)
		# Write the new register
		self._write_reg(_MPU6050_PWR_MGMT_1, new_register_value)

	def set_offsets(self, offsets):
		"""Used to calibrate the device.  offsets come from calibration code"""
		"""Offsets are passed in as a list [AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ]"""
		_buffer = [offsets[0]>>8, offsets[0] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_ACCEL_X_OFFSET_HIGH_ADDRESS, _buffer)
		_buffer = [offsets[1]>>8, offsets[1] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_ACCEL_Y_OFFSET_HIGH_ADDRESS, _buffer)
		_buffer = [offsets[2]>>8, offsets[2] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_ACCEL_Z_OFFSET_HIGH_ADDRESS, _buffer)
		_buffer = [offsets[3]>>8, offsets[3] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_GYRO_X_OFFSET_HIGH_ADDRESS, _buffer)
		_buffer = [offsets[4]>>8, offsets[4] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_GYRO_Y_OFFSET_HIGH_ADDRESS, _buffer)
		_buffer = [offsets[5]>>8, offsets[5] & 0xFF]
		self._bus.write_i2c_block_data(self._address, _MPU6050_GYRO_Z_OFFSET_HIGH_ADDRESS, _buffer)

	def read_imu7(self):
		"""
		Obtain all 7 data readings from the imu and return as a list
		"""
		accel_scale = _MPU6050_ACCEL_SCALE[self._cur_accel_setting]
		gyro_scale = _MPU6050_GYRO_SCALE[self._cur_gyro_setting]

		# Read data sequentially from start of block register
		read_buffer = self._bus.read_i2c_block_data(self._address, _MPU6050_ACCEL_XOUT_H, 14)
		rawAccX = read_buffer[0] << 8 | read_buffer[1]
		rawAccY = read_buffer[2] << 8 | read_buffer[3]
		rawAccZ = read_buffer[4] << 8 | read_buffer[5]
		rawTemp = read_buffer[6] << 8 | read_buffer[7]
		rawGyroX = read_buffer[8] << 8 | read_buffer[9]
		rawGyroY = read_buffer[10] << 8 | read_buffer[11]
		rawGyroZ = read_buffer[12] << 8 | read_buffer[13]

		temperature = (np.int16(rawTemp) / 340.0) + 36.53

		accX = (np.int16(rawAccX) / accel_scale)*_MPU6050_STANDARD_GRAVITY
		accY = (np.int16(rawAccY) / accel_scale)*_MPU6050_STANDARD_GRAVITY
		accZ = (np.int16(rawAccZ) / accel_scale)*_MPU6050_STANDARD_GRAVITY

		gyroX = (np.int16(rawGyroX)) / gyro_scale
		gyroY = (np.int16(rawGyroY)) / gyro_scale
		gyroZ = (np.int16(rawGyroZ)) / gyro_scale

		return [accX, accY, accZ, gyroX, gyroY, gyroZ, temperature]

	def read_accel(self):
		"""
		Obtain accelerometer readings from imu and return as a list
		"""
		# Read data sequentially from start of block register
		accel_scale = _MPU6050_ACCEL_SCALE[self._cur_accel_setting]
		read_buffer = self._bus.read_i2c_block_data(self._address, _MPU6050_ACCEL_XOUT_H, 6)
		rawAccX = read_buffer[0] << 8 | read_buffer[1]
		rawAccY = read_buffer[2] << 8 | read_buffer[3]
		rawAccZ = read_buffer[4] << 8 | read_buffer[5]

		accX = (np.int16(rawAccX) / accel_scale)*_MPU6050_STANDARD_GRAVITY
		accY = (np.int16(rawAccY) / accel_scale)*_MPU6050_STANDARD_GRAVITY
		accZ = (np.int16(rawAccZ) / accel_scale)*_MPU6050_STANDARD_GRAVITY

		return [accX, accY, accZ]

	def read_gyro(self):
		"""
		Obtain gyroscope readings from imu and return as a list
		"""
		# Read data sequentially from start of block register
		gyro_scale = _MPU6050_GYRO_SCALE[self._cur_gyro_setting]
		read_buffer = self._bus.read_i2c_block_data(self._address, _MPU6050_GYRO_XOUT_H, 6)
		rawGyroX = read_buffer[0] << 8 | read_buffer[1]
		rawGyroY = read_buffer[2] << 8 | read_buffer[3]
		rawGyroZ = read_buffer[4] << 8 | read_buffer[5]

		gyroX = (np.int16(rawGyroX)) / gyro_scale
		gyroY = (np.int16(rawGyroY)) / gyro_scale
		gyroZ = (np.int16(rawGyroZ)) / gyro_scale

		return [gyroX, gyroY, gyroZ]

	def read_temp_c(self):
		"""
		Obtain temperature readings from imu and return as a value in celsius
		"""
		read_buffer = self._bus.read_i2c_block_data(self._address, _MPU6050_TEMP_OUT_H, 2)
		rawTemp = read_buffer[0] << 8 | read_buffer[1]

		temperature = (np.int16(rawTemp) / 340.0) + 36.53
		return temperature

	def _test_bit(self, value, offset):
		"""Returns a nonzero value if bit at offset is set"""
		return value & (1 << offset)

	def _set_bit(self, value, offset):
		"""Sets the bit at the offset regardless of prior value.  Returns new value"""
		return value | (1 << offset)

	def _clear_bit(self, value, offset):
		"""Clears the bit at the offset regardless of prior value.  Returns new value"""
		return value & (~(1 << offset))

