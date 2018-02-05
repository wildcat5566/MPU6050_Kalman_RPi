#!/usr/bin/python

import smbus
import math
import time
import numpy as np
import kalman
import argparse

#power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
#IMU sensitivity parameters
gyro_sen = 131.0
acc_sen = 16384.0
#IMU offsets (updated 170905)
gx_offset = -502.42
gy_offset = 248.72
gz_offset = 15.75
ax_offset = 140.79
ay_offset = 122.76
az_offset = 1219.97
# 1219.97 = 17603.97(calibrated az_offset) - 16384(acc sensitivity)

def read_byte(adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val

def get_pitch(x, y, z): #y-rotation
	radians = math.atan2(x, math.sqrt(y*y + z*z))
	return math.degrees(radians)

def get_roll(x, y, z): #x-rotation
	radians = -math.atan2(y, z)
	return math.degrees(radians)

bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, power_mgmt_1, 0)

#Initialize kalman objects for RPY
Roll = kalman.Kalman()
Pitch = kalman.Kalman()
Yaw = kalman.Kalman()

#make initial guesses
time_pre = time.time()

ax_raw = read_word_2c(0x3b)
ay_raw = read_word_2c(0x3d)
az_raw = read_word_2c(0x3f)

ax = (ax_raw - ax_offset) / acc_sen
ay = (ay_raw - ay_offset) / acc_sen
az = (az_raw - az_offset) / acc_sen

#Initial guesses
Roll.setKalmanAngle(get_roll(ax, ay, az))
Pitch.setKalmanAngle(get_pitch(ax, ay, az))
Yaw.setKalmanAngle(0.)

while True:

	gx_raw = read_word_2c(0x43)
	gy_raw = read_word_2c(0x45)
	gz_raw = read_word_2c(0x47)

	gx = (gx_raw - gx_offset) / gyro_sen
	gy = (gy_raw - gy_offset) / gyro_sen
	gz = (gz_raw - gz_offset) / gyro_sen

	ax_raw = read_word_2c(0x3b)
	ay_raw = read_word_2c(0x3d)
	az_raw = read_word_2c(0x3f)

	ax = (ax_raw - ax_offset) / acc_sen
	ay = (ay_raw - ay_offset) / acc_sen
	az = (az_raw - az_offset) / acc_sen

	dt = time.time() - time_pre
	#print('dt: \t', dt)

	R = Roll.getKalmanAngle(get_roll(ax, ay, az), gx, dt)
	#print('Roll: \t', R)
	P = Pitch.getKalmanAngle(get_pitch(ax, ay, az), gy, dt)
	#print('Pitch: \t', P)
	print(gz)

	time_pre = time.time()

