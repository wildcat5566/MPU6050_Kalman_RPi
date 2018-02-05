#!/usr/bin/python
# MPU6050 calibration

import smbus
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--iterations", type = int)
args = parser.parse_args()

if args.iterations:
	iterations = args.iterations
	print("iterations: \t", iterations)

else:
	iterations = 1000
	print("iterations(default): \t", iterations) 
	
#power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

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

bus = smbus.SMBus(1)
address = 0x68

bus.write_byte_data(address, power_mgmt_1, 0)
count = 0
gyrox_list = []
gyroy_list = []
gyroz_list = []
accx_list = []
accy_list = []
accz_list = []
while count < iterations:

	gyro_x_raw = read_word_2c(0x43)
	gyro_y_raw = read_word_2c(0x45)
	gyro_z_raw = read_word_2c(0x47)

	acc_x_raw = read_word_2c(0x3b)
	acc_y_raw = read_word_2c(0x3d)
	acc_z_raw = read_word_2c(0x3f)

	gyrox_list.append(gyro_x_raw)
	gyroy_list.append(gyro_y_raw)
	gyroz_list.append(gyro_z_raw)

	accx_list.append(acc_x_raw)
	accy_list.append(acc_y_raw)
	accz_list.append(acc_z_raw)

	#print("acc_x_raw: \t", acc_x_raw)
	#print("acc_y_raw: \t", acc_y_raw)
	#print("acc_z_raw: \t", acc_z_raw)
	count = count + 1

gyrox_mean = sum(gyrox_list) / len(gyrox_list)
gyroy_mean = sum(gyroy_list) / len(gyroy_list)
gyroz_mean = sum(gyroz_list) / len(gyroz_list)
accx_mean = sum(accx_list) / len(accx_list)
accy_mean = sum(accy_list) / len(accy_list)
accz_mean = sum(accz_list) / len(accz_list)

print("gyrox_mean: \t", gyrox_mean)
print("gyroy_mean: \t", gyroy_mean)
print("gyroz_mean: \t", gyroz_mean)
print("accx_mean: \t", accx_mean)
print("accy_mean: \t", accy_mean)
print("accz_mean: \t", accz_mean)
