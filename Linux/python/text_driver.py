#!/usr/bin/env python3
import smbus
import time

bus = smbus.SMBus(1)  # I2C bus 1

LSM6DSOX_ADDR = 0x6A
LIS3MDL_ADDR = 0x1C

# Read LSM6DSOX WHO_AM_I
whoami = bus.read_byte_data(LSM6DSOX_ADDR, 0x0F)
print(f"LSM6DSOX WHO_AM_I: 0x{whoami:02x}")

# Read LIS3MDL WHO_AM_I  
whoami = bus.read_byte_data(LIS3MDL_ADDR, 0x0F)
print(f"LIS3MDL WHO_AM_I: 0x{whoami:02x}")

# Read LSM6DSOX status
status = bus.read_byte_data(LSM6DSOX_ADDR, 0x1E)
print(f"LSM6DSOX Status: 0x{status:02x}")

# Read accel data (OUTX_L_XL to OUTZ_H_XL)
accel_data = []
for reg in range(0x28, 0x2E):
    accel_data.append(bus.read_byte_data(LSM6DSOX_ADDR, reg))
print(f"Accel raw bytes: {['0x{:02x}'.format(b) for b in accel_data]}")

# Read compass data
compass_data = []
for reg in range(0x28, 0x2E):
    compass_data.append(bus.read_byte_data(LIS3MDL_ADDR, reg))
print(f"Compass raw bytes: {['0x{:02x}'.format(b) for b in compass_data]}")