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

# Add this after the compass data read
print("\n--- Testing 12-byte read from 0x22 (like dumpRawData) ---")
try:
    data_12 = bus.read_i2c_block_data(LSM6DSOX_ADDR, 0x80 | 0x22, 12)
    print(f"12-byte read from 0x22: {['0x{:02x}'.format(b) for b in data_12]}")
except Exception as e:
    print(f"Error: {e}")

# Also test direct read from 0x28 (accel only)
print("\n--- Testing accel data directly from 0x28 ---")
accel_direct = bus.read_i2c_block_data(LSM6DSOX_ADDR, 0x80 | 0x28, 6)
print(f"6-byte read from 0x28: {['0x{:02x}'.format(b) for b in accel_direct]}")