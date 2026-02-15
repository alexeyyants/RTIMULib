#!/usr/bin/env python3

import sys
import os
import time
import RTIMU

# Settings file
SETTINGS_FILE = "LSM6DSOX_LIS3MDL_Config"

# Create settings
s = RTIMU.Settings(SETTINGS_FILE)

# Create IMU (it will use auto-discovery or the settings file to detect the type)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if not imu.IMUInit():
    print("IMU Init Failed")
    sys.exit(1)

print("IMU Init Succeeded")

# Enable calibration modes
imu.setAccelCalibrationMode(True)
imu.setCompassCalibrationMode(True)

print("Starting calibration. Move the IMU in all directions...")
print("Press Ctrl+C to stop and save calibration.")

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

try:
    doit = True
    while doit:
        if imu.IMURead():
            # Data is being collected automatically in calibration mode
            data = imu.getIMUData()
            accel = data["accel"]
            compass = data["compass"]
            print("Accel: %.3f %.3f %.3f | Compass: %.3f %.3f %.3f" % (
                accel[0], accel[1], accel[2],
                compass[0], compass[1], compass[2]))
        time.sleep(poll_interval / 1000.0)
        doit = False  # Remove this line to run indefinitely
except KeyboardInterrupt:
    print("\nCalibration stopped. Saving settings...")
    s.saveSettings()
    print("Calibration saved to " + SETTINGS_FILE + ".ini")
