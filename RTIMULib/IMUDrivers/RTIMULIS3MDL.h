#ifndef _RTIMULIS3MDL_H
#define _RTIMULIS3MDL_H

#include "RTIMU.h"

class RTIMULIS3MDL : public RTIMU
{
public:
    RTIMULIS3MDL(RTIMUSettings *settings);
    ~RTIMULIS3MDL();

    // Pure virtual methods you MUST implement:
    virtual const char *IMUName() { return "LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LIS3MDL; }
    virtual bool IMUInit();        // Initialize the sensor
    virtual bool IMURead();        // Read sensor data
    virtual int IMUGetPollInterval(); // Return poll interval in milliseconds

private:
    // Add any private helper methods you need
    bool setSampleRate();
    bool setGyroConfig();
    // ... other setup methods based on your IMU's features

    unsigned char m_slaveAddr;    // I2C address
    // ... other member variables for state
};

#endif