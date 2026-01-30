#ifndef _RTIMULSM6DSOXandLIS3MDL_H
#define _RTIMULSM6DSOXandLIS3MDL_H

#include "RTIMU.h"

class RTIMULSM6DSOXandLIS3MDL : public RTIMU
{
public:
    RTIMULSM6DSOXandLIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DSOXandLIS3MDL();

    // Pure virtual methods you MUST implement:
    virtual const char *IMUName() { return "LSM6DSOX+LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DSOX_LIS3MDL; }  // Define this in RTIMUDefs.h
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();
    // Read back control registers for verification after init
    void verifyConfigs();
    // Dump raw sensor data registers (single-line output)
    void dumpRawData();

private:
    // Private methods for setup
    bool setLSM6DSOXConfig();
    bool setLIS3MDLConfig();
    void updateGyroSampleRate(unsigned char lsm6dsox_odr, bool low_power_mode = false);
    // Helpers to map settings values to register bits and scales
    unsigned char mapLSM6DSOXSampleRateToODR(int sampleRate);
    void mapLSM6DSOXAccelFsrToBitsAndScale(int accelFsr, unsigned char &bits, RTFLOAT &scale);
    void mapLSM6DSOXGyroFsrToBitsAndScale(int gyroFsr, unsigned char &bits, RTFLOAT &scale);

    unsigned char mapLIS3MDLDataRateToBits(int dataRate);
    unsigned char mapLIS3MDLXYPerfToBits(int xyPerf);
    unsigned char mapLIS3MDLZPerfToBits(int zPerf);
    void mapLIS3MDLRangeToBitsAndScale(int range, unsigned char &bits, RTFLOAT &scale);
    unsigned char mapLIS3MDLOpModeToBits(int opMode);

    // Member variables
    unsigned char m_lsm6dsoxAddr;  // I2C address for LSM6DSOX
    unsigned char m_lis3mdlAddr;   // I2C address for LIS3MDL
    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif