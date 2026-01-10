#include "RTIMULIS3MDL.h"
#include "RTIMUSettings.h"

RTIMULIS3MDL::RTIMULIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULIS3MDL::~RTIMULIS3MDL()
{
}

bool RTIMULIS3MDL::IMUInit()
{
    unsigned char whoami;
    
    // Verify communication
    if (!m_settings->m_I2CSlaveAddress)
        m_settings->m_I2CSlaveAddress = NEWBOARD_ADDRESS; // default address
    
    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    
    // Check WHO_AM_I register
    if (!m_hal->I2CRead(m_slaveAddr, NEWBOARD_WHOAMI_REG, 1, &whoami))
        return false;
        
    if (whoami != NEWBOARD_WHOAMI_VALUE)
        return false;
    
    // Initialize sensors
    if (!setGyroConfig() || !setAccelConfig() || !setSampleRate())
        return false;
    
    m_firstTime = true;
    return true;
}

bool RTIMULIS3MDL::IMURead()
{
    unsigned char data[20];  // adjust size based on register layout
    
    // Read sensor data
    if (!m_hal->I2CRead(m_slaveAddr, NEWBOARD_DATA_START_REG, sizeof(data), data))
        return false;
    
    // Convert raw bytes to sensor readings
    // Parse gyro: typically 3 16-bit signed values
    int16_t gx = (data[0] << 8) | data[1];
    int16_t gy = (data[2] << 8) | data[3];
    int16_t gz = (data[4] << 8) | data[5];
    
    // Parse accel: typically 3 16-bit signed values
    int16_t ax = (data[6] << 8) | data[7];
    int16_t ay = (data[8] << 8) | data[9];
    int16_t az = (data[10] << 8) | data[11];
    
    // Parse magnetometer: typically 3 16-bit signed values
    int16_t mx = (data[12] << 8) | data[13];
    int16_t my = (data[14] << 8) | data[15];
    int16_t mz = (data[16] << 8) | data[17];
    
    // Convert to appropriate scale (depends on FSR settings)
    // For example, if gyro FSR is ±2000 dps with 16-bit range:
    RTFLOAT gyroScale = (2000.0f / 32768.0f) * RTMATH_DEG_TO_RAD;
    m_imuData.gyro.setX(gx * gyroScale);
    m_imuData.gyro.setY(gy * gyroScale);
    m_imuData.gyro.setZ(gz * gyroScale);
    
    // Similar for accel and magnetometer
    // ...
    
    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    return true;
}

int RTIMULIS3MDL::IMUGetPollInterval()
{
    return 0;
}

bool RTIMULIS3MDL::setSampleRate()
{
    return false;
}

bool RTIMULIS3MDL::setGyroConfig()
{
    return false;
}
