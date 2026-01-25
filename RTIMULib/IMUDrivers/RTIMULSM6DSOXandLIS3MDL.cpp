#include "RTIMULSM6DSOXandLIS3MDL.h"
#include "RTIMUSettings.h"

RTIMULSM6DSOXandLIS3MDL::RTIMULSM6DSOXandLIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 104;  // Default to 104 Hz, adjust based on config
}

RTIMULSM6DSOXandLIS3MDL::~RTIMULSM6DSOXandLIS3MDL()
{
}

bool RTIMULSM6DSOXandLIS3MDL::IMUInit()
{
    unsigned char result;

    // Set validity flags
    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    // Set I2C addresses (use defaults or from settings)
    m_lsm6dsoxAddr = LSM6DSOX_I2CADDR_DEFAULT;
    m_lis3mdlAddr = LIS3MDL_I2CADDR_DEFAULT;

    setCalibrationData();  // Load calibration data from settings

    // Enable I2C bus
    if (!m_settings->HALOpen())
        return false;

    // Initialize LSM6DSOX (gyro + accel)
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_WHOAMI, 1, &result, "Failed to read LSM6DSOX WHOAMI"))
        return false;
    if (result != LSM6DSOX_WHOAMI_VALUE) {
        HAL_ERROR1("Incorrect LSM6DSOX id %d\n", result);
        return false;
    }

    if (!setLSM6DSOXConfig())
        return false;

    // Initialize LIS3MDL (magnetometer)
    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_WHO_AM_I, 1, &result, "Failed to read LIS3MDL WHOAMI"))
        return false;
    if (result != LIS3MDL_REG_WHO_AM_I_VALUE) {
        HAL_ERROR1("Incorrect LIS3MDL id %d\n", result);
        return false;
    }

    if (!setLIS3MDLConfig())
        return false;

    gyroBiasInit();
    HAL_INFO("LSM6DSOX+LIS3MDL init complete\n");
    return true;
}

bool RTIMULSM6DSOXandLIS3MDL::setLSM6DSOXConfig()
{
    // in order to disable accelerometer high performance mode, CTRL6_C bit 5 must be set to 1

    unsigned char ctrl3_c = LSM6DSOX_CTRL3_C_BDU_ENABLED | LSM6DSOX_CTRL3_C_INCREMENT_ENABLED;  // I2C enable, BDU on
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL3_C, ctrl3_c, "Failed to set LSM6DSOX CTRL3_C"))
        return false;

    // Set accel: ±2G, 104 Hz (adapt ranges/data rates as needed)
    unsigned char lsm6dsox_odr = LSM6DSOX_ODR_104_HZ;
    updateGyroSampleRate(lsm6dsox_odr);
    unsigned char ctrl1_xl = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_2G | lsm6dsox_odr;  // 104 Hz, ±2G
    m_accelScale = 2.0 / 32768.0;  // Scale for ±2G
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DSOX CTRL1_XL"))
        return false;

    // Set gyro: ±250 DPS, 104 Hz
    unsigned char ctrl2_g = LSM6DSOX_GYRO_FULLSCALE_250DPS | lsm6dsox_odr;  // Example: 104 Hz, ±250 DPS
    m_gyroScale = (250.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;  // Scale for ±250 DPS
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL2_G, ctrl2_g, "Failed to set LSM6DSOX CTRL2_G"))
        return false;

    return true;
}

bool RTIMULSM6DSOXandLIS3MDL::setLIS3MDLConfig()
{

    unsigned char ctrl1 = LIS3MDL_DATARATE_80_HZ | LIS3MDL_XY_MEDIUMMODE;  // 80 Hz, Medium performance
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG1, ctrl1, "Failed to set LIS3MDL CTRL_REG1"))
        return false;

    unsigned char ctrl2 = LIS3MDL_RANGE_4_GAUSS;  // ±4 Gauss (default)
    m_compassScale = 4.0 / 32768.0;  // Gauss to Gauss scale (convert to µT later)
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG2, ctrl2, "Failed to set LIS3MDL CTRL_REG2"))
        return false;

    unsigned char ctrl3 = LIS3MDL_CONTINUOUSMODE;  // Continuous mode
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG3, ctrl3, "Failed to set LIS3MDL CTRL_REG3"))
        return false;

    unsigned char ctrl4 = LIS3MDL_Z_MEDIUMMODE;  // Medium performance for Z
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG4, ctrl4, "Failed to set LIS3MDL CTRL_REG4"))
        return false;

    return true;
}

void RTIMULSM6DSOXandLIS3MDL::updateGyroSampleRate(
    unsigned char lsm6dsox_odr,
    bool low_power_mode)
{
    switch (lsm6dsox_odr)
    {
    case LSM6DSOX_ODR_12_5_HZ:
        m_sampleRate = 13;
        break;
    case LSM6DSOX_ODR_26_HZ:
        m_sampleRate = 26;
        break;
    case LSM6DSOX_ODR_52_HZ:
        m_sampleRate = 52;
        break;
    case LSM6DSOX_ODR_104_HZ:
        m_sampleRate = 104;
        break;
    case LSM6DSOX_ODR_208_HZ:
        m_sampleRate = 208;
        break;
    case LSM6DSOX_ODR_416_HZ:
        m_sampleRate = 416;
        break;
    case LSM6DSOX_ODR_833_HZ:
        m_sampleRate = 833;
        break;
    case LSM6DSOX_ODR_1_66_kHZ:
        m_sampleRate = 1660;
        break;
    case LSM6DSOX_ODR_3_33_kHZ:
        m_sampleRate = 3330;
        break;
    case LSM6DSOX_ODR_6_66_kHZ:
        m_sampleRate = 6660;
        break;
    case LSM6DSOX_ODR_1_6_HZ:
        m_sampleRate = (true == low_power_mode) ? 1 : 13;
        break;
    default:
        break;
    }
}

bool RTIMULSM6DSOXandLIS3MDL::IMURead()
{
    unsigned char status;
    unsigned char gyroAccelData[12];  // Gyro (6) + Accel (6)
    unsigned char compassData[6];

    // Check LSM6DSOX status
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_STATUS_REG, 1, &status, "Failed to read LSM6DSOX status"))
        return false;
    if ((status & 0x03) == 0)  // Check if gyro and accel data ready
        return false;

    // Read gyro and accel data
    if (!m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_G, 12, gyroAccelData, "Failed to read LSM6DSOX data"))
        return false;

    // Read compass data from LIS3MDL
    if (!m_settings->HALRead(m_lis3mdlAddr, 0x80 | LIS3MDL_REG_OUT_X_L, 6, compassData, "Failed to read LIS3MDL data"))
        return false;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    // Convert gyro and accel
    RTMath::convertToVector(gyroAccelData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(gyroAccelData + 6, m_imuData.accel, m_accelScale, false);

    // Convert compass (to µT)
    m_imuData.compass.setX((RTFLOAT)((int16_t)((compassData[1] << 8) | compassData[0])) * m_compassScale * 100.0);
    m_imuData.compass.setY((RTFLOAT)((int16_t)((compassData[3] << 8) | compassData[2])) * m_compassScale * 100.0);
    m_imuData.compass.setZ((RTFLOAT)((int16_t)((compassData[5] << 8) | compassData[4])) * m_compassScale * 100.0);

    // Apply axis corrections if needed (based on board orientation)

    // Standard processing
    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();
    updateFusion();

    return true;
}

int RTIMULSM6DSOXandLIS3MDL::IMUGetPollInterval()
{
    return 1000 / m_sampleRate;  // e.g., ~10ms for 100 Hz
}