#include "RTIMULSM6DSOXandLIS3MDL.h"
#include "RTIMUSettings.h"
#include <stdio.h>

RTIMULSM6DSOXandLIS3MDL::RTIMULSM6DSOXandLIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 104;  // Default to 104 Hz, adjust based on config
    
    // Write directly to stderr to test output
    HAL_INFO("\n=== RTIMULSM6DSOXandLIS3MDL CONSTRUCTOR CALLED ===\n");
}

RTIMULSM6DSOXandLIS3MDL::~RTIMULSM6DSOXandLIS3MDL()
{
    HAL_INFO("\n=== RTIMULSM6DSOXandLIS3MDL DESTRUCTOR CALLED ===\n");
}

bool RTIMULSM6DSOXandLIS3MDL::IMUInit()
{
    HAL_INFO("\n=== RTIMULSM6DSOXandLIS3MDL IMUInit CALLED ===\n");
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

    HAL_INFO1("LSM6DSOX address: 0x%02x\n", m_lsm6dsoxAddr);
    HAL_INFO1("LIS3MDL address: 0x%02x\n", m_lis3mdlAddr);

    setCalibrationData();  // Load calibration data from settings
    HAL_INFO("Calibration data loaded\n");

    // Enable I2C bus
    HAL_INFO("Opening HAL...\n");
    if (!m_settings->HALOpen())
    {
        HAL_ERROR("Failed to open HAL\n");
        return false;
    }
    HAL_INFO("HAL opened successfully\n");

    // Initialize LSM6DSOX (gyro + accel)
    HAL_INFO("Reading LSM6DSOX WHOAMI...\n");
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_WHOAMI, 1, &result, "Failed to read LSM6DSOX WHOAMI"))
    {
        HAL_ERROR("Could not read LSM6DSOX WHOAMI register\n");
        return false;
    }
    HAL_INFO2("LSM6DSOX WHOAMI result: 0x%02x (expected 0x%02x)\n", result, LSM6DSOX_WHOAMI_VALUE);
    
    if (result != LSM6DSOX_WHOAMI_VALUE) {
        HAL_ERROR1("Incorrect LSM6DSOX id: got 0x%02x\n", result);
        return false;
    }
    HAL_INFO("LSM6DSOX verified\n");

    HAL_INFO("Configuring LSM6DSOX...\n");
    if (!setLSM6DSOXConfig())
    {
        HAL_ERROR("Failed to configure LSM6DSOX\n");
        return false;
    }
    HAL_INFO("LSM6DSOX configured\n");

    // Initialize LIS3MDL (magnetometer)
    HAL_INFO("Reading LIS3MDL WHOAMI...\n");
    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_WHO_AM_I, 1, &result, "Failed to read LIS3MDL WHOAMI"))
    {
        HAL_ERROR("Could not read LIS3MDL WHOAMI register\n");
        return false;
    }
    HAL_INFO2("LIS3MDL WHOAMI result: 0x%02x (expected 0x%02x)\n", result, LIS3MDL_REG_WHO_AM_I_VALUE);
    
    if (result != LIS3MDL_REG_WHO_AM_I_VALUE) {
        HAL_ERROR1("Incorrect LIS3MDL id: got 0x%02x\n", result);
        return false;
    }
    HAL_INFO("LIS3MDL verified\n");

    HAL_INFO("Configuring LIS3MDL...\n");
    if (!setLIS3MDLConfig())
    {
        HAL_ERROR("Failed to configure LIS3MDL\n");
        return false;
    }
    HAL_INFO("LIS3MDL configured\n");

    HAL_INFO("Initializing gyro bias...\n");
    gyroBiasInit();
    HAL_INFO("LSM6DSOX+LIS3MDL init complete\n");
    return true;
}

bool RTIMULSM6DSOXandLIS3MDL::setLSM6DSOXConfig()
{
    // in order to disable accelerometer high performance mode, CTRL6_C bit 5 must be set to 1

    HAL_INFO("  Setting LSM6DSOX CTRL3_C...\n");
    unsigned char ctrl3_c = LSM6DSOX_CTRL3_C_BDU_ENABLED | LSM6DSOX_CTRL3_C_INCREMENT_ENABLED;  // I2C enable, BDU on
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL3_C, ctrl3_c, "Failed to set LSM6DSOX CTRL3_C"))
    {
        HAL_ERROR("Failed to write CTRL3_C\n");
        return false;
    }
    HAL_INFO1("  CTRL3_C set successfully (0x%02x)\n", ctrl3_c);

    // Set accel: ±2G, 104 Hz (adapt ranges/data rates as needed)
    HAL_INFO("  Setting LSM6DSOX CTRL1_XL...\n");
    unsigned char lsm6dsox_odr = LSM6DSOX_ODR_104_HZ;
    updateGyroSampleRate(lsm6dsox_odr);
    unsigned char ctrl1_xl = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_2G | lsm6dsox_odr;  // 104 Hz, ±2G
    m_accelScale = 2.0 / 32768.0;  // Scale for ±2G
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DSOX CTRL1_XL"))
    {
        HAL_ERROR("Failed to write CTRL1_XL\n");
        return false;
    }
    HAL_INFO1("  CTRL1_XL set successfully (0x%02x)\n", ctrl1_xl);

    // Set gyro: ±250 DPS, 104 Hz
    HAL_INFO("  Setting LSM6DSOX CTRL2_G...\n");
    unsigned char ctrl2_g = LSM6DSOX_GYRO_FULLSCALE_250DPS | lsm6dsox_odr;  // Example: 104 Hz, ±250 DPS
    m_gyroScale = (250.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;  // Scale for ±250 DPS
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL2_G, ctrl2_g, "Failed to set LSM6DSOX CTRL2_G"))
    {
        HAL_ERROR("Failed to write CTRL2_G\n");
        return false;
    }
    HAL_INFO1("  CTRL2_G set successfully (0x%02x)\n", ctrl2_g);

    return true;
}

bool RTIMULSM6DSOXandLIS3MDL::setLIS3MDLConfig()
{

    HAL_INFO("  Setting LIS3MDL CTRL_REG1...\n");
    unsigned char ctrl1 = LIS3MDL_DATARATE_80_HZ | LIS3MDL_XY_MEDIUMMODE;  // 80 Hz, Medium performance
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG1, ctrl1, "Failed to set LIS3MDL CTRL_REG1"))
    {
        HAL_ERROR("Failed to write CTRL_REG1\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG1 set successfully (0x%02x)\n", ctrl1);

    HAL_INFO("  Setting LIS3MDL CTRL_REG2...\n");
    unsigned char ctrl2 = LIS3MDL_RANGE_4_GAUSS;  // ±4 Gauss (default)
    m_compassScale = 4.0 / 32768.0;  // Gauss to Gauss scale (convert to µT later)
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG2, ctrl2, "Failed to set LIS3MDL CTRL_REG2"))
    {
        HAL_ERROR("Failed to write CTRL_REG2\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG2 set successfully (0x%02x)\n", ctrl2);

    HAL_INFO("  Setting LIS3MDL CTRL_REG3...\n");
    unsigned char ctrl3 = LIS3MDL_CONTINUOUSMODE;  // Continuous mode
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG3, ctrl3, "Failed to set LIS3MDL CTRL_REG3"))
    {
        HAL_ERROR("Failed to write CTRL_REG3\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG3 set successfully (0x%02x)\n", ctrl3);

    HAL_INFO("  Setting LIS3MDL CTRL_REG4...\n");
    unsigned char ctrl4 = LIS3MDL_Z_MEDIUMMODE;  // Medium performance for Z
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG4, ctrl4, "Failed to set LIS3MDL CTRL_REG4"))
    {
        HAL_ERROR("Failed to write CTRL_REG4\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG4 set successfully (0x%02x)\n", ctrl4);

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

    HAL_INFO("IMURead called\n");

    // Check LSM6DSOX status
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_STATUS_REG, 1, &status, "Failed to read LSM6DSOX status"))
        return false;
    HAL_INFO1("LSM6DSOX status: 0x%02x\n", status);
    if ((status & 0x03) == 0)  // Check if gyro and accel data ready
    {
        HAL_INFO("LSM6DSOX data not ready\n");
        return false;
    }
    HAL_INFO("LSM6DSOX data ready\n");

    // Read gyro and accel data
    if (!m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_G, 12, gyroAccelData, "Failed to read LSM6DSOX data"))
        return false;
    HAL_INFO("Raw LSM6DSOX data: ");
    for (int i = 0; i < 12; i++) {
        HAL_INFO1("%02x ", gyroAccelData[i]);
    }
    HAL_INFO("\n");

    // Read compass data from LIS3MDL
    if (!m_settings->HALRead(m_lis3mdlAddr, 0x80 | LIS3MDL_REG_OUT_X_L, 6, compassData, "Failed to read LIS3MDL data"))
        return false;
    HAL_INFO("Raw LIS3MDL data: ");
    for (int i = 0; i < 6; i++) {
        HAL_INFO1("%02x ", compassData[i]);
    }
    HAL_INFO("\n");

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    // Convert gyro and accel
    RTMath::convertToVector(gyroAccelData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(gyroAccelData + 6, m_imuData.accel, m_accelScale, false);

    HAL_INFO3("Converted accel: %.3f, %.3f, %.3f\n", m_imuData.accel.x(), m_imuData.accel.y(), m_imuData.accel.z());
    HAL_INFO3("Converted gyro: %.3f, %.3f, %.3f\n", m_imuData.gyro.x(), m_imuData.gyro.y(), m_imuData.gyro.z());

    // Convert compass (to µT)
    m_imuData.compass.setX((RTFLOAT)((int16_t)((compassData[1] << 8) | compassData[0])) * m_compassScale * 100.0);
    m_imuData.compass.setY((RTFLOAT)((int16_t)((compassData[3] << 8) | compassData[2])) * m_compassScale * 100.0);
    m_imuData.compass.setZ((RTFLOAT)((int16_t)((compassData[5] << 8) | compassData[4])) * m_compassScale * 100.0);

    HAL_INFO3("Converted compass: %.3f, %.3f, %.3f\n", m_imuData.compass.x(), m_imuData.compass.y(), m_imuData.compass.z());

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

// Read back the control registers that we set during configuration
void RTIMULSM6DSOXandLIS3MDL::verifyConfigs()
{
    unsigned char val;

    HAL_INFO("Verifying LSM6DSOX registers...\n");
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL3_C, 1, &val, "Failed to read LSM6DSOX CTRL3_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL3_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL3_C = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL1_XL, 1, &val, "Failed to read LSM6DSOX CTRL1_XL"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL1_XL\n");
    }
    else
        HAL_INFO1("  CTRL1_XL = 0x%02x\n", val);

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL2_G, 1, &val, "Failed to read LSM6DSOX CTRL2_G"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL2_G\n");
    }
    else
    {
        HAL_INFO1("  CTRL2_G = 0x%02x\n", val);
    }

    HAL_INFO("Verifying LIS3MDL registers...\n");
    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG1, 1, &val, "Failed to read LIS3MDL CTRL_REG1"))
    {
        HAL_ERROR("Failed to read LIS3MDL CTRL_REG1\n");
    }
    else
    {
        HAL_INFO1("  CTRL_REG1 = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG2, 1, &val, "Failed to read LIS3MDL CTRL_REG2"))
    {
        HAL_ERROR("Failed to read LIS3MDL CTRL_REG2\n");
    }
    else
    {
        HAL_INFO1("  CTRL_REG2 = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG3, 1, &val, "Failed to read LIS3MDL CTRL_REG3"))
    {
        HAL_ERROR("Failed to read LIS3MDL CTRL_REG3\n");
    }
    else
    {
        HAL_INFO1("  CTRL_REG3 = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG4, 1, &val, "Failed to read LIS3MDL CTRL_REG4"))
    {
        HAL_ERROR("Failed to read LIS3MDL CTRL_REG4\n");
    }
    else
    {
        HAL_INFO1("  CTRL_REG4 = 0x%02x\n", val);
    }
}

// Dump raw data registers as a single-line: "<LSM6DSOX regs | LIS3MDL regs>"
void RTIMULSM6DSOXandLIS3MDL::dumpRawData()
{
    unsigned char buf[12];
    unsigned char mag[6];

    bool ok1 = m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_G, 12, buf, "Failed to read LSM6DSOX data");
    bool ok2 = m_settings->HALRead(m_lis3mdlAddr, 0x80 | LIS3MDL_REG_OUT_X_L, 6, mag, "Failed to read LIS3MDL data");

    HAL_INFO("<");
    if (ok1) {
        HAL_INFO("LSM6DSOX regs ");
        for (int i = 0; i < 12; i++) {
            HAL_INFO1("%02x ", buf[i]);
        }
    } else {
        HAL_INFO("LSM6DSOX read-fail ");
    }

    HAL_INFO("| ");

    if (ok2) {
        HAL_INFO("LIS3MDL regs ");
        for (int i = 0; i < 6; i++) {
            HAL_INFO1("%02x ", mag[i]);
        }
    } else {
        HAL_INFO("LIS3MDL read-fail ");
    }

    HAL_INFO(">\n");
}