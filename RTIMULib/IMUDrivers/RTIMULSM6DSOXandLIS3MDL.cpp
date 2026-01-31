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
    unsigned char ctrl3_c = LSM6DSOX_CTRL3_C_BDU_DISABLED | LSM6DSOX_CTRL3_C_INCREMENT_ENABLED;  // I2C enable, continuous update, auto-increment
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL3_C, ctrl3_c, "Failed to set LSM6DSOX CTRL3_C"))
    {
        HAL_ERROR("Failed to write CTRL3_C\n");
        return false;
    }
    HAL_INFO1("  CTRL3_C set successfully (0x%02x)\n", ctrl3_c);

    // Get sample rate and fullscale from settings
    int sampleRate = m_settings->m_LSM6DSOXAccelSampleRate;
    int accelFsr = m_settings->m_LSM6DSOXAccelFsr;
    int gyroFsr = m_settings->m_LSM6DSOXGyroFsr;

    unsigned char lsm6dsox_odr = mapLSM6DSOXSampleRateToODR(sampleRate);
    updateGyroSampleRate(lsm6dsox_odr);

    unsigned char accelFsrBits;
    mapLSM6DSOXAccelFsrToBitsAndScale(accelFsr, accelFsrBits, m_accelScale);

    HAL_INFO("  Setting LSM6DSOX CTRL1_XL...\n");
    unsigned char ctrl1_xl = accelFsrBits | lsm6dsox_odr;
    if (!m_settings->HALWrite(m_lsm6dsoxAddr, LSM6DSOX_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DSOX CTRL1_XL"))
    {
        HAL_ERROR("Failed to write CTRL1_XL\n");
        return false;
    }
    HAL_INFO1("  CTRL1_XL set successfully (0x%02x)\n", ctrl1_xl);

    unsigned char gyroFsrBits;
    mapLSM6DSOXGyroFsrToBitsAndScale(gyroFsr, gyroFsrBits, m_gyroScale);

    HAL_INFO("  Setting LSM6DSOX CTRL2_G...\n");
    unsigned char ctrl2_g = gyroFsrBits | lsm6dsox_odr;
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
    // Get settings from RTIMUSettings
    int dataRate = m_settings->m_LIS3MDLCompassSampleRate;
    int range = m_settings->m_LIS3MDLCompassFsr;
    int xyPerf = m_settings->m_LIS3MDLXYPerformance;
    int zPerf = m_settings->m_LIS3MDLZPerformance;
    int opMode = m_settings->m_LIS3MDLOperationMode;

    // Map data rate to register bits
    unsigned char dataRateBits = mapLIS3MDLDataRateToBits(dataRate);

    // Map XY performance mode
    unsigned char xyPerfBits = mapLIS3MDLXYPerfToBits(xyPerf);

    HAL_INFO("  Setting LIS3MDL CTRL_REG1...\n");
    unsigned char ctrl1 = dataRateBits | xyPerfBits;
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG1, ctrl1, "Failed to set LIS3MDL CTRL_REG1"))
    {
        HAL_ERROR("Failed to write CTRL_REG1\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG1 set successfully (0x%02x)\n", ctrl1);

    // Map range to register bits
    unsigned char rangeBits;
    mapLIS3MDLRangeToBitsAndScale(range, rangeBits, m_compassScale);

    HAL_INFO("  Setting LIS3MDL CTRL_REG2...\n");
    unsigned char ctrl2 = rangeBits;
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG2, ctrl2, "Failed to set LIS3MDL CTRL_REG2"))
    {
        HAL_ERROR("Failed to write CTRL_REG2\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG2 set successfully (0x%02x)\n", ctrl2);

    // Map operation mode to register bits
    unsigned char modeBits = mapLIS3MDLOpModeToBits(opMode);

    HAL_INFO("  Setting LIS3MDL CTRL_REG3...\n");
    unsigned char ctrl3 = modeBits;
    if (!m_settings->HALWrite(m_lis3mdlAddr, LIS3MDL_REG_CTRL_REG3, ctrl3, "Failed to set LIS3MDL CTRL_REG3"))
    {
        HAL_ERROR("Failed to write CTRL_REG3\n");
        return false;
    }
    HAL_INFO1("  CTRL_REG3 set successfully (0x%02x)\n", ctrl3);

    // Map Z performance mode
    unsigned char zPerfBits = mapLIS3MDLZPerfToBits(zPerf);

    HAL_INFO("  Setting LIS3MDL CTRL_REG4...\n");
    unsigned char ctrl4 = zPerfBits;
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

// Helper implementations
unsigned char RTIMULSM6DSOXandLIS3MDL::mapLSM6DSOXSampleRateToODR(int sampleRate)
{
    switch (sampleRate)
    {
    case 12:
        return LSM6DSOX_ODR_12_5_HZ;
    case 26:
        return LSM6DSOX_ODR_26_HZ;
    case 52:
        return LSM6DSOX_ODR_52_HZ;
    case 104:
        return LSM6DSOX_ODR_104_HZ;
    case 208:
        return LSM6DSOX_ODR_208_HZ;
    case 416:
        return LSM6DSOX_ODR_416_HZ;
    case 833:
        return LSM6DSOX_ODR_833_HZ;
    case 1660:
        return LSM6DSOX_ODR_1_66_kHZ;
    case 3330:
        return LSM6DSOX_ODR_3_33_kHZ;
    case 6660:
        return LSM6DSOX_ODR_6_66_kHZ;
    default:
        HAL_INFO1("  Unknown sample rate %d, using 104 Hz\n", sampleRate);
        return LSM6DSOX_ODR_104_HZ;
    }
}

void RTIMULSM6DSOXandLIS3MDL::mapLSM6DSOXAccelFsrToBitsAndScale(int accelFsr, unsigned char &bits, RTFLOAT &scale)
{
    switch (accelFsr)
    {
    case 2:
        bits = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_2G;
        scale = 2.0 / 32768.0;
        break;
    case 4:
        bits = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_4G;
        scale = 4.0 / 32768.0;
        break;
    case 8:
        bits = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_8G;
        scale = 8.0 / 32768.0;
        break;
    case 16:
        bits = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_16G;
        scale = 16.0 / 32768.0;
        break;
    default:
        HAL_INFO1("  Unknown accel fullscale %d, using ±2G\n", accelFsr);
        bits = LSM6DSOX_ACCELEROMETER_FULLSCALE_HM0_2G;
        scale = 2.0 / 32768.0;
        break;
    }
}

void RTIMULSM6DSOXandLIS3MDL::mapLSM6DSOXGyroFsrToBitsAndScale(int gyroFsr, unsigned char &bits, RTFLOAT &scale)
{
    switch (gyroFsr)
    {
    case 250:
        bits = LSM6DSOX_GYRO_FULLSCALE_250DPS;
        scale = (250.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;
        break;
    case 500:
        bits = LSM6DSOX_GYRO_FULLSCALE_500DPS;
        scale = (500.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;
        break;
    case 1000:
        bits = LSM6DSOX_GYRO_FULLSCALE_1000DPS;
        scale = (1000.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;
        break;
    case 2000:
        bits = LSM6DSOX_GYRO_FULLSCALE_2000DPS;
        scale = (2000.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;
        break;
    default:
        HAL_INFO1("  Unknown gyro fullscale %d, using ±250DPS\n", gyroFsr);
        bits = LSM6DSOX_GYRO_FULLSCALE_250DPS;
        scale = (250.0 * RTMATH_DEGREE_TO_RAD) / 32768.0;
        break;
    }
}

unsigned char RTIMULSM6DSOXandLIS3MDL::mapLIS3MDLDataRateToBits(int dataRate)
{
    switch (dataRate)
    {
    case 0:
        return LIS3MDL_DATARATE_0_625_HZ;
    case 1:
        return LIS3MDL_DATARATE_1_25_HZ;
    case 2:
        return LIS3MDL_DATARATE_2_5_HZ;
    case 5:
        return LIS3MDL_DATARATE_5_HZ;
    case 10:
        return LIS3MDL_DATARATE_10_HZ;
    case 20:
        return LIS3MDL_DATARATE_20_HZ;
    case 40:
        return LIS3MDL_DATARATE_40_HZ;
    case 80:
        return LIS3MDL_DATARATE_80_HZ;
    case 155:
        return LIS3MDL_DATARATE_155_HZ;
    case 300:
        return LIS3MDL_DATARATE_300_HZ;
    case 560:
        return LIS3MDL_DATARATE_560_HZ;
    case 1000:
        return LIS3MDL_DATARATE_1000_HZ;
    default:
        HAL_INFO1("  Unknown LIS3MDL data rate %d, using 80 Hz\n", dataRate);
        return LIS3MDL_DATARATE_80_HZ;
    }
}

unsigned char RTIMULSM6DSOXandLIS3MDL::mapLIS3MDLXYPerfToBits(int xyPerf)
{
    switch (xyPerf)
    {
    case 0: return LIS3MDL_XY_LOWPOWERMODE;
    case 1: return LIS3MDL_XY_MEDIUMMODE;
    case 2: return LIS3MDL_XY_HIGHMODE;
    case 3: return LIS3MDL_XY_ULTRAHIGHMODE;
    default:
        HAL_INFO1("  Unknown LIS3MDL XY performance %d, using Medium\n", xyPerf);
        return LIS3MDL_XY_MEDIUMMODE;
    }
}

unsigned char RTIMULSM6DSOXandLIS3MDL::mapLIS3MDLZPerfToBits(int zPerf)
{
    switch (zPerf)
    {
    case 0: return LIS3MDL_Z_LOWPOWERMODE;
    case 1: return LIS3MDL_Z_MEDIUMMODE;
    case 2: return LIS3MDL_Z_HIGHMODE;
    case 3: return LIS3MDL_Z_ULTRAHIGHMODE;
    default:
        HAL_INFO1("  Unknown LIS3MDL Z performance %d, using Medium\n", zPerf);
        return LIS3MDL_Z_MEDIUMMODE;
    }
}

void RTIMULSM6DSOXandLIS3MDL::mapLIS3MDLRangeToBitsAndScale(int range, unsigned char &bits, RTFLOAT &scale)
{
    switch (range)
    {
    case 4:
        bits = LIS3MDL_RANGE_4_GAUSS;
        scale = 4.0 / 32768.0;
        break;
    case 8:
        bits = LIS3MDL_RANGE_8_GAUSS;
        scale = 8.0 / 32768.0;
        break;
    case 12:
        bits = LIS3MDL_RANGE_12_GAUSS;
        scale = 12.0 / 32768.0;
        break;
    case 16:
        bits = LIS3MDL_RANGE_16_GAUSS;
        scale = 16.0 / 32768.0;
        break;
    default:
        HAL_INFO1("  Unknown LIS3MDL range %d, using ±4 Gauss\n", range);
        bits = LIS3MDL_RANGE_4_GAUSS;
        scale = 4.0 / 32768.0;
        break;
    }
}

unsigned char RTIMULSM6DSOXandLIS3MDL::mapLIS3MDLOpModeToBits(int opMode)
{
    switch (opMode)
    {
    case 0: return LIS3MDL_CONTINUOUSMODE;
    case 1: return LIS3MDL_SINGLESHOTMODE;
    case 3: return LIS3MDL_POWERDOWNMODE;
    default:
        HAL_INFO1("  Unknown LIS3MDL operation mode %d, using Continuous\n", opMode);
        return LIS3MDL_CONTINUOUSMODE;
    }
}

bool RTIMULSM6DSOXandLIS3MDL::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    HAL_INFO("IMURead called\n");

    // Check LSM6DSOX status
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_STATUS_REG, 1, &status, "Failed to read LSM6DSOX status"))
        return false;
    HAL_INFO1("LSM6DSOX status: 0x%02x\n", status);
    if ((status & 0x03) == 0)  // Check if gyro and accel data ready
    {
        HAL_INFO("LSM6DSOX or LIS3MDL data not ready\n");
        return false;
    }
    HAL_INFO("LSM6DSOX and LIS3MDL data ready\n");

    // Read gyro and accel data
    // Read LSM6DSOX gyroscope data byte-by-byte
    for (int i = 0; i < 6; i++) {
        unsigned char regAddr = LSM6DSOX_OUTX_L_G + i;
        if (!m_settings->HALRead(m_lsm6dsoxAddr, regAddr, 1, &gyroData[i], "Failed to read LSM6DSOX gyro byte")) {
            return false;
        }
    }

    // Read LSM6DSOX accelerometer data byte-by-byte
    for (int i = 0; i < 6; i++) {
        unsigned char regAddr = LSM6DSOX_OUTX_L_A + i;
        if (!m_settings->HALRead(m_lsm6dsoxAddr, regAddr, 1, &accelData[i], "Failed to read LSM6DSOX accel byte")) {
            return false;
        }
    }
    // if (!m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_G, 6, gyroData, "Failed to read LSM6DSOX gyro data"))
    //     return false;
    // if (!m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_A, 6, accelData, "Failed to read LSM6DSOX accel data"))
    //     return false;

    // Read compass data from LIS3MDL byte-by-byte (multi-byte reads can fail on some I2C implementations)
    for (int i = 0; i < 6; i++) {
        unsigned char regAddr = LIS3MDL_REG_OUT_X_L + i;
        if (!m_settings->HALRead(m_lis3mdlAddr, regAddr, 1, &compassData[i], "Failed to read LIS3MDL data byte")) {
            return false;
        }
    }

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    // Convert gyro and accel
    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);

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
    
    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL3_C, 1, &val, "Failed to read LSM6DSOX CTRL3_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL3_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL3_C = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL4_C, 1, &val, "Failed to read LSM6DSOX CTRL4_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL4_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL4_C = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL5_C, 1, &val, "Failed to read LSM6DSOX CTRL5_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL5_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL5_C = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL6_C, 1, &val, "Failed to read LSM6DSOX CTRL6_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL6_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL6_C = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL7_G, 1, &val, "Failed to read LSM6DSOX CTRL7_G"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL7_G\n");
    }
    else
    {
        HAL_INFO1("  CTRL7_G = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL8_XL, 1, &val, "Failed to read LSM6DSOX CTRL8_XL"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL8_XL\n");
    }
    else
    {
        HAL_INFO1("  CTRL8_XL = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL9_XL, 1, &val, "Failed to read LSM6DSOX CTRL9_XL"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL9_XL\n");
    }
    else
    {
        HAL_INFO1("  CTRL9_XL = 0x%02x\n", val);
    }

    if (!m_settings->HALRead(m_lsm6dsoxAddr, LSM6DSOX_CTRL10_C, 1, &val, "Failed to read LSM6DSOX CTRL10_C"))
    {
        HAL_ERROR("Failed to read LSM6DSOX CTRL10_C\n");
    }
    else
    {
        HAL_INFO1("  CTRL10_C = 0x%02x\n", val);
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
    unsigned char gyro[6];
    unsigned char accel[6];
    unsigned char mag[6];

    // Read LSM6DSOX gyroscope data byte-by-byte
    bool ok1 = true;
    for (int i = 0; i < 6; i++) {
        unsigned char regAddr = LSM6DSOX_OUTX_L_G + i;
        if (!m_settings->HALRead(m_lsm6dsoxAddr, regAddr, 1, &gyro[i], "Failed to read LSM6DSOX gyro byte")) {
            ok1 = false;
            break;
        }
    }

    // Read LSM6DSOX accelerometer data byte-by-byte
    for (int i = 0; i < 6; i++) {
        unsigned char regAddr = LSM6DSOX_OUTX_L_A + i;
        if (!m_settings->HALRead(m_lsm6dsoxAddr, regAddr, 1, &accel[i], "Failed to read LSM6DSOX accel byte")) {
            ok1 = false;
            break;
        }
    }

    // Read LSM6DSOX data (multi-byte)
    // ok1 = m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_G, 6, gyro, "Failed to read LSM6DSOX gyro data");
    // ok1 &= m_settings->HALRead(m_lsm6dsoxAddr, 0x80 | LSM6DSOX_OUTX_L_A, 6, accel, "Failed to read LSM6DSOX accel data");

    // Read LIS3MDL data (multi-byte)
    bool ok2 = m_settings->HALRead(m_lis3mdlAddr, 0x80 | LIS3MDL_REG_OUT_X_L, 6, mag, "Failed to read LIS3MDL data");

    HAL_INFO("<");
    if (ok1) {
        HAL_INFO("LSM6DSOX regs ");
        for (int i = 0; i < 6; i++) {
            HAL_INFO1("%02x ", gyro[i]);
        }
        HAL_INFO(" | ");
        for (int i = 0; i < 6; i++) {
            HAL_INFO1("%02x ", accel[i]);
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