#include "IMU.h"

/**
 * imuEnable
 * enables the IMU on the Tiva
 * @param: imu: a pounter to the imu to enable
 */
void imuEnable(IMU* imu)
{
    // enable the I2C pins used
    I2C1_Config();

    //Software Initialization
    initMPU9250();

    initAndCalibrateAK8963(imu);

    // sets the biases
    imuCalibrate(imu);
}

/**
 * ReadIMUData
 * reads the supported data from the imu sensor. This includes
 * acceleration data and gyro data
 * @param imu: a pointer to the imu to read from
 */
void ReadIMUData(IMU* imu)
{
    readAccelerationData(imu);
    readGyroData(imu);
}

/**
 * initMPU9250
 * initializes the MPU chip on the IMU. It configures all of the necessary
 * settings needed for the gyroscope and the accelerometer
 */
void initMPU9250()
{
    uint8_t accelerationScale = AFS_2G;
    uint8_t gyroScale = GFS_250DPS;
//    enum AccelerationScale accelerationScale = AFS_2G;
//    enum GyroScale gyroScale = GFS_250DPS;

    // Clear sleep mode bit (6), enable all sensors
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    SysCtlDelay(100000);

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    SysCtlDelay(200000);

    /** Configure the GYRO **/
    // get current GYRO_CONFIG register value
    uint8_t byteConfig = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG);
    byteConfig = byteConfig & ~0x02; // Clear Fchoice bits [1:0]
    byteConfig = byteConfig & ~0x18; // Clear AFS bits [4:3]
    byteConfig = byteConfig | (((uint8_t)gyroScale) << 3); // Set full scale range for the gyro
    // Write new GYRO_CONFIG value to register
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG, byteConfig);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    byteConfig = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG);
    byteConfig = byteConfig & ~0x18;  // Clear AFS bits [4:3]
    byteConfig = byteConfig | (((uint8_t)accelerationScale) << 3); // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG, byteConfig);

    // Set accelerometer sample rate configuration
    byteConfig = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG2);
    byteConfig = byteConfig & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    byteConfig = byteConfig | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG2, byteConfig);

    SysCtlDelay(100000);

}

void initAndCalibrateAK8963(IMU* imu)
{
    uint8_t magnetometerOutputBitSetting = sixteenBits;
    uint8_t rawByteData[3];
    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL, 0x00);     // Power down magnetometer
    SysCtlDelay(100000);
    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL, 0x0F);     // Enter Fuse ROM Mode
    SysCtlDelay(100000);
    I2C_ReadBytes(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_ASAX, 3, rawByteData);

    imu->magnetometerData.MxFactoryBias = (float)(rawByteData[0] - 128) / 256.0 + 1.0;
    imu->magnetometerData.MyFactoryBias = (float)(rawByteData[1] - 128) / 256.0 + 1.0;
    imu->magnetometerData.MzFactoryBias = (float)(rawByteData[2] - 128) / 256.0 + 1.0;

    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL, 0x00);     // Power down magnetometer

    SysCtlDelay(100000);

    uint8_t byteConfig = (uint8_t) magnetometerOutputBitSetting << 4 | 0x06;

    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL, byteConfig);

    SysCtlDelay(100000);

//    printf("MxBias: %f, %d\n", imu->imuBias.MxBias, rawByteData[0]);
//    printf("MyBias: %f\n", imu->imuBias.MyBias);
//    printf("MzBias: %f\n", imu->imuBias.MzBias);

}

/**
 * imuConstruct
 * constructs an imu
 */
IMU imuConstruct(void)
{
    IMU imu;
    imu.enabled = false;
    imu.accelerationData.Ax = 0.0;
    imu.accelerationData.Ay = 0.0;
    imu.accelerationData.Az = 0.0;
    imu.gyroData.Gx = 0.0;
    imu.gyroData.Gy = 0.0;
    imu.gyroData.Gz = 0.0;

    imu.gyroData.GxBias = 0.0;
    imu.gyroData.GyBias = 0.0;
    imu.gyroData.GzBias = 0.0;

    imu.magnetometerData.MxFactoryBias = 0.0;
    imu.magnetometerData.MyFactoryBias = 0.0;
    imu.magnetometerData.MzFactoryBias = 0.0;

    imu.magnetometerData.Mx = 0.0;
    imu.magnetometerData.My = 0.0;
    imu.magnetometerData.Mz = 0.0;

    return imu;
}

/**
 * readAccelerationData
 * Reads the acceleration data from the imu
 * @param imu: a pointer to the IMU to read acceleration data from
 */
void readAccelerationData(IMU* imu)
{
    uint8_t rawByteData[6];  // x/y/z accel register data stored here
    int16_t rawAccelerationX;
    int16_t rawAccelerationY;
    int16_t rawAccelerationZ;
    // Read the six raw data registers into data array        // Note 0x68 is the IMU slave address
    I2C_ReadBytes(MPU9250_I2C_BASE, 0x68, ACCEL_XOUT_H, 6, &rawByteData[0]);        //#define ACCEL_XOUT_H       0x3B

    // Turn the MSB and LSB into a signed 16-bit value
    rawAccelerationX = ((int16_t)rawByteData[0] << 8) | rawByteData[1];
    rawAccelerationY = ((int16_t)rawByteData[2] << 8) | rawByteData[3];
    rawAccelerationZ = ((int16_t)rawByteData[4] << 8) | rawByteData[5];

    imu->accelerationData.Ax = (float)(rawAccelerationX / ACCELEROMETERFACTOR);
    imu->accelerationData.Ay = (float)(rawAccelerationY / ACCELEROMETERFACTOR);
    imu->accelerationData.Az = (float)(rawAccelerationZ / ACCELEROMETERFACTOR);
}

/**
 * readGyroData
 * Reads the gyro data from the imu
 * @param imu: a pointer to the IMU to read gyro data from
 */
void readGyroData(IMU* imu)
{
    uint8_t rawByteData[6];
    int16_t rawGyroX;
    int16_t rawGyroY;
    int16_t rawGyroZ;
    // Read the six raw data registers sequentially into data array
    I2C_ReadBytes(MPU9250_I2C_BASE, 0x68, GYRO_XOUT_H, 6, &rawByteData[0]);

    rawGyroX = ((int16_t)rawByteData[0] << 8) | rawByteData[1];
    rawGyroY = ((int16_t)rawByteData[2] << 8) | rawByteData[3];
    rawGyroZ = ((int16_t)rawByteData[4] << 8) | rawByteData[5];

    imu->gyroData.Gx = (float)(rawGyroX / GYROFACTOR) + imu->gyroData.GxBias;
    imu->gyroData.Gy = (float)(rawGyroY / GYROFACTOR) + imu->gyroData.GyBias;
    imu->gyroData.Gz = (float)(rawGyroZ / GYROFACTOR) + imu->gyroData.GzBias;
}

/**
 * readMagnetometerData
 * Reads magnetometer data from the imu
 * @param imu: a pointer to the IMU to read magnetometer data from
 */
void readMagnetometerData(IMU* imu)
{
    uint8_t magnetometerStatus = I2C_ReadByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_ST1);
    uint8_t rawBytes[7];
    if(magnetometerStatus & 0x01)   // if the ready status bit is set
    {
        I2C_ReadBytes(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawBytes[0]);
        // if(status & 0x02) != 0)
        //    return;
    }
    uint8_t dataStatus = rawBytes[6];
    int16_t rawMagnetometerX;
    int16_t rawMagnetometerY;
    int16_t rawMagnetometerZ;
    if(!(dataStatus & 0x08))
    {
        rawMagnetometerX = ((int16_t)rawBytes[1] << 8) | rawBytes[0];
        rawMagnetometerY = ((int16_t)rawBytes[3] << 8) | rawBytes[2];
        rawMagnetometerZ = ((int16_t)rawBytes[5] << 8) | rawBytes[4];
    }
}

/**
 * imuCalibrate
 * Calibrates the imu and addresses its initial biases. Calibrations involves
 * taking multiple reading iterations to do.
 */
void imuCalibrate(IMU* imu)
{
    float accelbiasX;
    float accelbiasY;
    float accelbiasZ;

    float gyrobiasX;
    float gyrobiasY;
    float gyrobiasZ;

    uint16_t i;
    int iter = 100;

    for(i = 0; i < iter; i++)
    {
        readAccelerationData(imu);
        readGyroData(imu);

        accelbiasX += imu->accelerationData.Ax;
        accelbiasY += imu->accelerationData.Ay;
        accelbiasZ += imu->accelerationData.Az;

        gyrobiasX += imu->gyroData.Gx;
        gyrobiasY += imu->gyroData.Gy;
        gyrobiasZ += imu->gyroData.Gz;
    }

    imu->gyroData.GxBias = -1.0 * (gyrobiasX/(iter + 1));
    imu->gyroData.GyBias = -1.0 * (gyrobiasY/(iter + 1));
    imu->gyroData.GzBias = -1.0 * (gyrobiasZ/(iter + 1));
}
