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
    enum AccelerationScale accelerationScale = AFS_2G;
    enum GyroScale gyroScale = GFS_250DPS;

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

    imu.imuBias.AxBias = 0.0;
    imu.imuBias.AyBias = 0.0;
    imu.imuBias.AzBias = 0.0;
    imu.imuBias.GxBias = 0.0;
    imu.imuBias.GyBias = 0.0;
    imu.imuBias.GzBias = 0.0;

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

    imu->accelerationData.Ax = (float)(rawAccelerationX / ACCELEROMETERFACTOR) + imu->imuBias.AxBias;
    imu->accelerationData.Ay = (float)(rawAccelerationY / ACCELEROMETERFACTOR) + imu->imuBias.AyBias;
    imu->accelerationData.Az = (float)(rawAccelerationZ / ACCELEROMETERFACTOR) + imu->imuBias.AzBias;
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

    imu->gyroData.Gx = (float)(rawGyroX / GYROFACTOR) + imu->imuBias.GxBias;
    imu->gyroData.Gy = (float)(rawGyroY / GYROFACTOR) + imu->imuBias.GyBias;
    imu->gyroData.Gz = (float)(rawGyroZ / GYROFACTOR) + imu->imuBias.GzBias;
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

    imu->imuBias.AxBias = (accelbiasX/(iter));
    imu->imuBias.AyBias = (accelbiasY/(iter));
    imu->imuBias.AxBias = (accelbiasZ/(iter));
    imu->imuBias.GxBias = -1.0 * (gyrobiasX/(iter));
    imu->imuBias.GyBias = -1.0 * (gyrobiasY/(iter));
    imu->imuBias.GzBias = -1.0 * (gyrobiasZ/(iter));
}
