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

    imuInitialize(imu);

    // sets the biases
    imuCalibrate(imu);
}

/**
 * readAK8963Registers
 * Reads data from the registers of the AK8963 (the magnetometer)
 * given the address and the number of bytes to read
 * @param subAddress: the sub-address of the registers located on the AK8963
 * @param length: the number of bytes to read
 * @param dest: a pointer to where the data read will be stored
 */
void readAK8963Registers(uint8_t subAddress, uint8_t length, uint8_t* dest)
{
    // instruct to talk to the AK8963
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG);

    // instruct to read from the given sub-address
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_REG, subAddress);

    // instruct to receive a certain number of bytes
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | length);

    // wait for it to come back online
    SysCtlDelay(100000);

    // read the data from the AK8963 and store it
    I2C_ReadBytes(MPU9250_I2C_BASE, MPU9250_ADDRESS, EXT_SENS_DATA_00, length, dest);

}

/**
 * writeAK8963Register
 * Write data to a register on the AK8963 (the magentometer)
 * give the sub-address to write to
 * @param subAddress: The sub-address of the register on the AK8963
 * @param data: the data to write to the register
 */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    // create a flag which signals if the write was successful
    bool successful = false;

    // keep trying to send data until the write is successful
    while(!successful)
    {
        // wait a little bit before we write data, writing data to fast
        // may result in a failure
        SysCtlDelay(1000000);

        // instruct to talk to the AK8963
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);

        // wait a little bit before we write data, writing data to fast
        // may result in a failure
        SysCtlDelay(1000000);

        // instruct to write to the sub-address given
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_REG, subAddress);

        // wait a little bit before we write data, writing data to fast
        // may result in a failure
        SysCtlDelay(1000000);

        // write the data
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_DO, data);

        // wait a little bit before we write data, writing data to fast
        // may result in a failure
        SysCtlDelay(1000000);

        // signal that the write has been completed
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);

        // wait a little bit before we write data, writing data to fast
        // may result in a failure
        SysCtlDelay(1000000);

        // read from the AK8963 to make sure the data was successfully written
        uint8_t result;
        readAK8963Registers(subAddress, 1, &result);
        successful = result == data || data == AK8963_RESET;
    }
}

/**
 * imuInitialize
 * Initializes all of the IMU peripherals:
 * accelerometer, gyro, mangetometer
 * @param imu: a pointer to the IMU to initialize
 */
void imuInitialize(IMU* imu)
{
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // Enable I2C Master mode
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, USER_CTRL, I2C_MST_EN);

    // set I2C bus speed to 400 KHz
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_MST_CTRL, I2C_MST_CLK);

    // set AK8963 to power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // reset the MPU9250
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, PWR_RESET);

    // wait for it to come back online
    SysCtlDelay(1000000);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2, AK8963_RESET);

    // select clock source to gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // check connection to the MPU9250, should return 113 or 115 in decimal
    // printf("%d\n", I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, WHO_AM_I_MPU9250));

    // enable the accelerometer and the gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_2, SEN_ENABLE);

    // set accelerometer range to 16G by default
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_SEL_16G);

    // set gyro range to 2000DPS
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);

    // set acceleration bandwidth to 182Hz by default
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG2, ACCEL_DLPF_184);

    // set gyro bandwith to 182Hz by default
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, CONFIG, GYRO_DLPF_184);

    // set sample rate divider to 0 as default
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, SMPDIV, 0x00);

    // enable I2C master mode
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, USER_CTRL, I2C_MST_EN);

    // set I2C bus speed to 400 Khz
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_MST_CTRL, I2C_MST_CLK);

    // check connectiont o the AK8962, should return 72 in decimal
    uint8_t result = 0;
    readAK8963Registers(WHO_AM_I_AK8963, 1 ,&result);
    //printf("%d\n", result);

    // get magnetometer calibration, power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // wait for it to come back online
    SysCtlDelay(1000000);

    // set AK8963 to FUSE ROM access
    writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);

    // wait for the change to occur
    SysCtlDelay(1000000);

    // read the raw factory bias values from the AK8963 (the mangeotmeter) to
    uint8_t results[3];
    readAK8963Registers(AK8963_ASA, 3, results);

    // store the factory bias values accordingly (values in micro Telsa after conversion)
    imu->magnetometerData.MxFactoryBias = ((((float)results[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
    imu->magnetometerData.MyFactoryBias = ((((float)results[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
    imu->magnetometerData.MzFactoryBias = ((((float)results[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;

    // set AK8963 to power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // set AK8963 to 16 bit resolution, 100Hz update rate
    writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

    // wait for mode change
    SysCtlDelay(1000000);

    // select clock source for gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // instruct the AK8963 (the magnetometer) to read 7 byte of
    // data to complete initialization
    uint8_t buffer[7];
    readAK8963Registers(0x03, 7, &buffer[0]);
}

/**
 * imuConstruct
 * constructs an imu
 */
IMU imuConstruct(void)
{
    IMU imu;

    // set it to disabled by default
    imu.enabled = false;

    // Initialize the acceleration data to 0
    imu.accelerationData.Ax = 0.0;
    imu.accelerationData.Ay = 0.0;
    imu.accelerationData.Az = 0.0;

    // Initialize the raw gyro data to 0
    imu.gyroData.GxRaw = 0;
    imu.gyroData.GyRaw = 0;
    imu.gyroData.GzRaw = 0;

    // Initialize the gyro data to 0
    imu.gyroData.Gx = 0.0;
    imu.gyroData.Gy = 0.0;
    imu.gyroData.Gz = 0.0;

    // Initialize the gyro bias data to 0
    imu.gyroData.GxBias = 0.0;
    imu.gyroData.GyBias = 0.0;
    imu.gyroData.GzBias = 0.0;

    // Initialize the raw magnetometer data to 0
    imu.magnetometerData.MxRaw = 0;
    imu.magnetometerData.MyRaw = 0;
    imu.magnetometerData.MzRaw = 0;

    // Initialize the mangetometer data to 0
    imu.magnetometerData.Mx = 0.0;
    imu.magnetometerData.My = 0.0;
    imu.magnetometerData.Mz = 0.0;

    // Initialize the magnetometer factory bias data to 0
    imu.magnetometerData.MxFactoryBias = 0.0;
    imu.magnetometerData.MyFactoryBias = 0.0;
    imu.magnetometerData.MzFactoryBias = 0.0;

    // Initialize the magnetometer bias data to 0
    imu.magnetometerData.MxBias = 0.0;
    imu.magnetometerData.MyBias = 0.0;
    imu.magnetometerData.MzBias = 0.0;

    // Initialize the magnetometer scale to 0
    imu.magnetometerData.MxScale = 0.0;
    imu.magnetometerData.MyScale = 0.0;
    imu.magnetometerData.MzScale = 0.0;

    // return the newly constructed IMU
    return imu;
}

/**
 * readSensorData
 * reads all the necessary sensor data from the IMU:
 * acceleration data, gyro data, magentometer data
 * @param imu: The IMU to read data from
 */
void readSensorData(IMU* imu)
{
    // read the raw data from the IMU
    uint8_t rawByteData[21];
    I2C_ReadBytes(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_OUT, 21, rawByteData);

    // store the raw accceleration data
    imu->accelerationData.AxRaw = (((int16_t)rawByteData[0]) << 8) | rawByteData[1];
    imu->accelerationData.AyRaw = (((int16_t)rawByteData[2]) << 8) | rawByteData[3];
    imu->accelerationData.AzRaw = (((int16_t)rawByteData[4]) << 8) | rawByteData[5];

    // store the raw gyro data
    imu->gyroData.GxRaw = (((int16_t)rawByteData[8]) << 8) | rawByteData[9];
    imu->gyroData.GyRaw = (((int16_t)rawByteData[10]) << 8) | rawByteData[11];
    imu->gyroData.GzRaw = (((int16_t)rawByteData[12]) << 8) | rawByteData[13];

    // store the raw magnetometer data
    imu->magnetometerData.MxRaw = (((int16_t)rawByteData[15]) << 8) | rawByteData[14];
    imu->magnetometerData.MyRaw = (((int16_t)rawByteData[17]) << 8) | rawByteData[16];
    imu->magnetometerData.MzRaw = (((int16_t)rawByteData[19]) << 8) | rawByteData[18];

    // Convert the raw acceleration data to the actual accleration data
    imu->accelerationData.Ax = (float)(imu->accelerationData.AxRaw) * ACCELEROMETERFACTOR_16G;
    imu->accelerationData.Ay = (float)(imu->accelerationData.AyRaw) * ACCELEROMETERFACTOR_16G;
    imu->accelerationData.Az = (float)(imu->accelerationData.AzRaw) * ACCELEROMETERFACTOR_16G;

    // Convert the raw gyro data to the actual gyro data
    imu->gyroData.Gx = ((float)(imu->gyroData.GxRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GxBias;
    imu->gyroData.Gy = ((float)(imu->gyroData.GyRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GyBias;
    imu->gyroData.Gz = ((float)(imu->gyroData.GzRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GzBias;

    // Convert the raw magnetometer data to actual magnetometer data
    imu->magnetometerData.Mx = ((float)imu->magnetometerData.MxRaw) * imu->magnetometerData.MxFactoryBias;
    imu->magnetometerData.My = ((float)imu->magnetometerData.MyRaw) * imu->magnetometerData.MyFactoryBias;
    imu->magnetometerData.Mz = ((float)imu->magnetometerData.MzRaw) * imu->magnetometerData.MzFactoryBias;
}


/**
 * imuCalibrate
 * Calibrates the imu and addresses its initial biases. Calibrations involves
 * taking multiple reading iterations to do.
 */
void imuCalibrate(IMU* imu)
{
    // Initialize the temporary bias variables for gyro
    float gyrobiasX = 0.0;
    float gyrobiasY = 0.0;
    float gyrobiasZ = 0.0;

    // take 100 samples
    uint16_t i;
    int iter = 100;
    for(i = 0; i < iter; i++)
    {
        // read the Sensor Data
        readSensorData(imu);

        // store the gyro bias for this iteration
        gyrobiasX += imu->gyroData.Gx;
        gyrobiasY += imu->gyroData.Gy;
        gyrobiasZ += imu->gyroData.Gz;
    }

    // take an average to store the bias
    imu->gyroData.GxBias = -1.0 * (gyrobiasX/(iter + 1));
    imu->gyroData.GyBias = -1.0 * (gyrobiasY/(iter + 1));
    imu->gyroData.GzBias = -1.0 * (gyrobiasZ/(iter + 1));
}
