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

//    initAndCalibrateAK8963(imu);
    //Software Initialization
//    initMPU9250();

    imuInitialize(imu);

    // sets the biases
    imuCalibrate(imu);
}

void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG);

    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_REG, subAddress);

    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | count);

    SysCtlDelay(100000);

    I2C_ReadBytes(MPU9250_I2C_BASE, MPU9250_ADDRESS, EXT_SENS_DATA_00, count, dest);

}

void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    bool successful = false;
    while(!successful)
    {

        SysCtlDelay(1000000);
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);

        SysCtlDelay(1000000);
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_REG, subAddress);

        SysCtlDelay(1000000);
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_DO, data);

        SysCtlDelay(1000000);
        I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
        SysCtlDelay(1000000);

        uint8_t result;
        readAK8963Registers(subAddress, 1, &result);
        successful = result == data || data == AK8963_RESET;
//        if(!successful)
//            printf("Write failed\n");
        /*if(result == data || data == AK8963_RESET)
            printf("Write successful!\n");
        else
        {
            printf("Write Failed\n");
            goto START;
        }*/
    }
}

void imuInitialize(IMU* imu)
{
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // Enable I2C Master mode
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, USER_CTRL, I2C_MST_EN);

    // set I2C bus speed to 400 KHz
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, I2C_MST_CTRL, I2C_MST_CLK);

    // set AK8963 to power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_PWR_DOWN);

    // reset the MPU9250
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, PWR_RESET);

    // wait for it to come back online
    SysCtlDelay(1000000);

    // reset the AK8963

    writeAK8963Register(AK8963_CNTL2, AK8963_RESET);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL2, AK8963_RESET);

    // select clock source to gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // check connection to the MPU9250, should return 113 or 115 in decimal
    printf("%d\n", I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, WHO_AM_I_MPU9250));

    // enable the accelerometer and the gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_2, SEN_ENABLE);

    // set accelerometer range to 16G by default
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_SEL_16G);

    // set acceleration scale here!!

    // set gyro range to 2000DPS
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);

    // set gyro variables here!!

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
    printf("%d\n", result);

    // get magnetometer calibration, power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_PWR_DOWN);

    // wait for it to come back online
    SysCtlDelay(1000000);

    // set AK8963 to FUSE ROM access
    writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_FUSE_ROM);

    // wait for the change to occur
    SysCtlDelay(1000000);

    uint8_t results[3];
    readAK8963Registers(AK8963_ASA, 3, results);

    imu->magnetometerData.MxFactoryBias = ((((float)results[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    imu->magnetometerData.MyFactoryBias = ((((float)results[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    imu->magnetometerData.MzFactoryBias = ((((float)results[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

//    printf("Mag Factory Bias X: %f\n", imu->magnetometerData.MxFactoryBias);
//    printf("Mag Factory Bias Y: %f\n", imu->magnetometerData.MyFactoryBias);
 //   printf("Mag Factory Bias Z: %f\n", imu->magnetometerData.MzFactoryBias);

    // set AK8963 to power down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_PWR_DOWN);

    // wait for mode change
//    SysCtlDelay(200000000);

    // set AK8963 to 16 bit resolution, 100Hz update rate
    writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);
//    SysCtlDelay(10000000);
 //   writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_CNT_MEAS2);

    // wait for mode change
    SysCtlDelay(1000000);

    // select clock source for gyro
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL);

    // instruct the MPU9250 to read 7 byte of data, (may not be needed)
    // set AK8963 to power down
//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, AK8963_PWR_DOWN);

    // wait for mode change
//    SysCtlDelay(1000000);

//    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL1, 0x12);

//    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMNT_1, CLOCK_SEL_PLL)

    // wait for mode change
  //  SysCtlDelay(1000000);
    uint8_t buffer[7];
    readAK8963Registers(0x03, 7, &buffer[0]);
}

/**
 * ReadIMUData
 * reads the supported data from the imu sensor. This includes
 * acceleration data and gyro data
 * @param imu: a pointer to the imu to read from
 */
void ReadIMUData(IMU* imu)
{
    readSensorData(imu);
    readAccelerationData(imu);
    readGyroData(imu);
//    readMagnetometerData(imu);
}

/**
 * initMPU9250
 * initializes the MPU chip on the IMU. It configures all of the necessary
 * settings needed for the gyroscope and the accelerometer
 */
void initMPU9250()
{
 //   printf("%d\n", I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, WHO_AM_I_MPU9250));
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
    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL, 0x00);     // Power down magnetometer
    SysCtlDelay(100000);
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, 0x68, 0x80);     // Power down magnetometer
    SysCtlDelay(100000);

    I2C_WriteByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_CNTL2, 0x01);
    SysCtlDelay(100000);
 //   printf("%d\n", (I2C_ReadByte(MPU9250_I2C_BASE, AK8963_ADDRESS, WHO_AM_I_AK8963)));
    // Get Factory Bias
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


    // Get Bias and scales
    int32_t scaleX = 0;
    int32_t scaleY = 0;
    int32_t scaleZ = 0;

    int16_t magRawMaxByteDataX = -32767;
    int16_t magRawMaxByteDataY = -32767;
    int16_t magRawMaxByteDataZ = -32767;

    int16_t magRawMinByteDataX = 32767;
    int16_t magRawMinByteDataY = 32767;
    int16_t magRawMinByteDataZ = 32767;

    int i;
    uint16_t count;
    if (MAG_MODE == 0x06)
        count = 1500;
    for(i = 0; i < count; i++)
    {
        readRawMagnetometerData(imu);
//        printf("RawMaxByteDataX: %d\n", imu->magnetometerData.MxRaw);
//        printf("RawMaxByteDataY: %d\n", imu->magnetometerData.MyRaw);
//        printf("RawMaxByteDataZ: %d\n", imu->magnetometerData.MzRaw);
        // compare Max Values
        if(imu->magnetometerData.MxRaw > magRawMaxByteDataX)
            magRawMaxByteDataX = imu->magnetometerData.MxRaw;
        if(imu->magnetometerData.MyRaw > magRawMaxByteDataY)
            magRawMaxByteDataY = imu->magnetometerData.MyRaw;
        if(imu->magnetometerData.MzRaw > magRawMaxByteDataZ)
            magRawMaxByteDataZ = imu->magnetometerData.MzRaw;

        // compare Min Values
        if(imu->magnetometerData.MxRaw < magRawMinByteDataX)
            magRawMinByteDataX = imu->magnetometerData.MxRaw;
        if(imu->magnetometerData.MyRaw < magRawMinByteDataY)
            magRawMinByteDataY = imu->magnetometerData.MyRaw;
        if(imu->magnetometerData.MzRaw < magRawMinByteDataZ)
            magRawMinByteDataZ = imu->magnetometerData.MzRaw;

        SysCtlDelay(10000);
    }

    imu->magnetometerData.MxBias = ((magRawMaxByteDataX + magRawMinByteDataX) / 2) * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MxFactoryBias;
    imu->magnetometerData.MyBias = ((magRawMaxByteDataY + magRawMinByteDataY) / 2) * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MyFactoryBias;
    imu->magnetometerData.MzBias = ((magRawMaxByteDataZ + magRawMinByteDataZ) / 2) * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MzFactoryBias;

    scaleX = (float)(magRawMaxByteDataX - magRawMinByteDataX) * imu->magnetometerData.MxFactoryBias / 2;
    scaleY = (float)(magRawMaxByteDataY - magRawMinByteDataY) * imu->magnetometerData.MyFactoryBias / 2;
    scaleZ = (float)(magRawMaxByteDataZ - magRawMinByteDataZ) * imu->magnetometerData.MzFactoryBias / 2;

    float average = (scaleX + scaleY + scaleX) / 3.0;
    imu->magnetometerData.MxScale = average / ((float)scaleX);
    imu->magnetometerData.MyScale = average / ((float)scaleY);
    imu->magnetometerData.MzScale = average / ((float)scaleZ);
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

    imu.gyroData.GxRaw = 0;
    imu.gyroData.GyRaw = 0;
    imu.gyroData.GzRaw = 0;

    imu.gyroData.Gx = 0.0;
    imu.gyroData.Gy = 0.0;
    imu.gyroData.Gz = 0.0;

    imu.gyroData.GxBias = 0.0;
    imu.gyroData.GyBias = 0.0;
    imu.gyroData.GzBias = 0.0;

    imu.magnetometerData.MxRaw = 0;
    imu.magnetometerData.MyRaw = 0;
    imu.magnetometerData.MzRaw = 0;

    imu.magnetometerData.Mx = 0.0;
    imu.magnetometerData.My = 0.0;
    imu.magnetometerData.Mz = 0.0;

    imu.magnetometerData.MxFactoryBias = 0.0;
    imu.magnetometerData.MyFactoryBias = 0.0;
    imu.magnetometerData.MzFactoryBias = 0.0;

    imu.magnetometerData.MxBias = 0.0;
    imu.magnetometerData.MyBias = 0.0;
    imu.magnetometerData.MzBias = 0.0;

    imu.magnetometerData.MxScale = 0.0;
    imu.magnetometerData.MyScale = 0.0;
    imu.magnetometerData.MzScale = 0.0;

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
void readRawMagnetometerData(IMU* imu)
{
    uint8_t magnetometerStatus = I2C_ReadByte(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_ST1);
    uint8_t rawBytes[7];
//    if(magnetometerStatus & 0x01)   // if the ready status bit is set
//    {
        I2C_ReadBytes(MPU9250_I2C_BASE, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawBytes[0]);
        // if(status & 0x02) != 0)
        //    return;
//    }
//    uint8_t dataStatus = rawBytes[6];
//    if(!(dataStatus & 0x08))
//    {
//        printf("Reading new data!\n");
        imu->magnetometerData.MxRaw = ((int16_t)rawBytes[1] << 8) | rawBytes[0];
        imu->magnetometerData.MyRaw = ((int16_t)rawBytes[3] << 8) | rawBytes[2];
        imu->magnetometerData.MzRaw = ((int16_t)rawBytes[5] << 8) | rawBytes[4];

//        printf("MxRaw: %d\n", imu->magnetometerData.MxRaw);
//    }
}

void readSensorData(IMU* imu)
{
    uint8_t rawByteData[21];
    I2C_ReadBytes(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_OUT, 21, rawByteData);

    imu->accelerationData.AxRaw = (((int16_t)rawByteData[0]) << 8) | rawByteData[1];
    imu->accelerationData.AyRaw = (((int16_t)rawByteData[2]) << 8) | rawByteData[3];
    imu->accelerationData.AzRaw = (((int16_t)rawByteData[4]) << 8) | rawByteData[5];

    imu->gyroData.GxRaw = (((int16_t)rawByteData[8]) << 8) | rawByteData[9];
    imu->gyroData.GyRaw = (((int16_t)rawByteData[10]) << 8) | rawByteData[11];
    imu->gyroData.GzRaw = (((int16_t)rawByteData[12]) << 8) | rawByteData[13];

    imu->magnetometerData.MxRaw = (((int16_t)rawByteData[15]) << 8) | rawByteData[14];
    imu->magnetometerData.MyRaw = (((int16_t)rawByteData[17]) << 8) | rawByteData[16];
    imu->magnetometerData.MzRaw = (((int16_t)rawByteData[19]) << 8) | rawByteData[18];

    // Convert Raw to actual value
    imu->accelerationData.Ax = (float)(imu->accelerationData.AxRaw) * ACCELEROMETERFACTOR_16G;
    imu->accelerationData.Ay = (float)(imu->accelerationData.AyRaw) * ACCELEROMETERFACTOR_16G;
    imu->accelerationData.Az = (float)(imu->accelerationData.AzRaw) * ACCELEROMETERFACTOR_16G;

    imu->gyroData.Gx = ((float)(imu->gyroData.GxRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GxBias;
    imu->gyroData.Gy = ((float)(imu->gyroData.GyRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GyBias;
    imu->gyroData.Gz = ((float)(imu->gyroData.GzRaw) * GYROFACTOR_2000DPS) + imu->gyroData.GzBias;
 //   printf("Gy: %f\n", imu->gyroData.Gy);

    imu->magnetometerData.Mx = ((float)imu->magnetometerData.MxRaw) * imu->magnetometerData.MxFactoryBias;
    imu->magnetometerData.My = ((float)imu->magnetometerData.MyRaw) * imu->magnetometerData.MyFactoryBias;
    imu->magnetometerData.Mz = ((float)imu->magnetometerData.MzRaw) * imu->magnetometerData.MzFactoryBias;

    //
//    printf("AxRaw: %d\n", AxRaw);
//
////    uint8_t magBuffer[6];
////    readAK8963Registers(AK8963_XOUT_L, 6, magBuffer);
//    int16_t MxRaw = (((int16_t)buffer[15]) << 8) | buffer[14];
//    printf("MxRaw: %d\n", MxRaw);

}

void getMagnetometerDataFromRaw(IMU* imu)
{
    imu->magnetometerData.Mx = (float)(imu->magnetometerData.MxRaw * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MxFactoryBias) * imu->magnetometerData.MxScale;
    imu->magnetometerData.My = (float)(imu->magnetometerData.MyRaw * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MyFactoryBias) * imu->magnetometerData.MyScale;
    imu->magnetometerData.Mz = (float)(imu->magnetometerData.MzRaw * M16BIT_MAG_RESOLUTION * imu->magnetometerData.MzFactoryBias) * imu->magnetometerData.MzScale;
}

void readMagnetometerData(IMU* imu)
{
    readRawMagnetometerData(imu);
    getMagnetometerDataFromRaw(imu);
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
        readSensorData(imu);

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
//    printf("%f\n", imu->gyroData.GyBias);
}
