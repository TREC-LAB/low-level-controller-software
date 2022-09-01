#include "IMU.h"


//Global variables  note most of these if not all must be changed to float values
//int16_t gyro[3], accel[3];//, mag[3];
//int16_t ConvGyro[3], ConvAccel[3];          //see in future if this can be made a float
//uint8_t FlagG = 0, FlagA = 1, FlagTest = 2; // FlagG = Gyro, FlagA = Accel
//int16_t Abias[3], Gbias[3];

// Changing all global variables to floats, Also potentially change some of these globals into structs
int16_t gyro[3], accel[3];//, mag[3];
float ConvGyro[3], ConvAccel[3];          //see in future if this can be made a float
uint8_t FlagG = 0, FlagA = 1, FlagTest = 2; // FlagG = Gyro, FlagA = Accel
float Abias[3], Gbias[3];



/**
 *
 This Function calls the necessary functions to initialize the IMU and set up.
 *
 **/
void MPU_START()
{
    MPU_Config();

    // Power ON for MPU (MOSFET CONTROL)
    MPU_PowerSwitch(true);


    //  Power cycle MPU chip before every SW initialization
//    MPU_PowerSwitch(false);
//    SysCtlDelay(20000);
//    MPU_PowerSwitch(true);
//    SysCtlDelay(30000);


    //Software Initialization
    initMPU9250();

    // biases -> Leave this for now, but biases are not used right now
    CalibrateMPU(Abias, Gbias);
}

/**
 *
 This Function call the necessary functions to read acceleration and gyro data
 *
 **/
void Read_IMU()
{

    // Reads Data in integers
    readAccelData(accel);
    readGyroData(gyro);

    // Converts data from int -> float and using necessary formulas
    RawToRefine(ConvAccel, accel, FlagA, Abias);
    RawToRefine(ConvGyro, gyro, FlagG, Gbias);

}

/**
 *
 This Function call the necessary functions to read and store IMU data
 *
 **/
void IMU_GET(IMU* IMU)
{
    Read_IMU();
    SaveIMUData(IMU);
}

/**
 *
 This Function initializes the I2C communication for the MPU 9250
 *
 **/
//void HAL_MPU_Init(void((*custHook)(void)))
void MPU_Config()
{
    I2C1_Config();
}

/**
 * Configure MPU9250 accelerometer and gyroscope
 * Configures gyro for 1kHz sampling rate, 42Hz bandwidth and 200Hz output rate
 * and use full scale readings (+/- 250 dps). Configures accel for 1kHz sampling
 * rate, 200Hz output rate and full scale readings (+/- 16g). Finally, configure
 * interrupt pin to be active high, push-pull, held high until cleared and
 * cleared by reading ANY register. I2C bypass is disabled to allow for SPI.
 * Data-ready interrupts are only allowed.
 */
void initMPU9250()
{
    uint8_t Gscale = GFS_250DPS; // GFS_250DPS
    uint8_t Ascale = AFS_2G;     // AFS_2G

    // Enable on first run to calibrate the sensor.             // for now just manually tare it, ie, take in a
//     calibrateMPU9250_2(gBias, aBias);
//
//     UARTprintf("  BiasAx = %05d, BiasAy = %05d, BiasAz = %05d  \n", aBias[0], aBias[1], aBias[2]);        // x,y,z accelerations
//     UARTprintf("  BiasGx = %05d, BiasGy = %05d, BiasGz = %05d  \n", gBias[0], gBias[1], gBias[2]);           // x,y,z gyrations
    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
//    HAL_DelayUS(1000*100); // Wait for all registers to reset
    SysCtlDelay(100000);

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
//    HAL_DelayUS(1000*200);
    SysCtlDelay(200000);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
    // 8 kHz, or 1 kHz
//    HAL_MPU_WriteByte_2(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above.
//    HAL_MPU_WriteByte_2(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    // get current GYRO_CONFIG register value
    uint8_t c = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    //c |= 0x03;
    // Write new GYRO_CONFIG value to register
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    c = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value     // This may actually work, hard to confirm
    c = I2C_ReadByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    I2C_WriteByte(MPU9250_I2C_BASE, MPU9250_ADDRESS, ACCEL_CONFIG2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of any register, and DISABLE
    // I2C_BYPASS_EN -> otherwise communication with AK8963 doesn't work when
    //  using SPI
//    HAL_MPU_WriteByte_2(MPU9250_ADDRESS, INT_PIN_CFG, 0x30);
//    // Enable data ready (bit 0) interrupt
//    HAL_MPU_WriteByte_2(MPU9250_ADDRESS, INT_ENABLE, 0x01);
////    HAL_DelayUS(1000*100);
    SysCtlDelay(100000);
}

/**
 * Control power-switch for MPU9250
 * Controls whether or not MPU sensors receives power (n-ch MOSFET as switch)
 * @param powerState Desired state of power switch (active high)
 */
void MPU_PowerSwitch(bool powerState)
{
    if (powerState)
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
}

/**
 *
 This function is setting up the data var in the struct
     Acceleration m/s^2
     Gyration : degrees/s
 *
 **/
IMU IMU_Struct_Config(void)
{
    IMU IMUsense;
    IMUsense.Ax = 0.0;
    IMUsense.Ay = 0.0;
    IMUsense.Az = 0.0;
    IMUsense.Gx = 0.0;
    IMUsense.Gy = 0.0;
    IMUsense.Gz = 0.0;

    return IMUsense;
}

/**
 *
 This function is assigning IMU data from global var to the struct data var
     Acceleration m/s^2
     Gyration : degrees/s
 *
 **/
void SaveIMUData(IMU* imu)
{
    imu->Ax = ConvAccel[0];
    imu->Ay = ConvAccel[1];
    imu->Az = ConvAccel[2];
    imu->Gx = ConvGyro[0];
    imu->Gy = ConvGyro[1];
    imu->Gz = ConvGyro[2];

//    printf("%f\n", ConvAccel[0]);
}
/**
 *
 This function takes in raw accelerations and gyro data and converts it to,
     Acceleration m/s^2
     Gyration : degrees/s
     Type Flag : defines if it is Gyro or Acceleration data
     Currently Bias have been REMOVED from calculations
 *
 **/
void RawToRefine(float * Converted_Data, int16_t * Raw_Data, uint8_t TypeFlag, float * bias)
{
    uint16_t i;
    int GyroType = 0, AccelType = 1;
    float GyroFactor = 131.0, AccelFactor = 16384.0;

    if(TypeFlag == GyroType)
    {
        for (i = 0; i < 3; i++)
        {                                                            // Converted Gyro data is degree/s

            Converted_Data[i] = ((float)(Raw_Data[i]/GyroFactor)) + Gbias[i];         // note 131 is LSB/(degrees/s)        // 250 is a dps setting
//            printf("Gyro[%d] = %f\n", i, Converted_Data[i]);
        }

    }
    else if(TypeFlag == AccelType)
    {
        for (i = 0; i < 3; i++)
        {                                                               // units are in g for gravity
            Converted_Data[i] = ((float)(Raw_Data[i]/AccelFactor));         // Hardcode, but g = 9.81 m/s^2  16384 is taken fromo mpu 9250 datasheet and LSB Senstivity
        }
    }

}

/**
 * Read raw accelerometer data into a provided buffer
 * @param destination Buffer to save x, y, z acceleration data (min. size = 3)
 */
void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array        // Note 0x68 is the IMU slave address
  I2C_ReadBytes(MPU9250_I2C_BASE, 0x68, ACCEL_XOUT_H, 6, &rawData[0]);        //#define ACCEL_XOUT_H       0x3B

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Read raw gyroscope data into a provided buffer
 * @param destination Buffer to save x, y, z gyroscope data (min. size = 3)
 */
void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  I2C_ReadBytes(MPU9250_I2C_BASE, 0x68, GYRO_XOUT_H, 6, &rawData[0]);  //#define GYRO_XOUT_H        0x43

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;

//  printf("Read Gyro Data: %f\n", (float)(destination[0]/131.0));
}

/**
 This function takes so many iterations of data, than takes an average of this data and applies it to taring variables
     which than are applied inside the RawToRefine function.

     Currently only one iter will work, issue arise when iter>1,
 */
void CalibrateMPU()
{
    uint16_t i;
    int16_t accelbias[3], gyrobias[3];
    float accelbiasX, accelbiasY, accelbiasZ, gyrobiasX, gyrobiasY, gyrobiasZ;

//    accelbiasX = /*-0.012*/ 0.0, accelbiasY = 1.93, accelbiasZ = 0.85, gyrobiasX = 0.0, gyrobiasY = 5.0, gyrobiasZ = 7.0;

    int iter = 100;


    for(i = 0; i < iter; i++) //obtain 20 iterations of data        NOTE for some reason taking average gives incorrect value, just use iter 1 for now
    {

        readAccelData(accelbias);
        readGyroData(gyrobias);

        accelbiasX += (float)(accelbias[0]/16384.0);       // Hardcode, but g = 9.81 m/s^2 and 16384 is just (2^16)/2,
        accelbiasY += (float)(accelbias[1]/16384.0);       // Note the scaling factor I believe is set to 2g for the IMU
        accelbiasZ += (float)(accelbias[2]/16384.0);

        gyrobiasX += (float)(gyrobias[0]/131.0);       // 250 is a dps setting
        gyrobiasY += (float)(gyrobias[1]/131.0);       //
        gyrobiasZ += (float)(gyrobias[2]/131.0);
//        printf("GX Bias: %f\n", gyrobiasX);

//        printf("%f\n", int2Float(gyrobias[0]/131));

    }

    //formula for data calibrations, ((20iters data)/iterations) then use conversion formula respectively
    // Accleration XYZ bias
    Abias[0] = (accelbiasX/(iter + 1));
    Abias[1] = (accelbiasY/(iter + 1));
    Abias[2] = (accelbiasZ/(iter + 1));
    Gbias[0] = -1.0 * (gyrobiasX/(iter + 1));
    Gbias[1] = -1.0 * (gyrobiasY/(iter + 1));
    Gbias[2] = -1.0 * (gyrobiasZ/(iter + 1));

    printf("%f\n", Gbias[0]);

/*    Abias[0] = 0.4;
    Abias[1] = 1.4;
    Abias[2] = 1.19;
    Gbias[0] = -20.0;
    Gbias[1] = 5.0;
    Gbias[2] = 7.0;*/
}
