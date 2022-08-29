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

    // Hardware Initialization
    MPU_Config();

    // Power ON for MPU (MOSFET CONTROL)
    MPU_PowerSwitch(true);


    //  Power cycle MPU chip before every SW initialization
    MPU_PowerSwitch(false);
    SysCtlDelay(20000);
    MPU_PowerSwitch(true);
    SysCtlDelay(30000);


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
void IMU_GET(IMUSensor* IMUsensed)
{
    Read_IMU();
    SaveIMUData(IMUsensed);
}

/**
 *
 This Function initializes the I2C communication for the MPU 9250
 *
 **/
//void HAL_MPU_Init(void((*custHook)(void)))
void MPU_Config()
{
    //  Enable peripherals in use. Also reset I2C2 at the end to allow calling
        //  this function at any point in order to reset I2C interface.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

        // Enable I2C communication interface, SCL, SDA lines

        GPIOPinConfigure(GPIO_PE5_I2C2SDA);
        GPIOPinConfigure(GPIO_PE4_I2C2SCL);
        GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

        //  Enable I2C master interface
        I2CMasterEnable(MPU9250_I2C_BASE);

        // Run I2C bus in high-speed mode, 400kHz speed (May not be at 400 kHz atm)
        I2CMasterInitExpClk(MPU9250_I2C_BASE, SysCtlClockGet(), true);

        //  Configure power-switch pin
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);

        //  Configure interrupt pin to receive output   // NOT USED
        //      (not used as actual interrupts)
//        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//        MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
//        MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

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
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
//    HAL_DelayUS(1000*100); // Wait for all registers to reset
    SysCtlDelay(100000);

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
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
    uint8_t c = HAL_MPU_ReadByte(MPU9250_ADDRESS, GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    //c |= 0x03;
    // Write new GYRO_CONFIG value to register
    HAL_MPU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    c = HAL_MPU_ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value     // This may actually work, hard to confirm
    c = HAL_MPU_ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
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
IMUSensor IMU_Struct_Config(void)
{

    IMUSensor IMUsense;

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
void SaveIMUData(IMUSensor* IMUsense)
{

    IMUsense->Ax = ConvAccel[0];
    IMUsense->Ay = ConvAccel[1];
    IMUsense->Az = ConvAccel[2];
    IMUsense->Gx = ConvGyro[0];
    IMUsense->Gy = ConvGyro[1];
    IMUsense->Gz = ConvGyro[2];

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
            Converted_Data[i] = int2Float(Raw_Data[i])/GyroFactor;         // note 131 is LSB/(degrees/s)        // 250 is a dps setting
        }

    }
    else if(TypeFlag == AccelType)
    {
        for (i = 0; i < 3; i++)
        {                                                               // units are in g for gravity
            Converted_Data[i] = int2Float(Raw_Data[i])/AccelFactor;         // Hardcode, but g = 9.81 m/s^2  16384 is taken fromo mpu 9250 datasheet and LSB Senstivity
        }
    }

}

/**
 * Read several bytes from I2C device
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param count number of bytes to red
 * @param dest pointer to data buffer in which data is saved after reading
 */
uint8_t MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                          uint16_t length, uint8_t* data)
{
    uint16_t i;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  but there are problems if it's not done this way :/
    I2CMasterSlaveAddrSet(I2C1_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(I2C1_BASE, regAddress);
    //  Send start sequence and address, followed by register address. Use burst
    //  mode as we're reading more than 1 byte                                               This sends controls byte and register address byte to slave device (MPU 9250)
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

//    HAL_DelayUS(4);
//    SysCtlDelay(40);      // not sure if this delay was necessary

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C1_BASE));

    //  Change address to reading mode
    I2CMasterSlaveAddrSet(I2C1_BASE, I2Caddress, true);

    //  Check how many bytes we need to receive
    if (length == 1)
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//        HAL_DelayUS(4);
//        SysCtlDelay(10);
        while(I2CMasterBusy(I2C1_BASE));
        data[0] = (uint8_t)(I2CMasterDataGet(I2C1_BASE) & 0xFF);

        for (i = 1; i < (length-1); i++)
        {
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//            HAL_DelayUS(4);
//            SysCtlDelay(10);
            while(I2CMasterBusy(I2C1_BASE));
            data[i] = (uint8_t)(I2CMasterDataGet(I2C1_BASE) & 0xFF);
        }
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

//    HAL_DelayUS(4);
//    SyswCtlDelay(10);
    while(I2CMasterBusy(I2C1_BASE));
    data[length-1] = (uint8_t)(I2CMasterDataGet(I2C1_BASE) & 0xFF);

    return 0;
}

/**
 * Read raw accelerometer data into a provided buffer
 * @param destination Buffer to save x, y, z acceleration data (min. size = 3)
 */
void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array        // Note 0x68 is the IMU slave address
  MPU_ReadBytes(0x68, ACCEL_XOUT_H, 6, &rawData[0]);        //#define ACCEL_XOUT_H       0x3B

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
  MPU_ReadBytes(0x68, GYRO_XOUT_H, 6, &rawData[0]);  //#define GYRO_XOUT_H        0x43

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @param data Data to write into the register of I2C device
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    //  Set I2C address of MPU, writing mode
    I2CMasterSlaveAddrSet(I2C1_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(I2C1_BASE, regAddress);
    //  Send start sequence and address, followed by register address
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//    HAL_DelayUS(4);
    SysCtlDelay(40);
    while(I2CMasterBusy(I2C1_BASE));

    //  Send register data to write and stop sequence
    I2CMasterDataPut(I2C1_BASE, data);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//    HAL_DelayUS(4);
    SysCtlDelay(40);
    while(I2CMasterBusy(I2C1_BASE));
}

/**
 * Read one byte of data from I2C device
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @return Data received from I2C device
 */
uint8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t data;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  bute there are problems if it's not done this way :/
    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    //  Send start sequence and address, followed by register address
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    //  Perform s single receive from I2C bus
    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));
    //  Read a byte from receiving buffer
    data = I2CMasterDataGet(MPU9250_I2C_BASE);

    //  We're dealing with 8-bit data so return only lower 8 bits
    return (data & 0xFF);
}

/**
 This function takes so many iterations of data, than takes an average of this data and applies it to taring variables
     which than are applied inside the RawToRefine function.

     Currently only one iter will work, issue arise when iter>1,
 */
void CalibrateMPU(float * Abias, float * Gbias)
{
    uint16_t i;
    int16_t accelbias[3], gyrobias[3];
    float accelbiasX, accelbiasY, accelbiasZ, gyrobiasX, gyrobiasY, gyrobiasZ;

    accelbiasX = 0.0, accelbiasY = 0.0, accelbiasZ = 0.0, gyrobiasX = 0.0, gyrobiasY = 0.0, gyrobiasZ = 0.0;

    int iter = 1;


    for(i = 0; i < iter; i++) //obtain 20 iterations of data        NOTE for some reason taking average gives incorrect value, just use iter 1 for now
    {

        readAccelData(accelbias);
        readGyroData(gyrobias);

        accelbiasX = abs(int2Float(accelbias[0])/16384) + accelbiasX;       // Hardcode, but g = 9.81 m/s^2 and 16384 is just (2^16)/2,
        accelbiasY = abs(int2Float(accelbias[1])/16384) + accelbiasY;       // Note the scaling factor I believe is set to 2g for the IMU
        accelbiasZ = abs(int2Float(accelbias[2])/16384) + accelbiasZ;

        gyrobiasX = abs(int2Float(gyrobias[0])/131) + gyrobiasX;       // 250 is a dps setting
        gyrobiasY = abs(int2Float(gyrobias[1])/131) + gyrobiasY;       //
        gyrobiasZ = abs(int2Float(gyrobias[2])/131) + gyrobiasZ;

    }

    //formula for data calibrations, ((20iters data)/iterations) then use conversion formula respectively
    // Accleration XYZ bias
    Abias[0] = (accelbiasX/iter) ;
    Abias[1] = (accelbiasY/iter) ;
    Abias[2] = (accelbiasZ/iter) ;

    Gbias[0] = (gyrobiasX/iter) ;
    Gbias[1] = (gyrobiasY/iter) ;
    Gbias[2] = (gyrobiasZ/iter) ;

}

/**
 *
  Convert a integer datatype to a floating point data type
 *
 **/
float int2Float(int integerValue)
{
    // In theory this should convert data type from a integer(16 bit) to float(32 bit)
    float ConversionValue = integerValue ;
    return(ConversionValue);
}
