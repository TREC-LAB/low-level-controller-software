/**
 * IMU.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding an FT Sensor
 */

#ifndef IMU_IMU_TIVA_H_
#define IMU_IMU_TIVA_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "HAL/I2C_Tiva.h"

#define GYROFACTOR_2000DPS      ((2000.0 / 32767.5))
#define ACCELEROMETERFACTOR_16G  (9.81 * (16.0/32767.5))

/**
 * AccelerationData
 * Holds the raw data and actual data regarding
 * acceleration data
 */
struct AccelerationData
{
    // raw data
    int16_t AxRaw;
    int16_t AyRaw;
    int16_t AzRaw;

    // actual floating point data
    float Ax;
    float Ay;
    float Az;
};
typedef struct AccelerationData AccelerationData;

/**
 * GyroData
 * Holds the raw data, actual data and gyro biases
 * regarding gyro data
 */
struct GyroData
{
    // raw data
    int16_t GxRaw;
    int16_t GyRaw;
    int16_t GzRaw;

    // actual data
    float Gx;
    float Gy;
    float Gz;

    // gyro bias data
    float GxBias;
    float GyBias;
    float GzBias;
};
typedef struct GyroData GyroData;

/**
 * MagnetometerData
 * Holds the raw data, actual data, magnetometer biases,
 * factory biases, and scale regarding mangetometer data
 */
struct MagnetometerData
{
    // raw data
    int16_t MxRaw;
    int16_t MyRaw;
    int16_t MzRaw;

    // actual data
    float Mx;
    float My;
    float Mz;

    // factory bias data
    float MxFactoryBias;
    float MyFactoryBias;
    float MzFactoryBias;

    // bias data
    float MxBias;
    float MyBias;
    float MzBias;

    // scale data
    float MxScale;
    float MyScale;
    float MzScale;
};
typedef struct MagnetometerData MagnetometerData;

/**
 * IMU
 * Contains all of the data needed by an IMU on the Tiva
 */
struct IMU
{
    bool enabled;
    AccelerationData accelerationData;
    GyroData gyroData;
    MagnetometerData magnetometerData;

};
typedef struct IMU IMU;

/**
 * AccelerationScale
 * The scale settings regarding acceleration for the IMU
 */
enum AccelerationScale
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

/**
 * GyroScale
 * The scale settings regarding gyroscope readings for the IMU
 */
enum GyroScale
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

/**
 * MagentometerOutputBits
 * The number of bits regarding resolution of the magnetometer
 */
enum MagnetometerOutputBits
{
    fourteenBits,
    sixteenBits
};

// construct the IMU
IMU imuConstruct(void);

// initalize the IMU
void imuInitialize(IMU* imu);

// enable the IMU
void imuEnable(IMU* imu);

// calibrate the imu
void imuCalibrate(IMU* imu);

// read all sensor data from the IMU
void readSensorData(IMU* imu);

// read to the registers of the AK8963 (the magnetometer)
void readAK8963Registers(uint8_t subAddress, uint8_t length, uint8_t* dest);

// write to the register of the AK8963 (the magnetometer)
void writeAK8963Register(uint8_t subAddress, uint8_t data);

/**     MPU9250 - related macros        */

#define MAG_MODE 0x06
#define M16BIT_MAG_RESOLUTION   (10. * 4912. / 32760.0)
#define M14BIT_MAG_RESOLUTION   (10. * 4912. / 8190.0)


#define MPU9250_I2C_BASE I2C1_BASE

/*      Device addresses        */
#define AK8963_ADDRESS   (0x0C) // Magnetometer I2C address
#define MPU9250_ADDRESS  0x68 // Device address when ADO = 0

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x00 // (AKA WIA) should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2     0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A       0x10

#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define MOT_DUR           0x20
// Zero-motion detection threshold bits [7:0]
#define ZMOT_THR          0x21
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
#define ZRMOT_DUR         0x22

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250   0x75 // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

#define PWR_MGMNT_1 0x6B
#define CLOCK_SEL_PLL  0x01
#define I2C_MST_EN   0x20
#define I2C_MST_CLK  0x0D
#define AK8963_PWR_DOWN  0x00
#define AK8963_CNTL1     0x0A
#define PWR_RESET        0x80
#define AK8963_RESET     0x01
#define PWR_MGMNT_2      0x6C
#define SEN_ENABLE       0x00
#define ACCEL_FS_SEL_16G 0x18
#define GYRO_FS_SEL_2000DPS 0x18
#define GYRO_DLPF_184    0x01
#define CONFIG           0x1A
#define ACCEL_DLPF_184   0x01
#define SMPDIV           0x19
#define I2C_READ_FLAG    0x80
#define I2C_SLV0_EN      0x80
#define AK8963_FUSE_ROM  0x0F
#define AK8963_ASA       0x10
#define AK8963_CNT_MEAS2 0x16
#define ACCEL_OUT        0x3B

#endif /* IMU_IMU_TIVA_H_ */
