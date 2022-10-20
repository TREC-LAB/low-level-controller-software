/**
 * FTSensor.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for FT sensor related functions
 */

#include "FTSensor.h"

/**
 * ftSensorConstruct
 * Constructs an FT sensor and initializes
 * all of the values accordingly
 */
FTSensor ftSensorConstruct(void)
{
    FTSensor ftSensor;

    // initialize the force values to 0
    ftSensor.forceX = 0.0;
    ftSensor.forceY = 0.0;
    ftSensor.forceZ = 0.0;

    // initialize the torque values to 0
    ftSensor.torqueX = 0.0;
    ftSensor.torqueY = 0.0;
    ftSensor.torqueZ = 0.0;

    // initialize the force bias values to 0
    ftSensor.bias.forceXBias = 0.0;
    ftSensor.bias.forceYBias = 0.0;
    ftSensor.bias.forceZBias = 0.0;

    // initialize the torque bias values to 0
    ftSensor.bias.torqueXBias = 0.0;
    ftSensor.bias.torqueYBias = 0.0;
    ftSensor.bias.torqueZBias = 0.0;

    return ftSensor;
}

/**
 * ftSensorEnable
 * enables the FT sensor and the corresponding
 * CAN Tx and Rx components
 */
void ftSensorEnable(FTSensor* ftSensor)
{
    // initialize CAN given the CAN rate
    CANInitial(500000);

    // Initialize Rx0 to receive data from the FT board
    CANRx0Initial(&ftSensor->RxData0, 0x1b5, 2);

    // Initialize Rx1 to receive data from the FT board
    CANRx1Initial(&ftSensor->RxData1, 0x1b6, 3);

    // Initialize Tx0 to send data to the FT board
    CANTxInitial(&ftSensor->TxData0, 0x1b0, 1);

    // To an initial write/read to the FT sensor
    // to initialize CAN
    SendFTSensorData(ftSensor);
    readForceTorqueData(ftSensor);

    // Send a data request to the FT sensor
    SendFTSensorData(ftSensor);

    // calibrate the FT sensor
    ftSensorCalibrate(ftSensor);
}

/**
 * ftSensorCalibrate
 * calibrates the FT sensor and sets the
 * biases accordingly
 * @param ftSensor: a pointer to the FT sensor
 * to calibrate
 */
void ftSensorCalibrate(FTSensor* ftSensor)
{
    // Read data from the FT Sensor
    ReadCAN(&ftSensor->RxData0);
    ReadCAN(&ftSensor->RxData1);
    readForceTorqueData(ftSensor);

    // set the force biases
    ftSensor->bias.forceXBias = -1.0 * ftSensor->forceX;
    ftSensor->bias.forceYBias = -1.0 * ftSensor->forceY;
    ftSensor->bias.forceZBias = -1.0 * ftSensor->forceZ;

    // set the torque biases
    ftSensor->bias.torqueXBias = -1.0 * ftSensor->torqueX;
    ftSensor->bias.torqueYBias = -1.0 * ftSensor->torqueY;
    ftSensor->bias.torqueZBias = -1.0 * ftSensor->torqueZ;
}

/**
 * SendFTSensorData
 * Sends data to the FT sensor which requests the
 * latest force and torque data
 * @param ftSensor: the FT sensor to send data too
 */
void SendFTSensorData(FTSensor* ftSensor)
{
    CANSend(&ftSensor->TxData0);
}

/**
 * readForceTorqueData
 * reads force and torque data from the FT sensor,
 * computes the output using the biases and stores it accordingly
 * @param ftSensor: the FT sensor to read force torque data from
 */
void readForceTorqueData(FTSensor* ftSensor)
{
    // Read from the CAN bus
    ReadCAN(&ftSensor->RxData0);
    ReadCAN(&ftSensor->RxData1);

    // These factor is hardware configuration in the Net FT board
    // use these factors to compute the force and torque respectively
    float torqueFactor = 1000000.0 / 611.0;
    float forceFactor = 1000000.0 / 35402.0;

    // store the force value in the X direction
    ftSensor->forceX = (BinaryToDecimal(ftSensor->RxData0.msgRxData[0], ftSensor->RxData0.msgRxData[1])/forceFactor) + ftSensor->bias.forceXBias;

    // store the torque value in the X direction
    ftSensor->torqueX = (BinaryToDecimal(ftSensor->RxData0.msgRxData[2], ftSensor->RxData0.msgRxData[3])/torqueFactor) + ftSensor->bias.torqueXBias;

    // store the force value in the Y direction
    ftSensor->forceY = (BinaryToDecimal(ftSensor->RxData0.msgRxData[4], ftSensor->RxData0.msgRxData[5])/forceFactor) + ftSensor->bias.forceYBias;

    // store the torque value in the Y direction
    ftSensor->torqueY = (BinaryToDecimal(ftSensor->RxData0.msgRxData[6], ftSensor->RxData0.msgRxData[7])/torqueFactor) + ftSensor->bias.torqueYBias;

    // store the force value in the Z direction
    ftSensor->forceZ = (BinaryToDecimal(ftSensor->RxData1.msgRxData[0], ftSensor->RxData1.msgRxData[1])/forceFactor) + ftSensor->bias.forceZBias;

    // store the torque value in the Z direction
    ftSensor->torqueZ = (BinaryToDecimal(ftSensor->RxData1.msgRxData[2], ftSensor->RxData1.msgRxData[3])/torqueFactor) + ftSensor->bias.torqueZBias;
}
