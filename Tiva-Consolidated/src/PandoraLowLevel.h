/**
 * PandoraLowLevel.h
 * @author: Nick Tremaroli
 * Contains all of the low-level features and functions
 * of Pandora
 */

#ifndef PANDORALOWLEVEL_H
#define PANDORALOWLEVEL_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "HAL/PWM_Driver.h"
#include "HAL/Timer_TIVA.h"
#include "EtherCAT_FrameData.h"

#include "Actuator.h"
#include "Joint.h"
#include "IMU.h"
#include "FTSensor.h"
#include "EasyCAT.h"

#define RED_LED GPIO_PIN_0
#define BLUE_LED GPIO_PIN_1
#define GREEN_LED GPIO_PIN_7

#define LED_PERIPH SYSCTL_PERIPH_GPION
#define LED_BASE GPIO_PORTB_BASE

#define LED_ON_DELAY_TIME 1000
#define LED_OFF_DELAY_TIME 500

/**
 * PandoraLowLevelSettings
 * A structure which holds settings regarding how
 * the microcontroller should operate
 */
struct PandoraLowLevelSettings
{
    bool softwareEStopEnable;
};
typedef struct PandoraLowLevelSettings PandoraLowLevelSettings;

/**
 * PandoraLowLevel
 * A structure consisting of all of the low level
 * variables needed for ONE Tiva. Each Tiva has 2 joints,
 * 2 actuators, various sensors, and variables to keep track
 * of the processID
 */
struct PandoraLowLevel
{
    // the two joints on pandora
    Joint joint0;
    Joint joint1;

    // the two actuators on pandora
    Actuator actuator0;
    Actuator actuator1;

    // the settings for pandora
    PandoraLowLevelSettings settings;

    // the signals to and from master
    uint8_t signalToMaster;
    uint8_t signalFromMaster;
    uint8_t prevSignalFromMaster;

    // the IMU
    IMU imu;

    // the FT sensor
    FTSensor ftSensor;

    // an easyCAT board for communication
    // to and from the master computer
    EasyCAT easyCAT;

    // for synchronizing frames from the master and the Tiva
    uint8_t prevProcessIdFromMaster;
    uint8_t processIdFromMaster;

    //  the number of initialization frame received
    uint8_t numberOfInitFramesReceived;

    // a flag to tell if pandora is initialized
    bool initialized;
};
typedef struct PandoraLowLevel PandoraLowLevel;

/**
 * floatByteData
 * A union to help with float to int conversion
 */
union FloatByteData
{
    uint8_t Byte[4];
    uint32_t intData;
    float floatData;
};
typedef union FloatByteData FloatByteData;

/**
 * ByteData
 * A union to help with int conversions
 */
union ByteData
{
    uint8_t Byte[4];
    uint16_t Word[2];
    uint32_t intData;
};
typedef union ByteData ByteData;


/*----------------------Initialization functions---------------------*/

// Construct and init the PandoraLowLevel object
PandoraLowLevel pandoraConstruct();

// Only initializes the Tiva's ethercat capabilities
// so it can read initialization data from the master.
// This function is meant to be called before tivaInit
void tivaInitEtherCAT(PandoraLowLevel* pandora);

// store the current initialization frame
void StoreCurrentInitFrame(PandoraLowLevel* pandora);

// initialize all of the tiva's peripherals needed
// after the initialization frame has been parsed
void tivaInit(PandoraLowLevel* pandora);

/*----------------------LED functions----------------------*/

// enable the LEDS which are used for debugging purposes
void enableDebugLEDS();

// disable the debug LEDS
void disableDebugLEDs();

// LED patterns for when the Tiva is not connected to the master
void notConnectedLEDS();

// LED patterns for when the Tiva is in idle mode
void idleLEDS();

// LED patterns for when the Tiva is in halt mode
void haltLEDS();

// LED patterns for when the Tiva is in control mode
void controlLEDS();

/*----------------------Low Level Tiva Functions----------------------*/

// Check if the actuators should be disabled
void checkActuatorDisable(PandoraLowLevel* athena);

// Decode the ethercat frame which was sent form the master
void storeDataFromMaster(PandoraLowLevel* athena);

// Process the data which was received from the master
bool processDataFromMaster(PandoraLowLevel* athena);

// load data in the ethercat frame to sent to the master
void loadDataForMaster(PandoraLowLevel* athena);

#endif /* PANDORALOWLEVEL_H */
