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

#include "HAL/LAN9252_TI_TIVA.h"
#include "HAL/PWM_Driver.h"

#include "Actuator.h"
#include "Joint.h"
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE

#define LED_ON_DELAY_TIME 1000
#define LED_OFF_DELAY_TIME 500


// For ethercat communication
//struct PROCBUFFER_OUT MasterToTiva;
//struct PROCBUFFER_IN TivaToMaster;
union EtherCATFrames_IN etherCATInputFrames;
union EtherCATFrames_OUT etherCATOutputFrames;


/**
 * TivaLocations
 * An enumeration of all of the different locations
 * the Tiva Board can be on Athena. The enumeration
 * values start at 1
 */
enum TivaLocations
{
    notValidLocation,
    hipL,
    hipR,
    thighL,
    thighR,
    ankleL,
    ankleR,
    DEBUG   // a location for debugging
};
typedef enum TivaLocations TivaLocations;

/**
 * TivaLocationBitSet
 * The bits that are used to determine which location
 * the Tiva is at. These boolean values are determined
 * by the pin configuration of the Tiva at startup
 */
struct TivaLocationBitSet
{
    bool Bit0 : 1;
    bool Bit1 : 1;
    bool Bit2 : 1;
};
typedef struct TivaLocationBitSet TivaLocationBitSet;

/**
 * InitializationData
 * Contains a dynamically allocated 2D array of the raw initialization
 * data sent by the master.
 */
struct InitializationData
{
    void** initalizationDataBlock;      // a pointer to the 2D array
    uint8_t numberOfInitFramesReceived; // the number of initialization frames received
};
typedef struct InitializationData InitializationData;

/**
 * PandoraLowLevel
 * A structure consisting of all of the low level
 * variables needed for ONE Tiva. Each Tiva has 2 joints,
 * 2 actuators, a location on pandora, and variables to keep track
 * of the processID
 */
struct PandoraLowLevel
{
    Joint joint0;
    Joint joint1;
    Actuator actuator0;
    Actuator actuator1;
    TivaLocations location;
    TivaLocations masterLocationGuess;
    uint8_t signalToMaster;
    uint8_t signalFromMaster;
    uint8_t prevSignalFromMaster;

    // for synchronizing frames from the master and the Tiva
    uint8_t prevProcessIdFromMaster;
    uint8_t processIdFromMaster;

    InitializationData initializationData;
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

// Configure the location pins
void tivaLocationPinsConfig();

// Get the Tiva Location from the set of pins
TivaLocations getLocationsFromPins(void);

// Only initializes the Tiva's ethercat capabilities
// so it can read initialization data from the master.
// This function is meant to be called before tivaInit
void tivaInitEtherCAT();

// store the raw initialization frame from the master
void storeInitFrame(PandoraLowLevel* pandora);

// parses the initialization data block and applies the settings
// to the Tiva
void ApplyInitializationSettings(PandoraLowLevel* pandora);

// initialize all of the tiva's peripherals needed
// after the initialization frame has been parsed
void tivaInit(PandoraLowLevel* pandora);

/*----------------------LED functions----------------------*/

// enable the LEDS which are used for debugging purposes
void enableDebugLEDS();

// disable the debug LEDS
void disableDebugLEDs();

// LED patterns for when the master is pinging a Tiva location
void checkLocationLEDS(TivaLocations locationGuess, TivaLocations actualLocation);

// LED patterns for when the Tiva is not connected to the master
void notConnectedLEDS();

// LED patterns for when the Tiva is in idle mode
void idleLEDS();

// LED patterns for when the Tiva is in halt mode
void haltLEDS();

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
