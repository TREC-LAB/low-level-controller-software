/**
 * AthenaLowLevel.h
 * @author: Nick Tremaroli and Sam Schoedel
 * Contains all of the low-level features and functions
 * of Athena
 */

#ifndef ATHENALOWLEVEL_H
#define ATHENALOWLEVEL_H

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
struct PROCBUFFER_OUT MasterToTiva;
struct PROCBUFFER_IN TivaToMaster;


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
    DEBUG
};
typedef enum TivaLocations TivaLocations;

/**
 * TivaLocationBitSet
 * The bits that are used to determine which location
 * the Tiva is at during start up. These boolean values
 * are determined by the pin configuration of the Tiva
 * at startup
 */
struct TivaLocationBitSet
{
    bool Bit0 : 1;
    bool Bit1 : 1;
    bool Bit2 : 1;
};
typedef struct TivaLocationBitSet TivaLocationBitSet;

struct InitializationData
{
    void** initalizationDataBlock;
    uint8_t initializationFrameNumberToReceive;
};
typedef struct InitializationData InitializationData;

/**
 * AthenaLowLevel
 * A structure consisting of all of the low level
 * variables needed for ONE Tiva. Each Tiva has 2 joints,
 * a location, and variables to keep track of the processID
 */
struct AthenaLowLevel
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
typedef struct AthenaLowLevel AthenaLowLevel;

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

union ByteData
{
    uint8_t Byte[4];
    uint16_t Word[2];
    uint32_t intData;
};
typedef union ByteData ByteData;

/*-----------Initialization function----------- */

// Configure the Joints
Joint joint0Config(uint16_t sample_rate, TivaLocations tivaLocation);
Joint joint1Config(uint16_t sample_rate, TivaLocations tivaLocation);
enum EncoderBrand checkAbsoluteEncoderBrand(int boardSide, TivaLocations tivaLocation);
// Get the Tiva Location from the set of pins
TivaLocations getLocationsFromPins(void);
// Configure the location pins
void tivaLocationPinsConfig();
// Construct and init the AthenaLowLevel object
AthenaLowLevel athenaConstruct(uint16_t sample_rate);
void tivaInit(AthenaLowLevel* athena);

/*-----------LED functions-----------*/

// Configure the debug LEDS which are used for debugging purposes
void debugLEDSConfig();
// The different functions which generate different LED colors
// depending on what the Tiva is doing
void checkLocationLEDS(TivaLocations locationGuess, TivaLocations actualLocation);
void disableCheckLocationLEDs();
void notConnectedLEDS();
void idleLEDS();
void controlsLEDS();
void haltLEDS();
void modifyLEDs();

/*-----------Low Level Tiva Functions-----------*/

// Emit a PWM signal
//void sendSignal(AthenaLowLevel* athena);
// Read the Forces and Joint angles
//void updateForces(AthenaLowLevel* athena);
//void updateJointAngles(AthenaLowLevel* athena);
//void updateMotorPositions(AthenaLowLevel* athena);
//void updateMotorVelocities(AthenaLowLevel* athena, int32_t sample_rate, int32_t countsPerRotation);
// Create the ethercat Frame to send data to the master
void tivaInitEtherCAT();

void loadDataForMaster(AthenaLowLevel* athena);
// Decode the ethercat Frame which was sent form the master
void storeDataFromMaster(AthenaLowLevel* athena);
// Process the data which was received from the master
bool processDataFromMaster(AthenaLowLevel* athena);
// Check if the motors should be disabled
void checkMotorDisable(AthenaLowLevel* athena);

void storeInitFrame(AthenaLowLevel* athena);

void PandoraInit(AthenaLowLevel* athena);

// TODO: Move this to HAL/SSI_TIVA!!
void disableSSI1();


#endif /* ATHENALOWLEVEL_H */
