/**
 * LAN9252_TI_TIVA.c
 * @author: Nick Tremaroli
 * Contains all the low-level code for LAN9252 related functions
 */

#ifndef LAN9252_TI_H
#define LAN9252_TI_H

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

/**********************Start of Pre-Processor functions*******************************/
/**
 * the preprocessor calculates the parameters necessary to transfer out data
 * define the number of bytes
 */

/*********************Start of Output related Pre-Processor functions ****************/
/**
 * These preprocessor calculations are used for the output buffer which is transfered
 * from the Tiva to the LAN9252
 */
#if (!defined BYTE_NUM && !defined CUSTOM)
    #define BYTE_NUM 128
#endif

// define TOT_BYTE_NUM_OUT as the total number of byte we have to
// transfer in output in Standard Mode, 16, 32, 64, 128
#ifdef  BYTE_NUM
    #define TOT_BYTE_NUM_OUT  BYTE_NUM

// in custom mode, any number between 0 and 128
#else
    #define TOT_BYTE_NUM_OUT  CUST_BYTE_NUM_OUT
#endif

// if there are more than 64 bytes to transfer
// than split the data in two
#if TOT_BYTE_NUM_OUT > 64

    // define the number of bytes of the second transfer
    #define SEC_BYTE_NUM_OUT  (TOT_BYTE_NUM_OUT - 64)

    // round the number of bytes of the second transfer to 4 (a long) if necessary
    #if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00)
        #define SEC_BYTE_NUM_ROUND_OUT  ((SEC_BYTE_NUM_OUT | 0x03) + 1)
    #else
        #define SEC_BYTE_NUM_ROUND_OUT  SEC_BYTE_NUM_OUT
    #endif

    // number of bytes of the first transfer
    #define FST_BYTE_NUM_OUT  64
    #define FST_BYTE_NUM_ROUND_OUT  64

// if we have a max of 64 bytes to transfer
#else

    // define number of bytes of the first and only transfer
    #define FST_BYTE_NUM_OUT  TOT_BYTE_NUM_OUT

    // round the number of bytes to a long
    #if ((FST_BYTE_NUM_OUT & 0x03) != 0x00)
        #define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
    #else
        #define FST_BYTE_NUM_ROUND_OUT  FST_BYTE_NUM_OUT
    #endif

    // second transfer not being used
    #define SEC_BYTE_NUM_OUT  0
    #define SEC_BYTE_NUM_ROUND_OUT  0

#endif

/*********************End of Output related Pre-Processor functions ****************/


/*********************Start of Input related Pre-Processor functions ****************/
/**
 * These preprocessor calculations are used for the input buffer which the Tiva
 * reads from the LAN9252
 */

// define TOT_BYTE_NUM_IN as the total number of byte we have to
// transfer as an input.
// In Standard Mode 16, 32, 64, 128
#ifdef  BYTE_NUM
    #define TOT_BYTE_NUM_IN  BYTE_NUM

// in Custom Mode any number between 0 and 128
#else
  #define TOT_BYTE_NUM_IN  CUST_BYTE_NUM_IN
#endif

// If we have more than 64 bytes we have to split the transfer in two
#if TOT_BYTE_NUM_IN > 64

    // define the number of bytes of the second transfer
    #define SEC_BYTE_NUM_IN  (TOT_BYTE_NUM_IN - 64)

    // round the number of bytes of the second transfer to 4 (a long) if necessary
    #if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)
        #define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)
    #else
        #define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN
    #endif

    // the number of bytes of the first transfer
    #define FST_BYTE_NUM_IN  64
    #define FST_BYTE_NUM_ROUND_IN  64

// if we have a max of 64 bytes to transfer
#else

    // number of bytes of the first and only transfer
    #define FST_BYTE_NUM_IN  TOT_BYTE_NUM_IN

    // round the number of bytes to 4 (a long)
    #if ((FST_BYTE_NUM_IN & 0x03) != 0x00)
        #define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
    #else
        #define FST_BYTE_NUM_ROUND_IN  FST_BYTE_NUM_IN
    #endif

    // we don't use the second round
    #define SEC_BYTE_NUM_IN  0
    #define SEC_BYTE_NUM_ROUND_IN  0

#endif

/*********************End of Input related Pre-Processor functions ****************/

/*********************Start of Pre-Processor error checking ****************/

// Check that BYTE_NUM and CUST_BYTE_NUM_OUT are not declared at
// the same time
#ifdef BYTE_NUM

    // cannot be defined at the same time
    #ifdef CUST_BYTE_NUM_OUT
        #error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
        #error "define them correctly in file EasyCAT.h"
    #endif

    // cannot be defined at the same time
    #ifdef CUST_BYTE_NUM_IN
        #error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
        #error "define them correctly in file EasyCAT.h"
    #endif
#endif

// check that BYTE_NUM is only either 16, 32, 64, 128
#ifdef BYTE_NUM
    #if ((BYTE_NUM !=16) && (BYTE_NUM !=32) && (BYTE_NUM !=64)  && (BYTE_NUM !=128))
        #error "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
    #endif

// if CUSTOM_BYTE_NUM_OUT or CUSTOM_BYTE_NUM_IN is defined make sure
// it is within an appropriate range
#else
    #if (CUST_BYTE_NUM_OUT > 128)
        #error "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
    #endif

    #if (CUST_BYTE_NUM_IN > 128)
        #error "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
    #endif
#endif

/*********************End of Pre-Processor error checking ****************/

/*********************Start of LAN9252 register addresses**************************/

//---- access to EtherCAT registers -------------------

#define ECAT_CSR_DATA           0x0300      // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            0x0304      // EtherCAT CSR Interface Command Register

//---- access to EtherCAT process RAM -----------------

#define ECAT_PRAM_RD_ADDR_LEN   0x0308      // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        0x030C      // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   0x0310      // EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD        0x0314      // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA       0x0000      // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA       0x0020      // EtherCAT Process RAM Write Data FIFO

//---- EtherCAT registers -----------------------------

#define AL_CONTROL              0x0120      // AL control
#define AL_STATUS               0x0130      // AL status
#define AL_STATUS_CODE          0x0134      // AL status code
#define AL_EVENT                0x0220      // AL event request
#define AL_EVENT_MASK           0x0204      // AL event interrupt mask

#define WDOG_STATUS             0x0440      // watch dog status

#define SM0_BASE                0x0800      // SM0 base address (output)
#define SM1_BASE                0x0808      // SM1 base address (input)

//---- LAN9252 registers ------------------------------

#define HW_CFG                  0x0074      // hardware configuration register
#define BYTE_TEST               0x0064      // byte order test register
#define RESET_CTL               0x01F8      // reset register
#define ID_REV                  0x0050      // chip ID and revision
#define IRQ_CFG                 0x0054      // interrupt configuration
#define INT_EN                  0x005C      // interrupt enable

//---- LAN9252 flags ------------------------------------------------------------------------------

#define ECAT_CSR_BUSY     		0x8000
#define PRAM_ABORT        		0x40000000
#define PRAM_BUSY         		0x80
#define PRAM_AVAIL        		0x01
#define READY             		0x0800
#define DIGITAL_RST       		0x00000001

//---- EtherCAT flags -----------------------------------------------------------------------------

#define ALEVENT_CONTROL         0x0001
#define ALEVENT_SM              0x0010

//----- state machine ------------------------------------------------------------

#define ESM_INIT                0x01          // state machine control
#define ESM_PREOP               0x02          // (state request)
#define ESM_BOOT                0x03          //
#define ESM_SAFEOP              0x04          // safe-operational
#define ESM_OP                  0x08          // operational

//--- ESC commands --------------------------------------------------------------------------------

#define ESC_WRITE 		   		0x80
#define ESC_READ 		   		0xC0

//---- SPI ----------------------------------------------------------------------------------------

#define COMM_SPI_READ    		0x03
#define COMM_SPI_WRITE   		0x02

#define DUMMY_BYTE       		0xFFFF
#define ETHERCAT_SELECT         26U

// Error checking
#define ETHERCAT_INIT_TIMEOUT 1000

union ULONG
{
    uint32_t Long;
    uint16_t Word[2];
    uint8_t Byte[4];
};
typedef union ULONG ULONG;

/**********InputFrames**********/

// used to avoid padding
/**
 * ControlSignalEtherCATFrame_IN
 * Contains all of the data for an input control signal frame
 * from the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) ControlSignalEtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t actuator0Direction;
    float actuator0DutyCycle;
    uint8_t actuator1Direction;
    float actuator1DutyCycle;
    uint8_t reaminingBytes[116];
};
typedef struct ControlSignalEtherCATFrame_IN ControlSignalEtherCATFrame_IN;

/**
 * LocationDebugSignalEtherCATFrame_IN
 * Contains all of the data for an input location debug signal frame
 * from the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) LocationDebugSignalEtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t masterLocationGuess;
    uint8_t remainingBytes[125];
};
typedef struct LocationDebugSignalEtherCATFrame_IN LocationDebugSignalEtherCATFrame_IN;

/**
 * InitSignal0EtherCATFrame_IN
 * Contains all of the data for the 0th input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal0EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t actuator0_QEIBaseNumber;
    uint16_t actuator0_QEISampleRate;
    int32_t actuator0_QEICountsPerRotation;
    uint8_t actuator0_ForceSensorADCBaseNumber;
    float actuator0_ForceSensorSlope;
    float actuator0_ForceSensorOffset;
    uint8_t remainingBytes[109];
};
typedef struct InitSignal0EtherCATFrame_IN InitSignal0EtherCATFrame_IN;

/**
 * InitSignal1EtherCATFrame_IN
 * Contains all of the data for the 1st input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal1EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t actuator1_QEIBaseNumber;
    uint16_t actuator1_QEISampleRate;
    int32_t actuator1_QEICountsPerRotation;
    uint8_t actuator1_ForceSensorADCBaseNumber;
    float actuator1_ForceSensorSlope;
    float actuator1_ForceSensorOffset;
    uint8_t remainingBytes[109];
};
typedef struct InitSignal1EtherCATFrame_IN InitSignal1EtherCATFrame_IN;

/**
 * InitSignal2EtherCATFrame_IN
 * Contains all of the data for the 2nd input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal2EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t joint0_SSIBaseNumber;
    uint8_t joint0_SSIEncoderBrandRaw;
    uint16_t joint0_SSISampleRate;
    int8_t joint0_ReverseFactor;
    float joint0_RawZeroPosition;
    float joint0_RawForwardRangeOfMotion;
    float joint0_RawBackwardRangeOfMotion;
    uint8_t remainingBytes[108];
};
typedef struct InitSignal2EtherCATFrame_IN InitSignal2EtherCATFrame_IN;

/**
 * InitSignal3EtherCATFrame_IN
 * Contains all of the data for the 3rd input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal3EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t joint1_SSIBaseNumber;
    uint8_t joint1_SSIEncoderBrandRaw;
    uint16_t joint1_SSISampleRate;
    int8_t joint1_ReverseFactor;
    float joint1_RawZeroPosition;
    float joint1_RawForwardRangeOfMotion;
    float joint1_RamBackwardRangeOfMotion;
    uint8_t remainingBytes[108];
};
typedef struct InitSignal3EtherCATFrame_IN InitSignal3EtherCATFrame_IN;

/**
 * InitSignal4EtherCATFrame_IN
 * Contains all of the data for the 4th input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal4EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t imuEnable;
    uint8_t remainingBytes[124];
};
typedef struct InitSignal4EtherCATFrame_IN InitSignal4EtherCATFrame_IN;

/**
 * InitSignal5EtherCATFrame_IN
 * Contains all of the data for the 5th input initialization signal frame
 * form the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignal5EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t softwareEStopEnable;
    uint8_t remainingBytes[124];
};
typedef struct InitSignal5EtherCATFrame_IN InitSignal5EtherCATFrame_IN;

/**
 * InitSignalHeaderEtherCATFrame_IN
 * Contains all of the data for the header of the initialization frame
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) InitSignalHeaderEtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t remainingBytes[125];
};
typedef struct InitSignalHeaderEtherCATFrame_IN InitSignalHeaderEtherCATFrame_IN;

/**
 * ControlSignalEtherCATFrame_OUT
 * Contains all of the data for an output control signal to send
 * to the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) ControlSignalEtherCATFrame_OUT
{
    uint8_t signalToMaster;
    uint8_t masterProcessId;
    float actuator0ForceInNewtons;
    float actuator1ForceInNewtons;
    float joint0angleRadians;
    float joint1angleRadians;
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
    float Mx;
    float My;
    float Mz;

    float ftForceX;
    float ftForceY;
    float ftForceZ;
    float ftTorqueX;
    float ftTorqueY;
    float ftTorqueZ;
    uint8_t remainingBytes[50];
};
typedef struct ControlSignalEtherCATFrame_OUT ControlSignalEtherCATFrame_OUT;

/**
 * LocationDebugSignalEtherCATFrame_OUT
 * Contains all of the data for an output location debug signal
 * to send to the master computer
 * Note the "__attribute__((__packed__))" is to prevent padding
 */
struct __attribute__((__packed__)) LocationDebugSignalEtherCATFrame_OUT
{
    uint8_t signalFromMaster;
    uint8_t masterProcessId;
    uint8_t masterLocationGuess;
    uint8_t remainingBytes[125];
};
typedef struct LocationDebugSignalEtherCATFrame_OUT LocationDebugSignalEtherCATFrame_OUT;

/**
 * InitSignalEtherCATFrame_OUT
 * Contains all of the data for an output initialization signal
 * responce frame which is sent to the master
 */
struct __attribute__((__packed__)) InitSignalEtherCATFrame_OUT
{
    uint8_t signalFromMaster;
    uint8_t masterProcessId;
    uint8_t numInitializationFramesReceived;
    uint8_t totalNumberOfInitializationFrames;
    uint8_t remainingBytes[124];
};
typedef struct InitSignalEtherCATFrame_OUT InitSignalEtherCATFrame_OUT;

/**
 * EtherCATFrames_IN
 * A union of all of the input etherCAT frames which can be
 * received from the master computer
 */
union EtherCATFrames_IN
{
    ControlSignalEtherCATFrame_IN controlSignalFrame;
    LocationDebugSignalEtherCATFrame_IN locationDebugSignalFrame;
    InitSignalHeaderEtherCATFrame_IN initSignalHeader;
    InitSignal0EtherCATFrame_IN initSignal0Frame;
    InitSignal1EtherCATFrame_IN initSignal1Frame;
    InitSignal2EtherCATFrame_IN initSignal2Frame;
    InitSignal3EtherCATFrame_IN initSignal3Frame;
    InitSignal4EtherCATFrame_IN initSignal4Frame;
    InitSignal5EtherCATFrame_IN initSignal5Frame;
    uint8_t rawBytes[BYTE_NUM];
};
typedef union EtherCATFrames_IN EtherCATFrames_IN;

/**
 * EtherCATFrames_OUT
 * A union of all of the output etherCAT frames which can be
 * send to the master computer
 */
union EtherCATFrames_OUT
{
    ControlSignalEtherCATFrame_OUT controlSignalFrame;
    LocationDebugSignalEtherCATFrame_OUT locationDebugSignalFrame;
    InitSignalEtherCATFrame_OUT initSignalFrame;
    uint8_t rawBytes[BYTE_NUM];
};
typedef union EtherCATFrames_OUT EtherCATFrames_OUT;

// directly write to a register
void LAN9252WriteRegisterDirect(uint16_t address, uint32_t dataToSend);

// directly read from a register
uint32_t LAN9252ReadRegisterDirect(uint16_t address, uint16_t length);

// indirectly write to a register
void LAN9252WriteRegisterIndirect(uint16_t address, uint32_t dataToSend, uint16_t length);

// indirectly read from a register(s)
uint32_t LAN9252ReadRegisterIndirect(uint16_t address, uint16_t length);
uint32_t LAN9252ReadRegisterIndirectOneByte(uint16_t address);

// Read/Write to the RAM FIFO
void LAN9252ReadProcRamFifo(EtherCATFrames_IN *etherCATInputFrames);
void LAN9252WriteProcRamFifo(EtherCATFrames_OUT *etherCATOutputFrames);

// Transfer SPI data to and from the device
uint16_t SPI_Transfer(uint16_t value);

#endif
