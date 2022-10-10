//Author:       Hongxu Guo
//Organization: Virginia Tech
//Description:  This library is a modified version of the EasyCAT library
//              written by AB&T. This library is written for 32 bit
//              microcontrollers, especially for TI C2000 microcontrollers.


// Version:      2.0 - TIVA support
// Last modified 2/24/2021

// Modified by: Sam Schoedel and Nick Tremaroli

#ifndef LAN9252_TI
#define LAN9252_TI

//#include "device.h"
#include "../main.h"
//Global Functiokns
uint16_t EtherCAT_Init(void);
uint16_t EtherCAT_MainTask(void);
uint16_t SPI_Transfer(uint16_t Value);

//-----------------------------------------------------------------------------------------------



#if (!defined BYTE_NUM && !defined CUSTOM)			// if BYTE_NUM is not declared
  #define BYTE_NUM 128
#endif                                    			//


//-- the preprocessor calculates the parameters necessary to transfer out data ---

                                                    // define TOT_BYTE_NUM_OUT as the
                                                    // total number of byte we have to
                                                    // transfer in output
                                                    // this must match the ESI XML
                                                    //
#ifdef  BYTE_NUM                                    // in Standard Mode
  #define TOT_BYTE_NUM_OUT  BYTE_NUM                // 16, 32, 64 or 128
                                                    //
#else                                               // in Custom Mode
  #define TOT_BYTE_NUM_OUT  CUST_BYTE_NUM_OUT       // any number between 0 and 128
#endif                                              //


#if TOT_BYTE_NUM_OUT > 64                           // if we have more then  64 bytes
                                                    // we have to split the transfer in two

  #define SEC_BYTE_NUM_OUT  (TOT_BYTE_NUM_OUT - 64) // number of bytes of the second transfer

  #if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the second transfer
                                                    // rounded to long
    #define SEC_BYTE_NUM_ROUND_OUT  ((SEC_BYTE_NUM_OUT | 0x03) + 1)
  #else                                             //
    #define SEC_BYTE_NUM_ROUND_OUT  SEC_BYTE_NUM_OUT//
  #endif                                            //

  #define FST_BYTE_NUM_OUT  64                      // number of bytes of the first transfer
  #define FST_BYTE_NUM_ROUND_OUT  64                // number of bytes of the first transfer
                                                    // rounded to 4 (long)

#else                                               // if we have max 64 bytes we transfer
                                                    // them in just one round

  #define FST_BYTE_NUM_OUT  TOT_BYTE_NUM_OUT        // number of bytes of the first and only transfer

  #if ((FST_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the first and only transfer
                                                    // rounded to 4 (long)
    #define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
  #else                                             //
    #define FST_BYTE_NUM_ROUND_OUT  FST_BYTE_NUM_OUT//
  #endif                                            //

  #define SEC_BYTE_NUM_OUT  0                       // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_OUT  0                 //

#endif


//-- the preprocessor calculates the parameters necessary to transfer in data ---

                                                    // define TOT_BYTE_NUM_IN as the
                                                    // total number of byte we have to
                                                    // transfer in input
                                                    // this must match the ESI XML
                                                    //
#ifdef  BYTE_NUM                                    // in Standard Mode
  #define TOT_BYTE_NUM_IN  BYTE_NUM                 // 16, 32, 64 or 128
                                                    //
#else                                               // in Custom Mode
  #define TOT_BYTE_NUM_IN  CUST_BYTE_NUM_IN         // any number between 0 and 128
#endif                                              //


#if TOT_BYTE_NUM_IN > 64                            // if we have more then  64 bytes
                                                    // we have to split the transfer in two

  #define SEC_BYTE_NUM_IN  (TOT_BYTE_NUM_IN - 64)   // number of bytes of the second transfer

  #if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the second transfer
                                                    // rounded to 4 (long)
    #define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)
  #else                                             //
    #define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN  //
  #endif                                            //

  #define FST_BYTE_NUM_IN  64                       // number of bytes of the first transfer
  #define FST_BYTE_NUM_ROUND_IN  64                 // number of bytes of the first transfer
                                                    // rounded to 4 (long)

#else                                               // if we have max 64 bytes we transfer
                                                    // them in just one round

  #define FST_BYTE_NUM_IN  TOT_BYTE_NUM_IN          // number of bytes of the first and only transfer

  #if ((FST_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the first and only transfer
                                                    // rounded to 4 (long)
    #define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
  #else                                             //
    #define FST_BYTE_NUM_ROUND_IN  FST_BYTE_NUM_IN  //
  #endif                                            //

  #define SEC_BYTE_NUM_IN  0                        // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_IN  0                  //

#endif


//----------------- sanity check -------------------------------------------------------


#ifdef BYTE_NUM                     // STANDARD MODE and CUSTOM MODE
                                    // cannot be defined at the same time
  #ifdef CUST_BYTE_NUM_OUT
    #error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
    #endif

  #ifdef CUST_BYTE_NUM_IN
    #error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
  #endif
#endif

#ifdef BYTE_NUM                     //--- for BYTE_NUM we accept only 16  32  64  128 --

  #if ((BYTE_NUM !=16) && (BYTE_NUM !=32) && (BYTE_NUM !=64)  && (BYTE_NUM !=128))
    #error "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
  #endif

#else
                                   //--- CUST_BYTE_NUM_OUT and CUST_BYTE_NUM_IN --------
                                   //    must be max 128
  #if (CUST_BYTE_NUM_OUT > 128)
    #error "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif

  #if (CUST_BYTE_NUM_IN > 128)
    #error "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif

#endif


//---- LAN9252 registers --------------------------------------------------------------------------

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


typedef union
{
    uint32_t Long;
    uint16_t Word[2];
    uint8_t Byte[4];
}ULONG;

//-------------------------------------------  Input/Output buffers for Standard Mode -----------
#ifdef BYTE_NUM

//  typedef struct						      //-- output buffer -----------------
//  {		 									  //
//    uint16_t  Word [BYTE_NUM];                 //
//  } PROCBUFFER_OUT;							  //
//
//  typedef struct                              //-- input buffer ------------------
//  {											  //
//    uint16_t  Word [BYTE_NUM];                 //
//  } PROCBUFFER_IN;                            //

struct PROCBUFFER_OUT                             //-- output buffer -----------------
{                                           //
    uint8_t  Byte [BYTE_NUM];                 //
};
typedef struct PROCBUFFER_OUT PROCBUFFER_OUT;

struct PROCBUFFER_IN                             //-- input buffer ------------------
{                                           //
    uint8_t  Byte [BYTE_NUM];                 //
};
typedef struct PROCBUFFER_IN PROCBUFFER_IN;

//PROCBUFFER_OUT MasterToTiva;
//PROCBUFFER_IN TivaToMaster;

// InputFrames
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


struct __attribute__((__packed__)) LocationDebugSignalEtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t masterLocationGuess;
    uint8_t remainingBytes[125];
};
typedef struct LocationDebugSignalEtherCATFrame_IN LocationDebugSignalEtherCATFrame_IN;


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

struct __attribute__((__packed__)) InitSignal4EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t imuEnable;
    uint8_t remainingBytes[124];
};
typedef struct InitSignal4EtherCATFrame_IN InitSignal4EtherCATFrame_IN;

struct __attribute__((__packed__)) InitSignal5EtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t softwareEStopEnable;
    uint8_t remainingBytes[124];
};
typedef struct InitSignal5EtherCATFrame_IN InitSignal5EtherCATFrame_IN;

struct __attribute__((__packed__)) InitSignalHeaderEtherCATFrame_IN
{
    uint8_t signalFromMaster;
    uint8_t masterProcessID;
    uint8_t currentInitFrame;
    uint8_t remainingBytes[125];
};
typedef struct InitSignalHeaderEtherCATFrame_IN InitSignalHeaderEtherCATFrame_IN;

// OutputFrames

//#pragma pack(1)     // used to avoid padding
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

//#pragma pack(1)     // used to avoid padding
struct __attribute__((__packed__)) LocationDebugSignalEtherCATFrame_OUT
{
    uint8_t signalFromMaster;
    uint8_t masterProcessId;
    uint8_t masterLocationGuess;
    uint8_t remainingBytes[125];
};
typedef struct LocationDebugSignalEtherCATFrame_OUT LocationDebugSignalEtherCATFrame_OUT;

//#pragma pack(1)     // used to avoid padding
struct __attribute__((__packed__)) InitSignalEtherCATFrame_OUT
{
    uint8_t signalFromMaster;
    uint8_t masterProcessId;
    uint8_t numInitializationFramesReceived;
    uint8_t totalNumberOfInitializationFrames;
    uint8_t remainingBytes[124];
};
typedef struct InitSignalEtherCATFrame_OUT InitSignalEtherCATFrame_OUT;

// the input frames which come from the master
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

//#pragma pack(1)
union EtherCATFrames_OUT
{
    ControlSignalEtherCATFrame_OUT controlSignalFrame;
    LocationDebugSignalEtherCATFrame_OUT locationDebugSignalFrame;
    InitSignalEtherCATFrame_OUT initSignalFrame;
    uint8_t rawBytes[BYTE_NUM];
};
typedef union EtherCATFrames_Out EtherCATFrames_OUT;

//EtherCATFrames_IN etherCATInputFrames;
//EtherCATFrames_OUT etherCATOutputFrames;

#endif


#endif
