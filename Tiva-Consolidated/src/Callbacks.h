/*
 * Callbacks.h
 *
 *  Created on: Oct 5, 2022
 *      Author: Max Stelmack
 */

#ifndef CALLBACKS_H_
#define CALLBACKS_H_

//*****************************************************************************
//
//includes
//

// Standard library
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

// TivaWare
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"

// HAL
#include "HAL/LAN9252_TI_TIVA.h"
#include "HAL/Timer_TIVA.h"
#include "HAL/UART_TIVA.h"
#include "HAL/CAN_TI_TIVA.h"

// Implementation
#include "EtherCAT_FrameData.h"
#include "PandoraLowLevel.h"
//#include "Callbacks.h"

//*****************************************************************************
//
// functions prototypes
//
extern void logData(void);
extern void checkKeyboard(void);
extern void runMotor();
//extern bool EngageVirtualEStop(PandoraLowLevel* pandora);

//Actuator References
extern const uint8_t actuator_0; // Knee and Ankle 0

extern const uint8_t actuator_1; // Ankle 1

//Reference model

//Duty Cycle Variable for Printing

// UART Variables

extern char direction_str[10];//store user's input sin freq, later will be convert to float.
extern uint8_t direction_str_counter;//counter for storing user input into direction_str

extern char duty_cycle_str[5];//store user's input duty cycle, later will be convert to int.
extern uint8_t duty_cycle_str_counter;//counter for storing user input into omega_str

extern char sample_rate_str[5];//store user's input sample rate, later will be convert to int.
extern uint8_t sample_rate_str_counter;//counter for storing user input into sample_rate_freq_str

extern char lower_joint_limit_str[15];//store user's input lower joint limit, later will be convert to int.
extern char upper_joint_limit_str[15];//store user's input upper joint limit, later will be convert to int.
extern uint8_t joint_limit_str_counter;//counter for storing user input into lower/upper joint limit.

extern char logging_rate_str[5];//store user's input logging rate, later will be convert to int.
extern uint8_t logging_rate_str_counter;//counter for storing user input into logging rate.

extern char receivedStr[16];
extern uint8_t receivedStrCounter;
extern volatile float kneePitch;

extern volatile enum state_t{
    normal,
    testing,
    changing_direction,
    changing_duty_cycle,
    changing_sample_rate,
    changing_logging_rate,
    changing_lower_joint_limit,
    changing_upper_joint_limit}op_state;

extern volatile enum dof{
    none,
    knee,
    ankle_pitch,
    ankle_roll,}dof_state;

void lowLevelStartup(void);
void lowLevelStartup_noMaster(void);
void readSensors(void);
void readForceSensors(void);
void readJointEncoders(void);
void readMotorPositions(void);
void readMotorVelocities(void);
void checkEstop(void);

#endif /* CALLBACKS_H_ */
