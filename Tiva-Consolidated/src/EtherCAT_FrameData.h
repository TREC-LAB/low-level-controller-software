#pragma once

/**
 * EtherCAT_FrameData.h
 *
 * A header file which contains all of the constants
 * needed for the EtherCAT communication on the Tiva
 */

#define FRAME_SIZE  32

// frame[0] determines how to interpret incoming data
#define SIGNAL_INDEX    0

// for frame sychronization
#define PROCESS_ID_INDEX      1

// for initialization
#define INITIALIZATION_FRAME_NUMBER_INDEX   2

// Commands that can be received from master
#define NOT_CONNECTED           0
#define HALT_SIGNAL             1
#define LOCATION_DEBUG_SIGNAL   2 // For determining TIVA location
#define CONTROL_SIGNAL          3 // When moving motors
#define IDLE_SIGNAL             4 // When master is active but not sending or receiving anything
//#define MODIFY_FORCES           5
//#define MODIFY_LIMITS           6
#define INITIALIZATION_SIGNAL   5

// Commands that can be sent to master
#define NOT_CONNECTED           0
#define HALT_SIGNAL_TM          1
#define REQUEST_FORCE_PARAMS_TM 2
#define NORMAL_OPERATION        3


// FOR LOCATION DEBUG
#define MASTER_LOCATION_GUESS   2

// These defines will be compared to that of frame[1]
#define LEFT_THIGH      1
#define RIGHT_THIGH     2
#define LEFT_KNEE       3
#define RIGHT_KNEE      4
#define LEFT_ANKLE      5
#define RIGHT_ANKLE     6


// FOR CONTROLS
// The indexes for the frame that is sending data
// to the master computer
// starts with the MSB!! so Byte 1 (B1) = MSB

#define FORCE0_B1       2
#define FORCE0_B2       3
#define FORCE0_B3       4
#define FORCE0_B4       5

#define FORCE1_B1       6
#define FORCE1_B2       7
#define FORCE1_B3       8
#define FORCE1_B4       9

#define ENCODER0_B1     10
#define ENCODER0_B2     11
#define ENCODER0_B3     12
#define ENCODER0_B4     13

#define ENCODER1_B1     14
#define ENCODER1_B2     15
#define ENCODER1_B3     16
#define ENCODER1_B4     17



// The indexes for the frame that is receiving data from the master computer
// Starts with the MSB. Byte 1 (B1) = MSB

// Indices when signalFromMaster == CONTROL_SIGNAL
#define DIRECTION0      2
#define DUTYCYCLE0_B1   3
#define DUTYCYCLE0_B2   4
#define DUTYCYCLE0_B3   5
#define DUTYCYCLE0_B4   6

#define DIRECTION1      7
#define DUTYCYCLE1_B1   8
#define DUTYCYCLE1_B2   9
#define DUTYCYCLE1_B3   10
#define DUTYCYCLE1_B4   11

// Indices when signalFromMaster == MODIFY_FORCES
#define FORCE_OFFSET0_B1    2
#define FORCE_OFFSET0_B2    3
#define FORCE_OFFSET0_B3    4
#define FORCE_OFFSET0_B4    5

#define FORCE_SLOPE0_B1     6
#define FORCE_SLOPE0_B2     7
#define FORCE_SLOPE0_B3     8
#define FORCE_SLOPE0_B4     9

#define FORCE_OFFSET1_B1    10
#define FORCE_OFFSET1_B2    11
#define FORCE_OFFSET1_B3    12
#define FORCE_OFFSET1_B4    13

#define FORCE_SLOPE1_B1     14
#define FORCE_SLOPE1_B2     15
#define FORCE_SLOPE1_B3     16
#define FORCE_SLOPE1_B4     17

// Indices when signalFromMaster == MODIFY_LIMITS
#define MAX_LIM_RAW0_B1  2
#define MAX_LIM_RAW0_B2  3
#define MAX_LIM_RAW0_B3  4
#define MAX_LIM_RAW0_B4  5

#define MIN_LIM_RAW0_B1  6
#define MIN_LIM_RAW0_B2  7
#define MIN_LIM_RAW0_B3  8
#define MIN_LIM_RAW0_B4  9

#define MAX_LIM_RAW1_B1  10
#define MAX_LIM_RAW1_B2  11
#define MAX_LIM_RAW1_B3  12
#define MAX_LIM_RAW1_B4  13

#define MIN_LIM_RAW1_B1  14
#define MIN_LIM_RAW1_B2  15
#define MIN_LIM_RAW1_B3  16
#define MIN_LIM_RAW1_B4  17


// for Initialization
#define NUMBER_OF_INITIALIZATION_FRAMES         6
#define NUMBER_OF_INIT_FRAMES_RECEIVED_INDEX    2
#define NUMBER_OF_TOTAL_INIT_FRAMES_INDEX       3

