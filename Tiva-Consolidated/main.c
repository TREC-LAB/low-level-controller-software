#include "main.h"

// EtherCAT buffer
extern PROCBUFFER_OUT MasterToTiva;
extern PROCBUFFER_IN TivaToMaster;

PandoraLowLevel pandora;

volatile bool runTimer1 = true;
volatile bool runTimer3 = true;

uint16_t sample_rate = 1000; // Hz
uint16_t logging_rate = 200; // Hz
uint16_t estop_rate = 1000;  // Hz


//*****************************************************************************
//
// define variables
//

volatile uint8_t uart;
volatile uint16_t duty_cycle = 0;
volatile uint8_t direction = 0;
volatile float last_dc = 0;

// Amplitudes for OL step inputs
volatile float s1 = 4;
volatile float s2 = 8;
volatile float s3 = 12;

volatile float OL_amplitude = 5;
volatile float OL_freq = 1;

//Actuator References used
const uint8_t actuator_0 = 0;
volatile float duty_cycle_0 = 0;
//DUTY_CYCLE duty_cycle0;
volatile uint8_t direction_0 = 0;

const uint8_t actuator_1 = 1;
volatile float duty_cycle_1 = 0;
//DUTY_CYCLE duty_cycle0;
volatile uint8_t direction_1 = 0;

volatile uint32_t lower_joint_limit_0 = 7000;
volatile uint32_t upper_joint_limit_0 = 30000;
volatile uint32_t lower_joint_limit_1 = 15000;
volatile uint32_t upper_joint_limit_1 = 34000;

bool UARTPutNonBlockingFlag = true;
bool log_data = false;
bool motor_state = false;

volatile float y_offset;
volatile uint32_t time_count; //in millisecond
volatile uint32_t time_prev;
uint32_t force_0; //Thigh & Ankle 0 actuator
uint32_t force_1; //Ankle 1 actuator
uint32_t adc_val0;
uint32_t adc_val1;
uint32_t adc_init;
volatile uint32_t adc_val4_filtered;

//data that the user typed down through the console
uint32_t readdata;
uint32_t motor_ctl_feedback;

volatile int32_t output;
//volatile int32_t qeiPosition_0;
//volatile uint32_t qeiVelocity_0;
//volatile int32_t qeiDirection_0;

//volatile int32_t qeiPosition_1;
//volatile uint32_t qeiVelocity_1;
//volatile int32_t qeiDirection_1;

//uint32_t abs_angle_0;
//uint32_t abs_angle_1;

volatile float u_sat = 25; //Input Saturation
volatile uint32_t des_force = 2200; //Desired Force for Closed-Loop control

volatile float error = 0;
volatile float error_total = 0;

//Reference model

float ap = 10;
float bp = 10;

float ar = 2;
float br = 2;

volatile float pitch_r;
volatile float dpitch_r = 0;

volatile float roll_r;
volatile float droll_r = 0;

//Duty Cycle Variable for Printing
volatile uint32_t duty_cycle_p = 0;
volatile uint32_t duty_cycle_p_1 = 0;

// UART Variables

char direction_str[10];//store user's input sin freq, later will be convert to float.
uint8_t direction_str_counter;//counter for storing user input into omega_str

char duty_cycle_str[5];//store user's input duty cycle, later will be convert to int.
uint8_t duty_cycle_str_counter;//counter for storing user input into duty_cycle_str

char sample_rate_str[5];//store user's input sample rate, later will be convert to int.
uint8_t sample_rate_str_counter;//counter for storing user input into sample_rate_freq_str

char lower_joint_limit_str[15];//store user's input lower joint limit, later will be convert to int.
char upper_joint_limit_str[15];//store user's input upper joint limit, later will be convert to int.
uint8_t joint_limit_str_counter;//counter for storing user input into lower/upper joint limit.

char logging_rate_str[5];//store user's input logging rate, later will be convert to int.
uint8_t logging_rate_str_counter;//counter for storing user input into logging rate.

char receivedStr[16];//char is a temporary, to be converted to int
uint8_t receivedStrCounter;
volatile float kneePitch;

volatile enum state_t op_state = testing; // all of UART was set to transition back to testing state rather than normal
volatile enum dof dof_state = none;

// Variables for converting integer encoder reading to joint angle

uint32_t enc0_offset = 34476; //Ankle Pitch
uint32_t enc1_offset = 27680; //Ankle Roll

volatile float q_0 = 0; //Ankle Pitch
volatile float q_1 = 0; //Ankle Roll

volatile float dq_0 = 0; //Ankle Pitch Angular Velocity
volatile float dq_1 = 0; //Ankle Roll Angular Velocity

volatile float pitch_des = 0;
volatile float roll_des = 0;

volatile float error_p;
volatile float error_r;

volatile float error_p_tot;
volatile float error_r_tot;

volatile float tau_p_in;
volatile float tau_r_in;

volatile float ctrl0_in = 0;
volatile float ctrl1_in = 0;

//Ankle pitch gains
float kp = 11; //9.5;
float ki = 2.5; //2;
float kd = 1;

//Ankle roll gains
float kp_r = 4;//2;
float ki_r = 2;//1;
float kd_r = 1;//0.5;//.75;

bool shut_down_signal = false;
uint32_t encVel = 0;
int32_t encDir = 0;

//*****************************************************************************

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void logData(void);
bool EngageVirtualEStop(PandoraLowLevel* pandora);

/*
 * main() is to initialize PWM, QEI, ADC, SSI, Timers, and UARTs modules.
 */
int main(void)
{

    //Set the system clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

//    SysCtlPeripheralEnable(LED_PERIPH);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlDelay(30);

    //Set the pin of your choice to output
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);

    SysCtlDelay(2000);

    // Populate pandora object
    pandora = pandoraConstruct();

    // Initialize tiva

    tivaInitEtherCAT();

    while(!pandora.initialized)
    {
        EtherCAT_MainTask();
        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = MasterToTiva.Byte[PROCESS_ID_INDEX];

        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            storeDataFromMaster(&pandora);
 //           processDataFromMaster(&pandora);
            loadDataForMaster(&pandora);
        }
    }

    tivaInit(&pandora);

    // Enable processor interrupts
    IntMasterEnable();

    startTimer1(estop_rate); // Start vstop timer
    startTimer3(sample_rate); // Start motor timer

    while(1)
    {
    }
}


/*
 * UART0 interrupt to handle communication between Tiva MCU and the computer
 */
void UART0IntHandler(void)
{
//    uint32_t ui32Status;
//
//    // Get the interrupt status.
//    ui32Status = UARTIntStatus(UART0_BASE, true);
//
//    // Clear the asserted interrupts.
//    UARTIntClear(UART0_BASE, ui32Status);
//
//    // Loop while there are characters in the receive FIFO.
//    while(UARTCharsAvail(UART0_BASE))
//    {
//        checkKeyboard();
//    }
}

/*
 * UART1 interrupt handler triggers if Tiva receives information from motor controller
 */
void UART1IntHandler(void)
{
/*    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ui32Status); */
}


/**
 * Logs through serial port
 */
//void logData(void)
//{
//    log_data = true;
//    if (log_data == true) {
//        char p[150];
//
//        // Logging all Connor board data
//        sprintf(&p[0],"f: %d,%d min: %d,%d max: %d,%d e: %d,%d pwm: %d,%d dir: %d,%d motors running: %d\n\r",pandora.actuator0.forceSensor.raw,pandora.actuator1.forceSensor.raw,pandora.joint0.lowerJointLimitRaw,pandora.joint1.lowerJointLimitRaw,pandora.joint0.upperJointLimitRaw,pandora.joint1.upperJointLimitRaw,pandora.joint0.encoder.raw,pandora.joint1.encoder.raw,(int)pandora.actuator0.dutyCycle,(int)pandora.actuator1.dutyCycle, pandora.actuator0.direction, pandora.actuator1.direction, runTimer3);
////        sprintf(&p[0],"newtons: %d,%d raw: %d,%d slope: %d,%d offset: %d,%d \n\r",pandora.joint0.forceSensor.newtons,pandora.joint1.forceSensor.newtons,pandora.joint0.forceSensor.raw,pandora.joint1.forceSensor.raw,pandora.joint0.forceSensor.slope,pandora.joint1.forceSensor.slope,pandora.joint0.encoder.raw,pandora.joint1.encoder.raw,pandora.joint0.forceSensor.slope,pandora.joint1.forceSensor.slope,pandora.joint0.forceSensor.offset,pandora.joint1.forceSensor.offset);
//
//        UARTSendString(0, p);
//    }
//}


/**
 * Timer1A interrupt handler
 * Checks for software estop conditions.
 * If any are met, turns off motors and sends shutdown signal to master.
 */
void Timer1AIntHandler(void)
{
    if (runTimer1)
    {
        if (EngageVirtualEStop(&pandora))
        {
            // Stop timer1
            runTimer3 = false;
            pandora.signalToMaster = HALT_SIGNAL_TM;

            // Stop motor
            pandora.actuator0.dutyCycle = 0;
            pandora.actuator1.dutyCycle = 0;
            SendPWMSignal(&pandora.actuator0);
            SendPWMSignal(&pandora.actuator1);

            // Send shutdown signal to master
            EtherCAT_MainTask();
        }
        else
        {
            pandora.signalToMaster = NORMAL_OPERATION;
        }
    }
    else
    {
        // Stop motors if not running estop interrupt
        pandora.actuator0.dutyCycle = 0;
        pandora.actuator1.dutyCycle = 0;
        SendPWMSignal(&pandora.actuator0);
        SendPWMSignal(&pandora.actuator1);
    }
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


/*
 * EngageVirtualEStop
 * Checks conditions to decide whether to engage Virtual EStop
 * based on Force, Abs Encoder Angle Ranges.
 */
bool EngageVirtualEStop(PandoraLowLevel* pandora) {
/*    bool retVal = false;
    printf("SIGNAL: %d\n", pandora->signalFromMaster);
    printf("joint0 raw: %d\n", pandora->joint0.encoder.raw);
    printf("joint0 actualRaw: %d\n", pandora->joint0.actualRaw);
    printf("joint0 zero: %d\n", pandora->joint0.rawZero);
    printf("joint0 actualRad: %f\n", pandora->joint0.angleRads);
    printf("joint1 raw: %d\n", pandora->joint1.encoder.raw);
    printf("joint1 actualRaw: %d\n", pandora->joint1.actualRaw);
    printf("joint1 zero: %d\n", pandora->joint1.rawZero);
    printf("joint1 actualRad: %f\n", pandora->joint1.angleRads);
    printf("joint1 -1 * backward range of motion: %d\n", -1 * pandora->joint1.rawBackwardRangeOfMotion);
    if(pandora->joint0.actualRaw < -1 * pandora->joint0.rawBackwardRangeOfMotion && pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        retVal = true;
        printf("SIGNAL: %d\n", pandora->signalFromMaster);
        printf("ESTOP TRIGGERED: joint0 actualRaw < -1 * joint0 backward range of motion\n");
        printf("joint0 raw: %d\n", pandora->joint0.encoder.raw);
        printf("joint0 actualRaw: %d\n", pandora->joint0.actualRaw);
        printf("joint0 zero: %d\n", pandora->joint0.rawZero);
        printf("joint0 actualRad: %f\n", pandora->joint0.angleRads);
        printf("joint0 -1 * backward range of motion: %d\n", -1 * pandora->joint0.rawBackwardRangeOfMotion);
    }
    if(pandora->joint0.actualRaw > pandora->joint0.rawForwardRangeOfMotion && pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        retVal = true;
        printf("SIGNAL: %d\n", pandora->signalFromMaster);
        printf("ESTOP TRIGGERED: joint0 actualRaw > joint0 forward range of motion\n");
        printf("joint0 raw: %d\n", pandora->joint0.encoder.raw);
        printf("joint0 actualRaw: %d\n", pandora->joint0.actualRaw);
        printf("joint0 zero: %d\n", pandora->joint0.rawZero);
        printf("joint0 actualRad: %f\n", pandora->joint0.angleRads);
        printf("joint0 forwardRangeOfMotion: %d\n", pandora->joint0.rawForwardRangeOfMotion);
    }
    if(pandora->joint1.actualRaw < -1 * pandora->joint1.rawBackwardRangeOfMotion && pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        retVal = true;
        printf("SIGNAL: %d\n", pandora->signalFromMaster);
        printf("ESTOP TRIGGERED: joint1 actualRaw < -1 * joint1 backward range of motion\n");
        printf("joint1 raw: %d\n", pandora->joint1.encoder.raw);
        printf("joint1 actualRaw: %d\n", pandora->joint1.actualRaw);
        printf("joint1 zero: %d\n", pandora->joint1.rawZero);
        printf("joint1 actualRad: %f\n", pandora->joint1.angleRads);
        printf("joint1 -1 * backward range of motion: %d\n", -1 * pandora->joint1.rawBackwardRangeOfMotion);
    }
    if(pandora->joint1.actualRaw > pandora->joint1.rawForwardRangeOfMotion && pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        retVal = true;
        printf("SIGNAL: %d\n", pandora->signalFromMaster);
        printf("ESTOP TRIGGERED: joint1 actualRaw > joint1 forward range of motion\n");
        printf("joint1 raw: %d\n", pandora->joint1.encoder.raw);
        printf("joint1 actualRaw: %d\n", pandora->joint1.actualRaw);
        printf("joint1 zero: %d\n", pandora->joint1.rawZero);
        printf("joint1 actualRad: %f\n", pandora->joint1.angleRads);
        printf("joint1 forwardRangeOfMotion: %d\n", pandora->joint1.rawForwardRangeOfMotion);
    }
    return retVal;*/

   return (((pandora->joint0.actualRaw < -1 * pandora->joint0.rawBackwardRangeOfMotion ||
        pandora->joint0.actualRaw > pandora->joint0.rawForwardRangeOfMotion)) ||
            (pandora->joint1.actualRaw < -1 * pandora->joint1.rawBackwardRangeOfMotion ||
        pandora->joint1.actualRaw > pandora->joint1.rawForwardRangeOfMotion) ||
        pandora->actuator0.forceSensor.newtons > pandora->actuator0.forceSensor.upperLimitNewtons ||
        pandora->actuator0.forceSensor.newtons < pandora->actuator0.forceSensor.lowerLimitNewtons ||
        pandora->actuator1.forceSensor.newtons > pandora->actuator1.forceSensor.upperLimitNewtons ||
        pandora->actuator1.forceSensor.newtons < pandora->actuator1.forceSensor.lowerLimitNewtons) &&
        pandora->signalFromMaster == CONTROL_SIGNAL;
}

/*
 * Timer2A interrupt handler
 * Used for data logging to serial port
 */
void Timer2AIntHandler(void) {}

/*
 * Timer3A interrupt handler
 * Sends TivaToMaster frame to master
 * Receives MasterToTiva frame from master
 */
void Timer3AIntHandler(void)
{
    if (pandora.signalFromMaster == CONTROL_SIGNAL)
    {
        updateForces(&pandora.actuator0.forceSensor);
        updateForces(&pandora.actuator1.forceSensor);
        updateJointAngles(&pandora.joint0);
        updateJointAngles(&pandora.joint1);
        readQEIEncoderPosition(&pandora.actuator0.motorEncoder);
        readQEIEncoderPosition(&pandora.actuator1.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator0.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator1.motorEncoder);
    }

    if (runTimer3)
    {
        // Send TivaToMaster and receive MasterToTiva
        loadDataForMaster(&pandora);
        EtherCAT_MainTask();

        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = MasterToTiva.Byte[PROCESS_ID_INDEX];

        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            // Read desired Tiva status, duty cycles, and directions from MasterToTiva
            storeDataFromMaster(&pandora);

            // Act according Tiva status
            // Turns off estop timer if necessary
            runTimer1 = processDataFromMaster(&pandora);
             // Populate TivaToMaster data frame
        //    loadDataForMaster(&pandora);
        }
        else
        {
            runTimer1 = processDataFromMaster(&pandora);
     //       loadDataForMaster(&pandora);
        }
    }
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}

