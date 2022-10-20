#include "UART_TIVA.h"

/**
 * Initializes UART0 for communicating between Tiva and Master
 */
/*
void UART0Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //also for uart0 and SSI
    // Set PA0 and PA1 as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 1,152,000, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 1152000,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}
*/
/**
 * Initializes UART0 for communicating between Tiva and motor controller
 */
/*
void UART1Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set PB0 and PB1 as UART pins.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}*/

/**
 * Sends instruction to users to the console
 */
/*
void sendInstruction()
{
    UARTSendString(0, "TIVA initialized, begin logging: \r\n");
}
*/
/*
 * Send a string to the UART.
 * uartMod is used to indicate which UART module you want to use
 * UART0 is used between MCU and the Computer console
 * UART1 is used between MCU and the motor controller
 */
/*
void UARTSendString(uint8_t uartMod, char pui8Buffer[])
{
    //
    // Loop while there are more characters to send.
    //
    uint8_t i;
    for (i = 0; i < strlen(pui8Buffer); i++)
    {
        //
        // Write the next character to the UART.
        //
        if (uartMod == 0)
        {
            //non blocking function were causing problem
//            UARTCharPutNonBlocking(UART0_BASE, pui8Buffer[i]);//uart0 module
            UARTCharPut(UART0_BASE, pui8Buffer[i]);        //uart0 module
        }
        else if (uartMod == 1)
        { //Unnecessary for Thor Actuator applications
            //UARTPutNonBlockingFlag will only become 0 if there is no space available in transmit FIFO
            //UARTPutNonBlockingFlag = UARTCharPutNonBlocking(UART1_BASE, pui8Buffer[i]);//uart1 module
            UARTCharPut(UART1_BASE, pui8Buffer[i]);
        }
    }
}
*/
/*
void checkKeyboard()
{
//    // Read the next character from the UART and write it back to the UART.
//    readdata = UARTCharGetNonBlocking(UART0_BASE);
//    // op_state = testing; //reinitialize op_state
//
//    //change joint limit
//    if (readdata == 'j')
//    {
//        //reset lower and upper limit
//        memset(lower_joint_limit_str, 0, 15);
//        memset(upper_joint_limit_str, 0, 15);
//        op_state = changing_lower_joint_limit;
//        UARTSendString(0, "lo_jt_limit= ");
//    }
    //start motor
//    else if (readdata == 'z')
//    {
//        motor_state = true;
//        time_count = 0;
//
//        //Reset all velocity measurements back to zero (assumes robot starts at rest)
//        abs_angle_0_vel_prev = 0;
//        abs_angle_1_vel_prev = 0;
//        abs_angle_0_vel = 0;
//        abs_angle_1_vel = 0;
//        abs_angle_0_vel_f = 0;
//        abs_angle_1_vel_f = 0;
//        //Set previous angle values to the starting values (not zero)
//        readAbsEnc_0();
//        readAbsEnc_1();
//        abs_angle_0_prev = abs_angle_0;
//        abs_angle_1_prev = abs_angle_1;
//        error_p_tot = 0;
//        error_r_tot = 0;
//        //Have reference model start from current position
//        pitch_r = ((float)abs_angle_0 - (float)enc0_offset)*2.746582e-3; //Ankle Pitch
//        roll_r = ((float)abs_angle_1 - (float)enc1_offset)*-2.746582e-3; //Ankle Roll
//
//
//        TimerEnable(TIMER3_BASE, TIMER_A);
//        TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / sample_rate); //Set the sampling rate
//        //Find initial value for ADC filter state right when the timer starts
////        readLoadCellDiff0();
//        readLoadCell_0();
//        adc_init = adc_val0;
//        //Reset Accumulated Error
//        error_total = 0;
//
//        UARTSendString(0, "Motors enabled!\r\n");
//    }
//    //stop all motors
//    else if (readdata == 'x')
//    {
//        duty_cycle_0 = 0;
//        setPulseWidth(actuator_0, 20000, duty_cycle_0, SysCtlClockGet(),
//                      direction_0);
//        duty_cycle_1 = 0;
//        setPulseWidth(actuator_1, 20000, duty_cycle_1, SysCtlClockGet(),
//                      direction_1);
//        TimerDisable(TIMER3_BASE, TIMER_A);
//        UARTSendString(0, "Motors stopped!\r\n");
//    }
//    // Start estop interrupt
//    else if (readdata == 's')
//    {
//        TimerEnable(TIMER1_BASE, TIMER_A);
//        TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / estop_rate);
//        UARTSendString(0, "Starting estop timer!\r\n");
//    }
//    // Stop estop interrupt
//    else if (readdata == 'm')
//    {
//        TimerDisable(TIMER1_BASE, TIMER_A);
//        UARTSendString(0, "Stopping estop timer!\r\n");
//    }
//    // Change motor direction
//    else if (readdata == 'f')
//    {
//        //reset direction string
//        //memset(direction_str, 0, 10);
//        //op_state = changing_direction;
//        //        UARTSendString(0, "\n\r direction= ");
//        UARTSendString(0, "\n\r Direction changed \n\r");
//        switch (uart)
//        {
//        case 0:
//            if (direction_0 == 0){
//                direction_0 = 1;
//            }
//            else {
//                direction_0 = 0;
//            }
//
//            break;
//        case 1:
//            if (direction_1 == 0) {
//                direction_1 = 1;
//            }
//            else {
//                direction_1 = 0;
//            }
//            break;
//        }
//    }
//    //change actuator 0 duty cycle
//    else if (readdata == 'q')
//    {
//        uart = actuator_0;
//        memset(duty_cycle_str, 0, 5);
//        op_state = changing_duty_cycle;
//        UARTSendString(0, "\n\r Actuator 0 duty cycle= ");
//    }
//    //change actuator 1 duty cycle
//    else if (readdata == 'w')
//    {
//        uart = actuator_1;
//        memset(duty_cycle_str, 0, 5);
//        op_state = changing_duty_cycle;
//         UARTSendString(0, "\n\r Actuator 1 duty cycle= ");
//    }
//    //logging enabled
//    else if (readdata == 'l')
//    {
//        log_data = true;
//        //UARTSendString(0, "Data logging enabled\n\r");
//        TimerEnable(TIMER2_BASE, TIMER_A);
//        TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() / logging_rate);
//        UARTSendString(0, "\n\r Logging enabled!");
//    }
//    //logging disabled
//    else if (readdata == 'k')
//    {
//        TimerDisable(TIMER2_BASE, TIMER_A);
//        log_data = false;
//        UARTSendString(0, "\n\r Logging disabled!");
//    }

//    //numbers from user for changing sinusoidal frequency
//    else if (readdata >= '0' && readdata <= '9' || readdata == '.')
//    { //if the user is trying to write a decimal number
//        //if it's in the change sin frequency state
//        switch (op_state)
//        {
//        case testing:
//            receivedStr[receivedStrCounter] = readdata;
//            receivedStrCounter++;
//            //UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_direction:
//            direction_str[direction_str_counter] = readdata;
//            direction_str_counter++;
//            UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_duty_cycle:
//            duty_cycle_str[duty_cycle_str_counter] = readdata;
//            duty_cycle_str_counter++;
//            UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_sample_rate:
//            sample_rate_str[sample_rate_str_counter] = readdata;
//            sample_rate_str_counter++;
//            //           UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_lower_joint_limit:
//            lower_joint_limit_str[joint_limit_str_counter] = readdata;
//            joint_limit_str_counter++;
//            //            UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_upper_joint_limit:
//            upper_joint_limit_str[joint_limit_str_counter] = readdata;
//            joint_limit_str_counter++;
//            //            UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case changing_logging_rate:
//            logging_rate_str[logging_rate_str_counter] = readdata;
//            logging_rate_str_counter++;
//            //            UARTCharPutNonBlocking(UART0_BASE,readdata);
//            break;
//        case normal:
//            break;
//        }
//    }

//    // Convert user's input string to number
//    else if (readdata == 13 || readdata == 0xD) // Press enter
//    {         //enter key
//        switch (op_state)
//        {
//        case testing:
//            //need to know how the type (int, float); most likely atof();
//            receivedStrCounter = 0;
//            op_state = testing;
//            kneePitch = atoi(receivedStr);
//            kneePitch /= 100;
//            kneePitch -= 100;
//
//            memset(receivedStr, 0, 16); //reset string
//            break;
//        case changing_direction:
//            direction_str_counter = 0;
//            op_state = testing;
//            switch (uart)
//            {
//            case 0:
//                direction_0 = atoi(direction_str);
//                break;
//            case 1:
//                direction_1 = atoi(direction_str);
//                break;
//            }
//            //          UARTSendString(0, "\r\n");
//            break;
//        case changing_duty_cycle:
//            printf("uart: %d\n", uart);
//            duty_cycle_str_counter = 0;
//            op_state = testing;
//            //switching statement referring to different actuators
//            switch (uart)
//            {
//            case 0:
//                duty_cycle_0 = atof(duty_cycle_str);
////                printf("duty_cycle_0:   %e\r\n", duty_cycle_0); // Sanity check prints
//                UARTSendString(0, "\r\nDuty cycle 0 changed\r\n");
//                break;
//            case 1:
//                duty_cycle_1 = atof(duty_cycle_str);
////                printf("duty_cycle_1:   %e\r\n", duty_cycle_1); // Sanity check prints
//                UARTSendString(0, "\r\nDuty cycle 1 changed\r\n");
//                break;
//            }
//            sendSignal();
//            //           UARTSendString(0, "\r\n");
//            break;
//        case changing_sample_rate:
//            sample_rate_str_counter = 0;
//            op_state = testing;
//            sample_rate = atoi(sample_rate_str);
//
//            time_count = 0;
//            TimerEnable(TIMER1_BASE, TIMER_A);
//            TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / sample_rate); //Set the sampling rate
//            //            UARTSendString(0, "\r\n");
//            break;
//
//        case changing_upper_joint_limit:
//            joint_limit_str_counter = 0;
//            upper_joint_limit_0 = atoi(upper_joint_limit_str);
//            upper_joint_limit_1 = upper_joint_limit_0;
//            op_state = testing;
//            //            UARTSendString(0, "\r\n");
//            break;
//
//        case changing_lower_joint_limit:
//            joint_limit_str_counter = 0;
//            lower_joint_limit_0 = atoi(lower_joint_limit_str);
//            lower_joint_limit_1 = lower_joint_limit_0;
//            //move on to the changing upper joint limit state;
//            UARTSendString(0, "\r\nup_jt_limit= ");
//            op_state = changing_upper_joint_limit;
//            break;
//
//        case changing_logging_rate:
//            logging_rate_str_counter = 0;
//            op_state = testing;
//            logging_rate = atoi(logging_rate_str);
//            //            UARTSendString(0, "\r\n");
//            break;
//
//        case normal:
//            //            UARTSendString(0, "\r\n");
//            break;
//        }
////            UARTSendString(0, "\r\n");
////        printf("done\n"); // Sanity check prints
//        sendSignal();
//    }
//
//    else
//    {  //echo back
//        UARTCharPutNonBlocking(UART0_BASE,readdata);
//    }
}
*/
