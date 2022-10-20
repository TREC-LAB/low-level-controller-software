/**
 * PandoraLowLevel.c
 * @author: Nick Tremaroli
 * Defines all of the low-level features and functions
 * of pandora
 */
#include "PandoraLowLevel.h"

/**
 * pandoraConstruct
 *
 * Constructs the pandoraLowLevel structure and sets
 * all of the starting values to 0. Initialization does
 * not happen at this stage
 *
 * @return: a basic pandoraLowLevel structure
 */
PandoraLowLevel pandoraConstruct()
{
    // create the pandora structure
    PandoraLowLevel pandora;

    // initialize the signals
    pandora.signalToMaster = 0;
    pandora.signalFromMaster = 0;
    pandora.prevSignalFromMaster = 0;

    // initialize the process IDs
    pandora.prevProcessIdFromMaster = 0;
    pandora.processIdFromMaster = 0;

    // initialize the IMU and FT sensor
    // TODO: move this to TivaInit
    pandora.imu = imuConstruct();
    pandora.ftSensor = ftSensorConstruct();

    // set pandora to un-initialized
    pandora.initialized = false;

    // set the number of frames received to 0
    pandora.numberOfInitFramesReceived = 0;

    // return the pandora structure
    return pandora;
}

/**
 * tivaInitEtherCAT
 *
 * Initializes the ethercat board on the Tiva. Note that this function
 * is called before the other peripheral based initialization functions.
 * This is because the Tiva must read the initialization input data
 * from the master first to know how to configure the rest of its peripherals
 */
void tivaInitEtherCAT(PandoraLowLevel* pandora)
{
    // Configure SSI3 for SPI for use with EtherCAT
    SSI3_Config_SPI();

    // Run the EtherCAT_Init function
    int ret = EasyCAT_Init(&pandora->easyCAT.etherCATOutputFrames);
}

/**
 * ProcessCurrentInitFrame
 *
 * Processes the current initialization frame and applies it to
 * the PandoraLowLevel structure
 * @param pandora: a pointer to the pandora structure from which
 * the initialization frame will be stored to
 */
void StoreCurrentInitFrame(PandoraLowLevel* pandora)
{
    // get the current initialization frame
    uint8_t currentInitFrame = pandora->easyCAT.etherCATInputFrames.initSignalHeader.currentInitFrame;

    // actuator 0 is the first initialization frame sent
    if(currentInitFrame == 0)
    {
        uint8_t actuatorNumber = 0;

        // get the raw QEI Base number (0, 1)
        uint8_t rawQEIBase = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_QEIBaseNumber;

        // convert it to the actual QEI Base
        uint32_t actuator0_QEIBase = QEI0_BASE + ((1 << 12) * rawQEIBase);

        // get the QEI sample rate
        uint16_t actuator0_QEISampleRate = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_QEISampleRate;

        // get the QEI counts per rotation
        uint32_t actuator0_QEICountsPerRotation = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_QEICountsPerRotation;

        // get the raw ADC Base number (0, 1)
        uint8_t rawADCBase = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorADCBaseNumber;

        // convert it to the actual ADC Base
        uint32_t actuator0_ADCBase = ADC0_BASE + ((1 << 12) * rawADCBase);

        // get the force sensor slope
        float actuator0_ForceSensorSlope = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorSlope;

        // get the force sensor offset
        float actuator0_ForceSensorOffset = pandora->easyCAT.etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorOffset;

        // construct the pandora actuator on the 0th side
        Actuator actuator0 = actuatorConstruct(actuatorNumber, actuator0_QEIBase,
                                                actuator0_QEISampleRate, actuator0_QEICountsPerRotation,
                                                actuator0_ADCBase, actuator0_ForceSensorSlope,
                                                actuator0_ForceSensorOffset);
        // store it to the pandora structure
        pandora->actuator0 = actuator0;

    }

    // actuator 1 is the second initialization frame send
    else if(currentInitFrame == 1)
    {
        uint8_t actuatorNumber = 1;

        // get the raw QEI Base number (0, 1)
        uint8_t rawQEIBase = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_QEIBaseNumber;

        // convert it to the actual QEI Base
        uint32_t actuator1_QEIBase = QEI0_BASE + ((1 << 12) * rawQEIBase);

        // get the QEI sample rate
        uint16_t actuator1_QEISampleRate = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_QEISampleRate;

        // get the QEI counts per rotation
        uint32_t actuator1_QEICountsPerRotation = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_QEICountsPerRotation;

        // get the raw ADC Base number (0, 1)
        uint8_t rawADCBase = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorADCBaseNumber;

        // convert it to the actual ADC Base
        uint32_t actuator1_ADCBase = ADC0_BASE + ((1 << 12) * rawADCBase);

        // get the force sensor slope
        float actuator1_ForceSensorSlope = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorSlope;

        // get the force sensor offset
        float actuator1_ForceSensorOffset = pandora->easyCAT.etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorOffset;

        // construct the pandora actuator on the 1th side
        Actuator actuator1 = actuatorConstruct(actuatorNumber, actuator1_QEIBase,
                                                actuator1_QEISampleRate, actuator1_QEICountsPerRotation,
                                                actuator1_ADCBase, actuator1_ForceSensorSlope,
                                                actuator1_ForceSensorOffset);

        // store it to the pandora structure
        pandora->actuator1 = actuator1;
    }

    // joint 0 is the third initialization frame send
    else if(currentInitFrame == 2)
    {
        // get the raw SSI Base number (0, 1, etc.)
        uint8_t rawSSIBaseNumber = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_SSIBaseNumber;

        // convert it to the actual SSI Base
        uint32_t joint0_SSIBaseNumber = SSI0_BASE + ((1 << 12) * rawSSIBaseNumber);

        // get the SSI Brand
        uint8_t ssiEncoderBrandRaw = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_SSIEncoderBrandRaw;
        SSIEncoderBrand joint0_SSIEncoderBrand = (SSIEncoderBrand)ssiEncoderBrandRaw;

        // get the SSI sample rate
        uint16_t joint0_SSIEncoderSampleRate = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_SSISampleRate;

        // get the joint reverse factor
        int8_t joint0_jointReverseFactor = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_ReverseFactor;

        // get the actual zero
        float rawZero = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_RawZeroPosition;

        // get the actual forward range of motion
        float rawForwardRangeOfMotion = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_RawForwardRangeOfMotion;

        // get the actual backward range of motion
        float rawBackwardRangeOfMotion = pandora->easyCAT.etherCATInputFrames.initSignal2Frame.joint0_RawBackwardRangeOfMotion;

        // convert the actual values to raw based on the encoder brand
        uint16_t joint0_rawZero, joint0_rawForwardRangeOfMotion, joint0_rawBackwardRangeOfMotion;
        if(joint0_SSIEncoderBrand == Gurley_Encoder)
        {
            joint0_rawZero = (uint16_t)(rawZero * (65535.0 / 180.0));
            joint0_rawForwardRangeOfMotion = (uint16_t)(rawForwardRangeOfMotion * (65535.0 / 180.0));
            joint0_rawBackwardRangeOfMotion = (uint16_t)(rawBackwardRangeOfMotion * (65535.0 / 180.0));
        }
        else if(joint0_SSIEncoderBrand == Orbis_Encoder)
        {
            joint0_rawZero = (uint16_t)(rawZero * (16383.0 / 360.0));
            joint0_rawForwardRangeOfMotion = (uint16_t)(rawForwardRangeOfMotion * (16383.0 / 360.0));
            joint0_rawBackwardRangeOfMotion = (uint16_t)(rawBackwardRangeOfMotion * (16383.0 / 360.0));
        }

        // construct the pandora joint on the 0th side
        Joint joint0 = jointConstruct(joint0_SSIBaseNumber, joint0_SSIEncoderBrand, joint0_SSIEncoderSampleRate,
                                      joint0_jointReverseFactor, joint0_rawZero,
                                      joint0_rawForwardRangeOfMotion, joint0_rawBackwardRangeOfMotion);

        // store it to the pandora structure
        pandora->joint0 = joint0;
    }

    // joint 1 is the fourth initialization frame sent
    else if(currentInitFrame == 3)
    {
        // get the raw SSI Base number (0, 1, etc.)
        uint8_t rawSSIBaseNumber = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_SSIBaseNumber;

        // convert it to the actual SSI Base
        uint32_t joint1_SSIBaseNumber = SSI0_BASE + ((1 << 12) * rawSSIBaseNumber);

        // get the SSI Brand
        uint8_t ssiEncoderBrandRaw = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_SSIEncoderBrandRaw;
        SSIEncoderBrand joint1_SSIEncoderBrand = (SSIEncoderBrand)ssiEncoderBrandRaw;

        // get the SSI sample rate
        uint16_t joint1_SSIEncoderSampleRate = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_SSISampleRate;

        // get the joint reverse factor
        int8_t joint1_jointReverseFactor = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_ReverseFactor;

        // get the actual zero
        float rawZero = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_RawZeroPosition;

        // get the actual forward range of motion
        float rawForwardRangeOfMotion = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_RawForwardRangeOfMotion;

        // get the actual backward range of motion
        float rawBackwardRangeOfMotion = pandora->easyCAT.etherCATInputFrames.initSignal3Frame.joint1_RamBackwardRangeOfMotion;

        // convert the actual values to raw based on the encoder brand
        uint16_t joint1_rawZero, joint1_rawForwardRangeOfMotion, joint1_rawBackwardRangeOfMotion;
        if(joint1_SSIEncoderBrand == Gurley_Encoder)
        {
            joint1_rawZero = (uint16_t)(rawZero * (65535.0 / 180.0));
            joint1_rawForwardRangeOfMotion = (uint16_t)(rawForwardRangeOfMotion * (65535.0 / 180.0));
            joint1_rawBackwardRangeOfMotion = (uint16_t)(rawBackwardRangeOfMotion * (65535.0 / 180.0));
        }
        else if(joint1_SSIEncoderBrand == Orbis_Encoder)
        {
            joint1_rawZero = (uint16_t)(rawZero * (16383.0 / 360.0));
            joint1_rawForwardRangeOfMotion = (uint16_t)(rawForwardRangeOfMotion * (16383.0 / 360.0));
            joint1_rawBackwardRangeOfMotion = (uint16_t)(rawBackwardRangeOfMotion * (16383.0 / 360.0));
        }

        // construct the pandora joint on the 1th side
        Joint joint1 = jointConstruct(joint1_SSIBaseNumber, joint1_SSIEncoderBrand, joint1_SSIEncoderSampleRate,
                                      joint1_jointReverseFactor, joint1_rawZero,
                                      joint1_rawForwardRangeOfMotion, joint1_rawBackwardRangeOfMotion);

        // store it to the pandora structure
        pandora->joint1 = joint1;
    }

    // the IMU enable feature is sent in the fifth initialization frame
    else if(currentInitFrame == 4)
    {
        // get the imu enable feature
        uint8_t imuEnable = pandora->easyCAT.etherCATInputFrames.initSignal4Frame.imuEnable;

        // store it to the pandora structure
        pandora->imu.enabled = imuEnable;
    }

    // the settings and estop enable feature is sent in the
    // sixth initialization frame
    else if(currentInitFrame == 5)
    {
        // get the estop enable option
        uint8_t softwareEStopEnable = pandora->easyCAT.etherCATInputFrames.initSignal5Frame.softwareEStopEnable;

        // create a pandora settings structure and store the value to it
        PandoraLowLevelSettings settings;
        settings.softwareEStopEnable = softwareEStopEnable;

        // store the settings in the pandora structure
        pandora->settings = settings;
    }
}

/**
 * tivaInit
 *
 * The initialization function which initializes
 * all of the necessary Tiva peripherals
 *
 * @param pandora: a pointer to the pandora structure
 */
void tivaInit(PandoraLowLevel* pandora)
{
    // Enable the PWM
    PWMConfig();

    // Enable the force sensors
    enableForceSensor(&pandora->actuator0.forceSensor);
    enableForceSensor(&pandora->actuator1.forceSensor);

    // Enable the SSI Encoders
    enableSSIEncoder(&pandora->joint0.encoder);
    enableSSIEncoder(&pandora->joint1.encoder);

    // enable the QEI Encoders
    enableQEIEncoder(&pandora->actuator0.motorEncoder);
    enableQEIEncoder(&pandora->actuator1.motorEncoder);

    // enable the debug LEDs
    enableDebugLEDS();

    // if the IMU is set to be enabled
    // enable it
    if(pandora->imu.enabled)
        imuEnable(&pandora->imu);

    // enable the FT sensor
    ftSensorEnable(&pandora->ftSensor);

    // enable the timers
    timer1A_Config();
    timer2A_Config();
    timer3A_Config();
}

/**
 * enableDebugLEDS
 *
 * Configures the LEDs which are used for debugging
 * by emitting different colors of lights.
 *
 */
void enableDebugLEDS()
{
    // Enable the LED peripheral on the Tiva
    SysCtlPeripheralEnable(LED_PERIPH);

    // configure the GPIO to be used as LEDs
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED | GREEN_LED);

    // initially set the LEDs to be off
    GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, 0);
}

/**
 * disableDebugLEDs
 *
 * Disables the LEDs used for debugging.
 */
void disableDebugLEDs()
{
    // set all the LEDs to be off
    GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, 0);

    // disable the LED peripheral on the Tiva
    SysCtlPeripheralDisable(LED_PERIPH);
}

/**
 * notConnectedLEDS
 *
 * Configures the LEDS to act a certain
 * way when the Tiva is not connected to the master.
 */
void notConnectedLEDS()
{
    // initialize the static variables for
    // the blinking ability
    static bool led_on = false;
    static uint32_t led_start = 0;
    static uint32_t led_current = 0;

    // Turn on Red after a certain time
    if (!led_on && (abs(led_current - led_start) > 100))
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, RED_LED);
        led_on = true;
        led_start = 0;
        led_current = 0;
    }

    // Turn off Red after a certain time
    else if (led_on && (abs(led_current - led_start) > 50))
    {
        GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, 0);
        led_on = false;
        led_start = 0;
        led_current = 0;
    }
    led_current++;
}

/**
 * idleLEDS
 *
 * Configures the LEDS to act a certain
 * way when the master is sending an idle
 * signal to the Tiva.
 */
void idleLEDS()
{
    // solid purple
    GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, BLUE_LED);
    GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, RED_LED);
}

/**
 * haltLEDS
 *
 * configures the LEDS to act a certain
 * way when the master sends a halt signal
 * to the Tiva.
 */
void haltLEDS()
{
    // solid red
    GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, RED_LED);
    GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
}

void controlLEDS()
{
    // solid blue
    GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, BLUE_LED);
    GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
}

/**
 * checkActuatorDisable
 *
 * Checks if the actuators should be disabled.
 *
 * @param pandora: a pointer to the pandora Tiva struct
 */
void checkActuatorDisable(PandoraLowLevel* pandora)
{
    // DO NOT EDIT THIS CODE (please)
    if(pandora->signalFromMaster != CONTROL_SIGNAL)
    {
        // STOP MOTORS
        pandora->actuator0.pwmGenerator.dutyCycle = 0;
        pandora->actuator1.pwmGenerator.dutyCycle = 0;
        SendPWMSignal(&pandora->actuator0);
        SendPWMSignal(&pandora->actuator1);
    }
}

/**
 * storeDataFromMaster
 *
 * Deserializes the data received from the master
 * and stores the data accordingly.
 *
 * @param pandora: a pointer to the pandora structure,
 * all of the data from the master gets stored in this
 * structure
 */
void storeDataFromMaster(PandoraLowLevel* pandora)
{
    // get the signal from the master computer
    pandora->signalFromMaster = pandora->easyCAT.etherCATInputFrames.rawBytes[SIGNAL_INDEX];

    // if a control signal is received
    if (pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Store joint 0 direction
        pandora->actuator0.pwmGenerator.direction = pandora->easyCAT.etherCATInputFrames.controlSignalFrame.actuator0Direction;

        // Store joint 0 duty cycle
        pandora->actuator0.pwmGenerator.dutyCycle = pandora->easyCAT.etherCATInputFrames.controlSignalFrame.actuator0DutyCycle;

        // Store joint 1 direction
        pandora->actuator1.pwmGenerator.direction = pandora->easyCAT.etherCATInputFrames.controlSignalFrame.actuator1Direction;

        // Store joint 1 duty cycle
        pandora->actuator1.pwmGenerator.dutyCycle = pandora->easyCAT.etherCATInputFrames.controlSignalFrame.actuator1DutyCycle;
    }

    // if an initialization signal is received
    else if (pandora->signalFromMaster == INITIALIZATION_SIGNAL)
        // increment how many initialization frames have been received
        pandora->numberOfInitFramesReceived = pandora->easyCAT.etherCATInputFrames.initSignalHeader.currentInitFrame + 1;
}

/**
 * processDataFromMaster
 *
 * Processes the data received form the master
 * Acts on received master data according to desired TIVA mode
 *
 * @param pandora: a pointer to the pandora structure
 * @return: 1 if estop timer should be run, otherwise return 0.
 */
bool processDataFromMaster(PandoraLowLevel* pandora)
{
    /***********FOR INITIALIZATION***********/
    // if an initialization signal is received
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        // if pandora is already initialized, setup for
        // re-initialization
        if (pandora->initialized)
            pandora->initialized = false;

        // store the current initialization frame which was sent
        StoreCurrentInitFrame(pandora);

        // if all of the initialization frames were sent
        if(pandora->numberOfInitFramesReceived == NUMBER_OF_INITIALIZATION_FRAMES)
        {
            // set pandora to be initialized
            pandora->initialized = true;
            return true;
        }

        // update the previous signal from master
        pandora->prevSignalFromMaster = pandora->signalFromMaster;
        return false;
    }

    // reset the number of initialization frames received if
    // an initialization signal is not being sent and pandora is
    // already initialized
    else if(pandora->signalFromMaster != INITIALIZATION_SIGNAL && pandora->initialized)
        pandora->numberOfInitFramesReceived = 0;

    // if pandora is not initialized
    if(!pandora->initialized)
    {
        // update the previous signal from master
        pandora->prevSignalFromMaster = pandora->signalFromMaster;
        return false;
    }
    /***********FOR ALL OTHER SIGNALS***********/


    int run_estop = false;

    // DO NOT REMOVE THIS LINE
    // Ensures motor PWMs are set to 0 in the case signalFromMaster != CONTROL_SIGNAL but motors are still moving
    checkActuatorDisable(pandora);

    // if the master computer is not connected
    if (pandora->signalFromMaster == NOT_CONNECTED)
        // the LEDs will flash Red
        notConnectedLEDS();

    // if the master computer or the Tiva sends a halt signal
    // TODO: add feature to stop the motors
    else if (pandora->signalFromMaster == HALT_SIGNAL || pandora->signalToMaster == HALT_SIGNAL_TM)
        // the LEDs will be solid Red
        haltLEDS();

    // if the master computer sends out an Idle Signal
    else if (pandora->signalFromMaster == IDLE_SIGNAL)
        // the LEDs will be solid purple
        idleLEDS();

    // if the master computer is sending a control signal
    else if (pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // the LEDs will be solid Blue
        controlLEDS();

        // Send motor PWMs and directions to motor controllers
        SendPWMSignal(&pandora->actuator0);
        SendPWMSignal(&pandora->actuator1);

        // Run estop interrupt
        run_estop = true;
    }

    // update the previous signal from master
    pandora->prevSignalFromMaster = pandora->signalFromMaster;

    return run_estop;
}

/**
 * loadDataForMaster
 *
 * serializes the data to send back to the master computer
 * over ethercat and puts all the data in its corresponding
 * index within the frame to send back.
 *
 * @param pandora: a pointer to the pandora structure,
 * the data to send is serialized from this structure
 */
void loadDataForMaster(PandoraLowLevel* pandora)
{
    // echo the signal the master was sending out
    pandora->signalToMaster = pandora->signalFromMaster;

    // load the signal frame in the etherCAT output frame
    pandora->easyCAT.etherCATOutputFrames.rawBytes[SIGNAL_INDEX] = (uint8_t)pandora->signalToMaster;

    // load the master process ID frame in the etherCAT output frame
    pandora->easyCAT.etherCATOutputFrames.rawBytes[PROCESS_ID_INDEX] = pandora->processIdFromMaster;

    // if the signal from master is a control signal
    if(pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Package force sensor 0 Newton value
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.actuator0ForceInNewtons = pandora->actuator0.forceSensor.newtons;

        // Package force sensor 1 Newton value
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.actuator1ForceInNewtons = pandora->actuator1.forceSensor.newtons;

        // Package encoder 0 radian value
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.joint0angleRadians = pandora->joint0.angleRads;

        // Package encoder 1 radian value
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.joint1angleRadians = pandora->joint1.angleRads;

        // Package IMU Acceleration X
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Ax = pandora->imu.accelerationData.Ax;

        // Package IMU Acceleration Y
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Ay = pandora->imu.accelerationData.Ay;

        // Package IMU Acceleration Z
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Az = pandora->imu.accelerationData.Az;

        // Package IMU Gyro X
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Gx = pandora->imu.gyroData.Gx;

        // Package IMU Gyro Y
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Gy = pandora->imu.gyroData.Gy;

        // Package IMU Gyro Z
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Gz = pandora->imu.gyroData.Gz;

        // Package IMU Magnetometer X
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Mx = pandora->imu.magnetometerData.Mx;

        // Package IMU Magnetometer Y
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.My = pandora->imu.magnetometerData.My;

        // Package IMU Magnetometer Z
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.Mz = pandora->imu.magnetometerData.Mz;

        // Package FT Sensor Force X
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftForceX = pandora->ftSensor.forceX;

        // Package FT Sensor Force Y
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftForceY = pandora->ftSensor.forceY;

        // Package FT Sensor Force Z
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftForceZ = pandora->ftSensor.forceZ;

        // Package FT Sensor Torque X
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftTorqueX = pandora->ftSensor.torqueX;

        // Package FT Sensor Torque Y
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftTorqueY = pandora->ftSensor.torqueY;

        // Package FT Sensor Torque Z
        pandora->easyCAT.etherCATOutputFrames.controlSignalFrame.ftTorqueZ = pandora->ftSensor.torqueZ;
    }

    // if the signal from master is an initialization signal
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        // Package the number of initialization frames received
        pandora->easyCAT.etherCATOutputFrames.initSignalFrame.numInitializationFramesReceived = pandora->numberOfInitFramesReceived;

        // Package the total number of initialization frames to receive
        pandora->easyCAT.etherCATOutputFrames.initSignalFrame.totalNumberOfInitializationFrames = NUMBER_OF_INITIALIZATION_FRAMES;
    }
}

