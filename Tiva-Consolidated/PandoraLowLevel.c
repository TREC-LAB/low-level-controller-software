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
    PandoraLowLevel pandora;

    // Configure the Location pins
//    tivaLocationPinsConfig();
    // Then read the location from them
 //   pandora.location = getLocationsFromPins();

    pandora.signalToMaster = 0;
    pandora.signalFromMaster = 0;
    pandora.prevSignalFromMaster = 0;

    pandora.masterLocationGuess = notValidLocation;

    pandora.prevProcessIdFromMaster = 0;
    pandora.processIdFromMaster = 0;

    pandora.imu = imuConstruct();
    pandora.ftSensor = ftSensorConstruct();

    pandora.initialized = false;
    // for the initialization DATA!! Allocate data on the heap to delete it later
    pandora.numberOfInitFramesReceived = 0;
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
//    tivaLocationPinsConfig();
    SSI3_Config_SPI(); // Configure SSI3 for SPI for use with EtherCAT
    int ret = EtherCAT_Init(&pandora->etherCATOutputFrames);
//    printf("%d\n", ret);
}

/**
 * ProcessCurrentInitFrame
 *
 * Processes the current initialization frame and applies it to
 * the PandoraLowLevel structure
 */
void StoreCurrentInitFrame(PandoraLowLevel* pandora)
{
    // actuator0 is the first initialization frame received
    uint8_t currentInitFrame = pandora->etherCATInputFrames.initSignalHeader.currentInitFrame;
    if(currentInitFrame == 0)
    {
        uint8_t actuatorNumber = 0;

        uint8_t rawQEIBase = pandora->etherCATInputFrames.initSignal0Frame.actuator0_QEIBaseNumber;
        uint32_t actuator0_QEIBase = QEI0_BASE + ((1 << 12) * rawQEIBase);

        uint16_t actuator0_QEISampleRate = pandora->etherCATInputFrames.initSignal0Frame.actuator0_QEISampleRate;
        uint32_t actuator0_QEICountsPerRotation = pandora->etherCATInputFrames.initSignal0Frame.actuator0_QEICountsPerRotation;

        uint8_t rawADCBase = pandora->etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorADCBaseNumber;
        uint32_t actuator0_ADCBase = ADC0_BASE + ((1 << 12) * rawADCBase);

        float actuator0_ForceSensorSlope = pandora->etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorSlope;
        float actuator0_ForceSensorOffset = pandora->etherCATInputFrames.initSignal0Frame.actuator0_ForceSensorOffset;

        Actuator actuator0 = actuatorConstruct(actuatorNumber, actuator0_QEIBase,
                                                actuator0_QEISampleRate, actuator0_QEICountsPerRotation,
                                                actuator0_ADCBase, actuator0_ForceSensorSlope,
                                                actuator0_ForceSensorOffset);
        pandora->actuator0 = actuator0;

    }
    else if(currentInitFrame == 1)
    {
        uint8_t actuatorNumber = 1;

        uint8_t rawQEIBase = pandora->etherCATInputFrames.initSignal1Frame.actuator1_QEIBaseNumber;
        uint32_t actuator1_QEIBase = QEI0_BASE + ((1 << 12) * rawQEIBase);

        uint16_t actuator1_QEISampleRate = pandora->etherCATInputFrames.initSignal1Frame.actuator1_QEISampleRate;
        uint32_t actuator1_QEICountsPerRotation = pandora->etherCATInputFrames.initSignal1Frame.actuator1_QEICountsPerRotation;

        uint8_t rawADCBase = pandora->etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorADCBaseNumber;
        uint32_t actuator1_ADCBase = ADC0_BASE + ((1 << 12) * rawADCBase);


        float actuator1_ForceSensorSlope = pandora->etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorSlope;
        float actuator1_ForceSensorOffset = pandora->etherCATInputFrames.initSignal1Frame.actuator1_ForceSensorOffset;

        Actuator actuator1 = actuatorConstruct(actuatorNumber, actuator1_QEIBase,
                                                actuator1_QEISampleRate, actuator1_QEICountsPerRotation,
                                                actuator1_ADCBase, actuator1_ForceSensorSlope,
                                                actuator1_ForceSensorOffset);
        pandora->actuator1 = actuator1;
    }
    else if(currentInitFrame == 2)
    {
        uint8_t rawSSIBaseNumber = pandora->etherCATInputFrames.initSignal2Frame.joint0_SSIBaseNumber;
        uint32_t joint0_SSIBaseNumber = SSI0_BASE + ((1 << 12) * rawSSIBaseNumber);

        uint8_t ssiEncoderBrandRaw = pandora->etherCATInputFrames.initSignal2Frame.joint0_SSIEncoderBrandRaw;
        SSIEncoderBrand joint0_SSIEncoderBrand = (SSIEncoderBrand)ssiEncoderBrandRaw;

        uint16_t joint0_SSIEncoderSampleRate = pandora->etherCATInputFrames.initSignal2Frame.joint0_SSISampleRate;
        int8_t joint0_jointReverseFactor = pandora->etherCATInputFrames.initSignal2Frame.joint0_ReverseFactor;

        float rawZero = pandora->etherCATInputFrames.initSignal2Frame.joint0_RawZeroPosition;
        float rawForwardRangeOfMotion = pandora->etherCATInputFrames.initSignal2Frame.joint0_RawForwardRangeOfMotion;
        float rawBackwardRangeOfMotion = pandora->etherCATInputFrames.initSignal2Frame.joint0_RawBackwardRangeOfMotion;
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
        Joint joint0 = jointConstruct(joint0_SSIBaseNumber, joint0_SSIEncoderBrand, joint0_SSIEncoderSampleRate,
                                      joint0_jointReverseFactor, joint0_rawZero,
                                      joint0_rawForwardRangeOfMotion, joint0_rawBackwardRangeOfMotion);
        pandora->joint0 = joint0;

    }
    else if(currentInitFrame == 3)
    {
        uint8_t rawSSIBaseNumber = pandora->etherCATInputFrames.initSignal3Frame.joint1_SSIBaseNumber;
        uint32_t joint1_SSIBaseNumber = SSI0_BASE + ((1 << 12) * rawSSIBaseNumber);

        uint8_t ssiEncoderBrandRaw = pandora->etherCATInputFrames.initSignal3Frame.joint1_SSIEncoderBrandRaw;
        SSIEncoderBrand joint1_SSIEncoderBrand = (SSIEncoderBrand)ssiEncoderBrandRaw;

        uint16_t joint1_SSIEncoderSampleRate = pandora->etherCATInputFrames.initSignal3Frame.joint1_SSISampleRate;
        int8_t joint1_jointReverseFactor = pandora->etherCATInputFrames.initSignal3Frame.joint1_ReverseFactor;

        float rawZero = pandora->etherCATInputFrames.initSignal3Frame.joint1_RawZeroPosition;
        float rawForwardRangeOfMotion = pandora->etherCATInputFrames.initSignal3Frame.joint1_RawForwardRangeOfMotion;
        float rawBackwardRangeOfMotion = pandora->etherCATInputFrames.initSignal3Frame.joint1_RamBackwardRangeOfMotion;
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
        Joint joint1 = jointConstruct(joint1_SSIBaseNumber, joint1_SSIEncoderBrand, joint1_SSIEncoderSampleRate,
                                      joint1_jointReverseFactor, joint1_rawZero,
                                      joint1_rawForwardRangeOfMotion, joint1_rawBackwardRangeOfMotion);
        pandora->joint1 = joint1;
    }
    else if(currentInitFrame == 4)
    {
        uint8_t imuEnable = pandora->etherCATInputFrames.initSignal4Frame.imuEnable;
        pandora->imu.enabled = imuEnable;
    }
    else if(currentInitFrame == 5)
    {
        uint8_t softwareEStopEnable = pandora->etherCATInputFrames.initSignal5Frame.softwareEStopEnable;
        PandoraLowLevelSettings settings;
        settings.softwareEStopEnable = softwareEStopEnable;
        pandora->settings = settings;
    }
}

/**
 * tivaInit
 *
 * The initialization function with inits all of the
 * Tiva's peripherals needed.
 *
 * @param pandora: a pointer to the pandora structure
 */
void tivaInit(PandoraLowLevel* pandora)
{

    PWMConfig();
    enableForceSensor(&pandora->actuator0.forceSensor);
    enableForceSensor(&pandora->actuator1.forceSensor);
    enableSSIEncoder(&pandora->joint0.encoder);
    enableSSIEncoder(&pandora->joint1.encoder);
    enableQEIEncoder(&pandora->actuator0.motorEncoder);
    enableQEIEncoder(&pandora->actuator1.motorEncoder);
    enableDebugLEDS();
    if(pandora->imu.enabled)
        imuEnable(&pandora->imu);
    ftSensorEnable(&pandora->ftSensor);
    timer1A_Config();
    timer2A_Config();
    timer3A_Config();
}

void GetAndSendDataToMaster(PandoraLowLevel* pandora)
{
    EtherCAT_MainTask(&pandora->etherCATOutputFrames, &pandora->etherCATInputFrames);
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
    SysCtlPeripheralEnable(LED_PERIPH);
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED | GREEN_LED);
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
    static bool led_on = false;
    static uint32_t led_start = 0;
    static uint32_t led_current = 0;

    // Blink Red
    if (!led_on && (abs(led_current - led_start) > 100))
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GREEN_LED, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, BLUE_LED, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, RED_LED, RED_LED);
        led_on = true;
        led_start = 0;
        led_current = 0;
    }
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
    /////
    if(pandora->signalFromMaster != CONTROL_SIGNAL)
    {
        // STOP MOTORS
        pandora->actuator0.pwmGenerator.dutyCycle = 0;
        pandora->actuator1.pwmGenerator.dutyCycle = 0;
        SendPWMSignal(&pandora->actuator0);
        SendPWMSignal(&pandora->actuator1);
    }
    /////
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
    pandora->signalFromMaster = pandora->etherCATInputFrames.rawBytes[SIGNAL_INDEX];
    // notice how the different control signals effect
    // how the data gets stored
    if (pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Set joint 0 direction
        pandora->actuator0.pwmGenerator.direction = pandora->etherCATInputFrames.controlSignalFrame.actuator0Direction;

        // Set joint 0 duty cycle
        pandora->actuator0.pwmGenerator.dutyCycle = pandora->etherCATInputFrames.controlSignalFrame.actuator0DutyCycle;

        // Set joint 1 direction
        pandora->actuator1.pwmGenerator.direction = pandora->etherCATInputFrames.controlSignalFrame.actuator1Direction;

        // Set joint 1 duty cycle
        pandora->actuator1.pwmGenerator.dutyCycle = pandora->etherCATInputFrames.controlSignalFrame.actuator1DutyCycle;
    }
    else if (pandora->signalFromMaster == LOCATION_DEBUG_SIGNAL)
    {
        pandora->masterLocationGuess = (TivaLocations)pandora->etherCATInputFrames.locationDebugSignalFrame.masterLocationGuess;
    }
    else if (pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        pandora->numberOfInitFramesReceived = pandora->etherCATInputFrames.initSignalHeader.currentInitFrame + 1;
    }
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
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        // for re-initialization
        if (pandora->initialized)
            pandora->initialized = false;
        StoreCurrentInitFrame(pandora);
        if(pandora->numberOfInitFramesReceived == NUMBER_OF_INITIALIZATION_FRAMES)
        {
            pandora->initialized = true;
            return true;
        }
        pandora->prevSignalFromMaster = pandora->signalFromMaster;
        return false;
    }
    else if(pandora->signalFromMaster != INITIALIZATION_SIGNAL && pandora->initialized)
        pandora->numberOfInitFramesReceived = 0;

    if(!pandora->initialized)
    {
        pandora->prevSignalFromMaster = pandora->signalFromMaster;
        return false;
    }

    /***********FOR ALL OTHER SIGNALS***********/

    int run_estop = false;
    // DO NOT REMOVE THIS LINE
    // Ensures motor PWMs are set to 0 in the case signalFromMaster != CONTROL_SIGNAL but motors are still moving
    checkActuatorDisable(pandora);

    if (pandora->signalFromMaster == NOT_CONNECTED)
    {
        // Waiting for master to connect
        notConnectedLEDS(); // Flash red
    }
    else if (pandora->signalFromMaster == HALT_SIGNAL || pandora->signalToMaster == HALT_SIGNAL_TM)
    {
        // Stop motors signal received from master
        haltLEDS(); // Solid red
    }
    else if (pandora->signalFromMaster == LOCATION_DEBUG_SIGNAL)
    {
        // Check if master's guessed TIVA location is the same as TIVA's actual location
        checkLocationLEDS(pandora->masterLocationGuess, pandora->location);
        // If guess is wrong, solid yellow
        // If guess is correct, flash blue
    }
    else if (pandora->signalFromMaster == IDLE_SIGNAL)
    {
        // Master is active but not sending or receiving anything
        idleLEDS(); // Solid purple
    }
    else if (pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        controlLEDS();
        // Send motor PWMs and directions to motor controllers
        SendPWMSignal(&pandora->actuator0);
        SendPWMSignal(&pandora->actuator1);

        // Run estop interrupt
        run_estop = true;
    }

    pandora->prevSignalFromMaster = pandora->signalFromMaster;
    return run_estop;
}

/**
 * loadDataForMaster
 *
 * serializes the data to send back to the masteri
 * over ethercat and puts all the data in its corresponding
 * index within the frame to send back.
 *
 * @param pandora: a pointer to the pandora structure,
 * the data to send is serialized from this structure
 */
void loadDataForMaster(PandoraLowLevel* pandora)
{
    pandora->signalToMaster = pandora->signalFromMaster;
    pandora->etherCATOutputFrames.rawBytes[SIGNAL_INDEX] = (uint8_t)pandora->signalToMaster;
    pandora->etherCATOutputFrames.rawBytes[PROCESS_ID_INDEX] = pandora->processIdFromMaster;

    // Notice how the signals determine how the data gets serialized
    if(pandora->signalFromMaster == LOCATION_DEBUG_SIGNAL && pandora->location == pandora->masterLocationGuess)
        pandora->etherCATOutputFrames.locationDebugSignalFrame.masterLocationGuess = (uint8_t)(pandora->location);
    if(pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Package force sensor 0 Newton value
        pandora->etherCATOutputFrames.controlSignalFrame.actuator0ForceInNewtons = pandora->actuator0.forceSensor.newtons;

        // Package force sensor 1 Newton value
        pandora->etherCATOutputFrames.controlSignalFrame.actuator1ForceInNewtons = pandora->actuator1.forceSensor.newtons;

        // Package encoder 0 radian value
        pandora->etherCATOutputFrames.controlSignalFrame.joint0angleRadians = pandora->joint0.angleRads;

        // Package encoder 1 radian value
        pandora->etherCATOutputFrames.controlSignalFrame.joint1angleRadians = pandora->joint1.angleRads;

        // Package IMU Acceleration X
        pandora->etherCATOutputFrames.controlSignalFrame.Ax = pandora->imu.accelerationData.Ax;

        // Package IMU Acceleration Y
        pandora->etherCATOutputFrames.controlSignalFrame.Ay = pandora->imu.accelerationData.Ay;

        // Package IMU Acceleration Z
        pandora->etherCATOutputFrames.controlSignalFrame.Az = pandora->imu.accelerationData.Az;

        // Package IMU Gyro X
        pandora->etherCATOutputFrames.controlSignalFrame.Gx = pandora->imu.gyroData.Gx;

        // Package IMU Gyro Y
        pandora->etherCATOutputFrames.controlSignalFrame.Gy = pandora->imu.gyroData.Gy;

        // Package IMU Gyro Z
        pandora->etherCATOutputFrames.controlSignalFrame.Gz = pandora->imu.gyroData.Gz;

        // Package IMU Magnetometer X
        pandora->etherCATOutputFrames.controlSignalFrame.Mx = pandora->imu.magnetometerData.Mx;

        // Package IMU Magnetometer Y
        pandora->etherCATOutputFrames.controlSignalFrame.My = pandora->imu.magnetometerData.My;

        // Package IMU Magnetometer Z
        pandora->etherCATOutputFrames.controlSignalFrame.Mz = pandora->imu.magnetometerData.Mz;

        // Package FT Sensor Force X
        pandora->etherCATOutputFrames.controlSignalFrame.ftForceX = pandora->ftSensor.forceX;

        // Package FT Sensor Force Y
        pandora->etherCATOutputFrames.controlSignalFrame.ftForceY = pandora->ftSensor.forceY;

        // Package FT Sensor Force Z
        pandora->etherCATOutputFrames.controlSignalFrame.ftForceZ = pandora->ftSensor.forceZ;

        // Package FT Sensor Torque X
        pandora->etherCATOutputFrames.controlSignalFrame.ftTorqueX = pandora->ftSensor.torqueX;

        // Package FT Sensor Torque Y
        pandora->etherCATOutputFrames.controlSignalFrame.ftTorqueY = pandora->ftSensor.torqueY;

        // Package FT Sensor Torque Z
        pandora->etherCATOutputFrames.controlSignalFrame.ftTorqueZ = pandora->ftSensor.torqueZ;
    }
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        pandora->etherCATOutputFrames.initSignalFrame.numInitializationFramesReceived = pandora->numberOfInitFramesReceived;
        pandora->etherCATOutputFrames.initSignalFrame.totalNumberOfInitializationFrames = NUMBER_OF_INITIALIZATION_FRAMES;
    }
}

