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
    tivaLocationPinsConfig();
    // Then read the location from them
    pandora.location = getLocationsFromPins();

    pandora.signalToMaster = 0;
    pandora.signalFromMaster = 0;
    pandora.prevSignalFromMaster = 0;

    pandora.masterLocationGuess = notValidLocation;

    pandora.prevProcessIdFromMaster = 0;
    pandora.processIdFromMaster = 0;

    pandora.initialized = false;
    // for the initialization DATA!! Allocate data on the heap to delete it later
    pandora.initializationData.numberOfInitFramesReceived = 0;
    pandora.initializationData.initalizationDataBlock =
            (void**)malloc(sizeof(void**) * NUMBER_OF_INITIALIZATION_FRAMES);
    int i;
    for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
            *(pandora.initializationData.initalizationDataBlock + i) = (void*)malloc(FRAME_SIZE);
    return pandora;
}

/**
 * tivaLocationPinsConfig
 *
 * Configures the location pins which the
 * Tiva uses to determine which location
 * it is in.
 */
void tivaLocationPinsConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Configure input pin
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_WAKE_LOW);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_WAKE_LOW);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_WAKE_LOW);
}

/**
 * getLocationsFromPins
 *
 * Gets the Tiva location from the pin
 * configuration at startup.
 *
 * @return: the location the tiva is in
 */
TivaLocations getLocationsFromPins()
{
    TivaLocations tivaLocation;
    TivaLocationBitSet locationBitSet;

    // read from the pins
    uint32_t readValue = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6);
    locationBitSet.Bit0 = ((readValue & GPIO_PIN_6) == GPIO_PIN_6);
    readValue = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
    locationBitSet.Bit1 = ((readValue & GPIO_PIN_7) == GPIO_PIN_7);

    readValue = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3);
    locationBitSet.Bit2 = ((readValue & GPIO_PIN_3) == GPIO_PIN_3);

    // get the location from the pin set
    // The bit is True if there is a connection to it
    if(locationBitSet.Bit0 && !locationBitSet.Bit1 && !locationBitSet.Bit2)
        tivaLocation = hipL;
    else if(!locationBitSet.Bit0 && locationBitSet.Bit1 && !locationBitSet.Bit2)
        tivaLocation = hipR;
    else if(locationBitSet.Bit0 && locationBitSet.Bit1 && !locationBitSet.Bit2)
        tivaLocation = thighL;
    else if(!locationBitSet.Bit0 && !locationBitSet.Bit1 && locationBitSet.Bit2)
        tivaLocation = thighR;
    else if(locationBitSet.Bit0 && !locationBitSet.Bit1 && locationBitSet.Bit2)
        tivaLocation = ankleL;
    else if(!locationBitSet.Bit0 && locationBitSet.Bit1 && locationBitSet.Bit2)
        tivaLocation = ankleR;
    else if(locationBitSet.Bit0 && locationBitSet.Bit1 && locationBitSet.Bit2)
        tivaLocation = DEBUG;
    else
        tivaLocation = notValidLocation;

    return tivaLocation;
}

/**
 * tivaInitEtherCAT
 *
 * Initializes the ethercat board on the Tiva. Note that this function
 * is called before the other peripheral based initialization functions.
 * This is because the Tiva must read the initialization input data
 * from the master first to know how to configure the rest of its peripherals
 */
void tivaInitEtherCAT()
{
    tivaLocationPinsConfig();
    SSI3_Config_SPI(); // Configure SSI3 for SPI for use with EtherCAT
    int ret = EtherCAT_Init();
   // printf("%d\n", ret);
}

/**
 * storeInitFrame
 *
 * Stores the most recent raw initialization frame that was sent from the
 * master by adding it to the dynamically allocated 2D array.
 *
 * @param pandora: a pointer to the pandora Tiva struct
 */
void storeInitFrame(PandoraLowLevel* pandora)
{
    // check to make sure the conditions match up
    if(pandora->signalFromMaster != INITIALIZATION_SIGNAL)
        return;

    // check to make sure there is no error regarding initialization
    // this is important because sending the wrong initialization data can crash
    // the tiva. Initialization Frame Numbers start counting from zero
    uint8_t initFrameNumber = etherCATInputFrames.rawBytes[INITIALIZATION_FRAME_NUMBER_INDEX];
    if(initFrameNumber != pandora->initializationData.numberOfInitFramesReceived
            || initFrameNumber > NUMBER_OF_INITIALIZATION_FRAMES)
    {
        pandora->signalToMaster = HALT_SIGNAL_TM;
        return;
    }
    int i;
    // 3 is the start of the valuable data
    for(i = 3; i < FRAME_SIZE; i++)
        *(uint8_t*)(*(uint8_t**)(pandora->initializationData.initalizationDataBlock + initFrameNumber) + i) =
                etherCATInputFrames.rawBytes[i];
    pandora->initializationData.numberOfInitFramesReceived++;
}

/**
 * ApplyInitializationSettings
 *
 * Called after the Tiva received all of the initialziation frames
 * from the master. This functions deserializes the initialization frames
 * and initializes the pandora struct passed from the initialization data
 *
 * @param pandora: a pointer to the pandora Tiva struct which
 * will be initialized with the deserialized data
 */
void ApplyInitializationSettings(PandoraLowLevel* pandora)
{
    // TODO: validate the initialization settings
    Actuator actuator0, actuator1;
    Joint joint0, joint1;
    int i;
    void** initializationData = pandora->initializationData.initalizationDataBlock;
    ByteData byteData;
    FloatByteData floatByteData;
    for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
    {
        // data stored from the first initialization frame!
        // this frame is for the actuator
        if(i == 0 || i == 1)
        {
            uint8_t QEIBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 3));
            uint32_t actuatorMotorEncoderQEIBaseInit = QEI0_BASE + ((1 << 12) * QEIBaseNumber);

            byteData.Byte[3] = 0;
            byteData.Byte[2] = 0;
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 4));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 5));

            uint16_t sampleRateInit = (uint16_t)byteData.Word[0];

            byteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 6));
            byteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 7));
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 8));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 9));

            int32_t countsPerRotationInit = byteData.intData;

            uint8_t ADCBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 10));
            uint32_t actuatorForceSensorADCBaseInit = ADC0_BASE + ((1 << 12) * ADCBaseNumber);

            floatByteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 11));
            floatByteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 12));
            floatByteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 13));
            floatByteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 14));

            float forceSensorSlopeInit = floatByteData.floatData;

            floatByteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 15));
            floatByteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 16));
            floatByteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 17));
            floatByteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 18));

            float forceSensorOffsetInit = floatByteData.floatData;

            if(i == 0)
                // 0 for actuator 0
                actuator0 = actuatorConstruct(0, actuatorMotorEncoderQEIBaseInit, sampleRateInit, countsPerRotationInit,
                                          actuatorForceSensorADCBaseInit, forceSensorSlopeInit, forceSensorOffsetInit);
            else
                // 1 for actuator 1
                actuator1 = actuatorConstruct(1, actuatorMotorEncoderQEIBaseInit, sampleRateInit, countsPerRotationInit,
                                          actuatorForceSensorADCBaseInit, forceSensorSlopeInit, forceSensorOffsetInit);
        }
        // this data is for the joint
        else
        {
            uint8_t SSIBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 3));
            uint32_t jointEncoderSSIBaseInit = SSI0_BASE + ((1 << 12) * SSIBaseNumber);

            SSIEncoderBrand SSIEncoderBrandInit = (SSIEncoderBrand)(*(uint8_t*)(*(uint8_t**)(initializationData + i) + 4));

            byteData.Byte[3] = 0;
            byteData.Byte[2] = 0;
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 5));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 6));

            uint16_t sampleRateInit = byteData.Word[0];

            int8_t jointReverseFactor =  (*(int8_t*)(*(int8_t**)(initializationData + i) + 7));

            floatByteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 8));
            floatByteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 9));
            floatByteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 10));
            floatByteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 11));

            uint16_t jointRawZeroPosition;
            if(SSIEncoderBrandInit == Gurley_Encoder)
                jointRawZeroPosition = (uint16_t)(floatByteData.floatData * (65535.0 / 180.0));
            else if(SSIEncoderBrandInit == Orbis_Encoder)
                jointRawZeroPosition = (uint16_t)(floatByteData.floatData * (16383.0 / 360.0));

            floatByteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 12));
            floatByteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 13));
            floatByteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 14));
            floatByteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 15));

            uint16_t jointRawForwardRangeOfMotion;
            if(SSIEncoderBrandInit == Gurley_Encoder)
                jointRawForwardRangeOfMotion = (uint16_t)(floatByteData.floatData * (65535.0) / 180.0);
            else if(SSIEncoderBrandInit == Orbis_Encoder)
                jointRawForwardRangeOfMotion = (uint16_t)(floatByteData.floatData * (16383.0 / 360.0));

            floatByteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 16));
            floatByteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 17));
            floatByteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 18));
            floatByteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 19));

            uint16_t jointRawBackwardRangeOfMotion;
            if(SSIEncoderBrandInit == Gurley_Encoder)
                jointRawBackwardRangeOfMotion = (uint16_t)(floatByteData.floatData * (65535.0 / 180.0));
            else if(SSIEncoderBrandInit == Orbis_Encoder)
                jointRawBackwardRangeOfMotion = (uint16_t)(floatByteData.floatData * (16383.0 / 360.0));

            if(i == 2)
                joint0 = jointConstruct(jointEncoderSSIBaseInit, SSIEncoderBrandInit, sampleRateInit,
                                        jointReverseFactor, jointRawZeroPosition,
                                        jointRawForwardRangeOfMotion, jointRawBackwardRangeOfMotion);
            else
                joint1 = jointConstruct(jointEncoderSSIBaseInit, SSIEncoderBrandInit, sampleRateInit,
                                        jointReverseFactor, jointRawZeroPosition,
                                        jointRawForwardRangeOfMotion, jointRawBackwardRangeOfMotion);
        }
    }
    pandora->joint0 = joint0;
    pandora->joint1 = joint1;
    pandora->actuator0 = actuator0;
    pandora->actuator1 = actuator1;

    pandora->initialized = true;
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
//    debugLEDSConfig();    TODO: uncomment when LEDS are moved to different pins
    timer1A_Config();
    timer2A_Config();
    timer3A_Config();

//    printf("ret: %d\n", ret);
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
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
}

/**
 * disableDebugLEDs
 *
 * Disables the LEDs used for debugging.
 */
void disableDebugLEDs()
{
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    SysCtlPeripheralDisable(LED_PERIPH);
}

/**
 * checkLocationLEDS
 *
 * Emits certain blink codes or solid colors depending on if the
 * Tiva Location is the same as the location guess input.
 *
 * @param locationGuess: the guess location
 * @param actualLocation: the actual location the Tiva is in which
 * is determined by the location pins at startup
 */
void checkLocationLEDS(TivaLocations locationGuess, TivaLocations actualLocation)
{
    static bool led_on = false;
    static uint32_t led_start = 0;
    static uint32_t led_current = 0;

    if(locationGuess != actualLocation || locationGuess == notValidLocation)
    {
        // if the location guess is wrong or not value, turn the LEDS to yellow
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    }
    else
    {
        // if the location guess matches the actual location
        // then blink blue
        if (!led_on && (abs(led_current - led_start) > 100))
        {
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
            led_on = true;
            led_start = 0;
            led_current = 0;
        }
        else if (led_on && (abs(led_current - led_start) > 50))
        {
            GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
            led_on = false;
            led_start = 0;
            led_current = 0;
        }
        led_current++;
    }
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
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        led_on = true;
        led_start = 0;
        led_current = 0;
    }
    else if (led_on && (abs(led_current - led_start) > 50))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
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
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
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
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
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
        pandora->actuator0.dutyCycle = 0;
        pandora->actuator1.dutyCycle = 0;
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
    pandora->signalFromMaster = etherCATInputFrames.rawBytes[SIGNAL_INDEX];
    // notice how the different control signals effect
    // how the data gets stored
    if (pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Set joint 0 direction
        pandora->actuator0.direction = etherCATInputFrames.controlSignalFrame.actuator0Direction;

        // Set joint 0 duty cycle
        pandora->actuator0.dutyCycle = etherCATInputFrames.controlSignalFrame.actuator0DutyCycle;

        // Set joint 1 direction
        pandora->actuator1.direction = etherCATInputFrames.controlSignalFrame.actuator1Direction;

        // Set joint 1 duty cycle
        pandora->actuator1.dutyCycle = etherCATInputFrames.controlSignalFrame.actuator1DutyCycle;
    }
    else if (pandora->signalFromMaster == LOCATION_DEBUG_SIGNAL)
    {
        pandora->masterLocationGuess = (TivaLocations)etherCATInputFrames.locationDebugSignalFrame.masterLocationGuess;
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

    // check for initialization or reinitialization and handle the case correctly
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        // for re-initialization
        if (pandora->initialized)
        {
            pandora->initialized = false;
            pandora->initializationData.numberOfInitFramesReceived = 0;

            // re-allocate data on the heap to store the incoming init data
            pandora->initializationData.initalizationDataBlock = (void**)malloc(sizeof(void**) * NUMBER_OF_INITIALIZATION_FRAMES);
            int i;
            for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
                *(pandora->initializationData.initalizationDataBlock + i) = (void*)malloc(FRAME_SIZE);
        }
        storeInitFrame(pandora);
        // once all of the initialization frames are received
        if(pandora->initializationData.numberOfInitFramesReceived == NUMBER_OF_INITIALIZATION_FRAMES)
        {
            ApplyInitializationSettings(pandora);
            pandora->initialized = true;
            // free all temp init data after it is no longer needed
            int i;
            for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
                free((pandora->initializationData.initalizationDataBlock + i));
            free(pandora->initializationData.initalizationDataBlock);
        }
        return false;
    }

    if(!pandora->initialized)
        return false;

    int run_estop = false;

    // DO NOT REMOVE THIS LINE
    // Ensures motor PWMs are set to 0 in the case signalFromMaster != CONTROL_SIGNAL but motors are still moving
    checkActuatorDisable(pandora);

    // Check if we have switched into control mode
    if (pandora->prevSignalFromMaster != CONTROL_SIGNAL && pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Reconfigure port F to use SSI1
        disableDebugLEDs();
        if (!pandora->joint1.encoder.enabled)
        {
            enableSSIEncoder(&pandora->joint1.encoder);
        }
    }

    // Check if we have switched out of control mode
    if (pandora->prevSignalFromMaster == CONTROL_SIGNAL && pandora->signalFromMaster != CONTROL_SIGNAL)
    {
        if (pandora->joint1.encoder.enabled)
        {
            disableSSIEncoder(&pandora->joint1.encoder);
        }
        enableDebugLEDS();
    }

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
    etherCATOutputFrames.rawBytes[SIGNAL_INDEX] = (uint8_t)pandora->signalToMaster;
    etherCATOutputFrames.rawBytes[PROCESS_ID_INDEX] = pandora->processIdFromMaster;

    // Notice how the signals determine how the data gets serialized
    if(pandora->signalFromMaster == LOCATION_DEBUG_SIGNAL && pandora->location == pandora->masterLocationGuess)
        etherCATOutputFrames.locationDebugSignalFrame.masterLocationGuess = (uint8_t)(pandora->location);
    if(pandora->signalFromMaster == CONTROL_SIGNAL)
    {
        // Package force sensor 0 Newton value
        etherCATOutputFrames.controlSignalFrame.actuator0ForceInNewtons = pandora->actuator0.forceSensor.newtons;

        // Package force sensor 1 Newton value
        etherCATOutputFrames.controlSignalFrame.actuator1ForceInNewtons = pandora->actuator1.forceSensor.newtons;

        // Package encoder 0 radian value
        etherCATOutputFrames.controlSignalFrame.joint0angleRadians = pandora->joint0.angleRads;

        // Package encoder 1 radian value
        etherCATOutputFrames.controlSignalFrame.joint1angleRadians = pandora->joint1.angleRads;
    }
    if(pandora->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        etherCATOutputFrames.initSignalFrame.numInitializationFramesReceived = pandora->initializationData.numberOfInitFramesReceived;
        etherCATOutputFrames.initSignalFrame.totalNumberOfInitializationFrames = NUMBER_OF_INITIALIZATION_FRAMES;
    }
}

