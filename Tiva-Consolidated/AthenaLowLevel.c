/**
 * AthenaLowLevel.c
 * @author: Nick Tremaroli, Sam Schoedel, and Connor Herron
 * Defines all of the low-level features and functions
 * of Athena
 */
#include "AthenaLowLevel.h"

/**
 * joint0Config
 *
 * Configures Joint 0 for this Tiva on Athena.
 *
 * @param sample_rate: The sample rate of the Encoder for this joint
 * @return: an initialized joint for position 0
 */
/*
Joint joint0Config(uint16_t sample_rate,  TivaLocations tivaLocation)
{
    Joint joint;

    // Initialize correct Absolute Encoder based on Tiva Location for the 0th board side.
    enum EncoderBrand absEncoderBrand = checkAbsoluteEncoderBrand(0, tivaLocation);

    joint.forceSensor   = forceSensorConstruct(ADC0_BASE);
    joint.encoder       = encoderConstruct(SSI0_BASE, QEI0_BASE, SSI_Encoder, sample_rate, absEncoderBrand);
    joint.motorEncoder  = encoderConstruct(SSI0_BASE, QEI0_BASE, QEI_Encoder, sample_rate, Motor_Encoder);

    joint.dutyCycle = 0.0;
    joint.direction = 0;

    if (joint.encoder.encoderBrand == Gurley_Encoder){
        joint.upperJointLimitRaw = 65535;
    }
    // Orbis has 14 bit resolution
    else if (joint.encoder.encoderBrand == Orbis_Encoder){
        joint.upperJointLimitRaw = 16383;
    }
    joint.lowerJointLimitRaw = 0;

    return joint;
}
*/
/**
 * joint1Config
 *
 * Configures Joint 1 for this Tiva on Athena.
 *
 * @param sample_rate: The sample rate of the Encoder for this joint
 * @return: an initialized joint for position 1
 */
/*
Joint joint1Config(uint16_t sample_rate, TivaLocations tivaLocation)
{
    Joint joint;

    // Initialize correct Absolute Encoder based on Tiva Location for the 1th board side.
    enum EncoderBrand absEncoderBrand = checkAbsoluteEncoderBrand(1, tivaLocation);

    joint.forceSensor   = forceSensorConstruct(ADC1_BASE);
    joint.encoder       = encoderConstruct(SSI1_BASE, QEI1_BASE, SSI_Encoder, sample_rate, absEncoderBrand);
    joint.motorEncoder  = encoderConstruct(SSI1_BASE, QEI1_BASE, QEI_Encoder, sample_rate, Motor_Encoder);

    joint.dutyCycle = 0.0;
    joint.direction = 0;

    if (joint.encoder.encoderBrand == Gurley_Encoder){
        joint.upperJointLimitRaw = 65535;
    }
    // Orbis has 14 bit resolution
    else if (joint.encoder.encoderBrand == Orbis_Encoder){
        joint.upperJointLimitRaw = 16383;
    }
    joint.lowerJointLimitRaw = 0;

    return joint;
}
*/
/**
 * athenaConstruct
 *
 * Constructs the AthenaLowLevel structure and initializes
 * all of the starting values to 0.
 *
 * @param sample_rate: the sample rate to be used by the encoders
 * at the joints this Tiva is connected too
 * @return: a basic AthenaLowLevel structure
 */
AthenaLowLevel athenaConstruct(uint16_t sample_rate)
{
    AthenaLowLevel athena;

    // Configure the Location pins
    tivaLocationPinsConfig();
    // Then read the location from them
    athena.location = getLocationsFromPins();

    athena.signalToMaster = 0;
    athena.signalFromMaster = 0;
    athena.prevSignalFromMaster = 0;

    athena.masterLocationGuess = notValidLocation;

    athena.prevProcessIdFromMaster = 0;
    athena.processIdFromMaster = 0;

    athena.initialized = false;
    // for the initialization DATA!! Allocate data on the heap to delete it later
    athena.initializationData.initializationFrameNumberToReceive = 0;
    athena.initializationData.initalizationDataBlock =
            (void**)malloc(sizeof(void**) * NUMBER_OF_INITIALIZATION_FRAMES);
    int i;
    for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
            *(athena.initializationData.initalizationDataBlock + i) = (void*)malloc(sizeof(FRAME_SIZE));
    return athena;
}

void storeInitFrame(AthenaLowLevel* athena)
{
    // check to make sure the conditions match up
    if(athena->signalFromMaster != INITIALIZATION_SIGNAL)
        return;

    // check to make sure there is no error regarding initialization
    // this is important because sending the wrong initialization data can crash
    // the tiva
    uint8_t initFrameNumber = MasterToTiva.Byte[INITIALIZATION_FRAME_NUMBER_INDEX];
    if(initFrameNumber != athena->initializationData.initializationFrameNumberToReceive
            || initFrameNumber > NUMBER_OF_INITIALIZATION_FRAMES)
    {
        athena->signalToMaster = HALT_SIGNAL_TM;
        return;
    }
    int i;
    // 3 is the start of the valuable data
    for(i = 3; i < FRAME_SIZE; i++)
    {
        *(uint8_t*)(*(uint8_t**)(athena->initializationData.initalizationDataBlock + initFrameNumber) + i) =
                MasterToTiva.Byte[i];
    }
    athena->initializationData.initializationFrameNumberToReceive++;
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
 * tivaLocationPinsConfig
 *
 * Configures the location pins which the
 * Tiva uses to determine which location
 * it is in.
 *
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
 * debugLEDSConfig
 *
 * Configures the LEDs which are used for debugging
 * by emitting different colors of lights.
 *
 */
void debugLEDSConfig()
{
    SysCtlPeripheralEnable(LED_PERIPH);
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED | GREEN_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
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
 *
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
 *
 */
void idleLEDS()
{
    // solid purple
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
}

/**
 * modifyLEDS
 *
 * Configures the LEDS to act a certain
 * way when the master is sending signals to
 * modify the encoder or force sensor limits.
 *
 */
void modifyLEDs()
{
    static bool led_on = false;
    static uint32_t led_start = 0;
    static uint32_t led_current = 0;

    // blink purple
    if (!led_on && (abs(led_current - led_start) > 100))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        led_on = true;
        led_start = 0;
        led_current = 0;
    }
    else if (led_on && (abs(led_current - led_start) > 50))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
        led_on = false;
        led_start = 0;
        led_current = 0;
    }
    led_current++;
}

/**
 * haltLEDS
 *
 * configures the LEDS to act a certain
 * way when the master sends a halt signal
 * to the Tiva.
 *
 */
void haltLEDS()
{
    // solid red
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
}

/**
 * tivaInit
 *
 * The intalization function with inits all of the
 * Tiva's peripherals needed.
 *
 * @param athena: a pointer to the athena structure
 */
void tivaInit(AthenaLowLevel* athena)
{
    PWMConfig();
    enableForceSensor(&athena->actuator0.forceSensor);
    enableForceSensor(&athena->actuator1.forceSensor);
    enableSSIEncoder(&athena->joint0.encoder);
    enableSSIEncoder(&athena->joint1.encoder);
    enableQEIEncoder(&athena->actuator0.motorEncoder);
    enableQEIEncoder(&athena->actuator1.motorEncoder);
    debugLEDSConfig();
    timer1A_Config();
    timer2A_Config();
    timer3A_Config();
//    UART1Config();

//    printf("ret: %d\n", ret);
}

void PandoraInit(AthenaLowLevel* athena)
{
    Actuator actuator0, actuator1;
    Joint joint0, joint1;
    int i;
    void** initializationData = athena->initializationData.initalizationDataBlock;
    ByteData byteData;
    FloatByteData floatByteData;
    for(i = 0; i < NUMBER_OF_INITIALIZATION_FRAMES; i++)
    {
        // data stored from the first initialization frame!
        // this frame is for the actuator
        if(i == 0 || i == 1)
        {
            uint8_t QEIBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 3));
            uint32_t actuatorMotorEncoderQEIBaseInit = QEI0_BASE + ((1 << 16) * QEIBaseNumber);

            byteData.Byte[3] = 0;
            byteData.Byte[2] = 0;
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 4));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 5));

            uint16_t sampleRateInit = byteData.Word[0];

            byteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 6));
            byteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 7));
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 8));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 9));

            int32_t countsPerRotationInit = byteData.intData;

            uint8_t ADCBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 10));
            uint32_t actuatorForceSensorADCBaseInit = ADC0_BASE + ((1 << 16) * ADCBaseNumber);

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
                actuator0 = actuatorConstruct(actuatorMotorEncoderQEIBaseInit, sampleRateInit, countsPerRotationInit,
                                          actuatorForceSensorADCBaseInit, forceSensorSlopeInit, forceSensorOffsetInit);
            else
                actuator1 = actuatorConstruct(actuatorMotorEncoderQEIBaseInit, sampleRateInit, countsPerRotationInit,
                                          actuatorForceSensorADCBaseInit, forceSensorSlopeInit, forceSensorOffsetInit);
        }
        else
        {
            uint8_t SSIBaseNumber = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 3));
            uint32_t jointEncoderSSIBaseInit = SSI0_BASE + ((1 << 16) * SSIBaseNumber);

            SSIEncoderBrand SSIEncoderBrandInit = (SSIEncoderBrand)(*(uint8_t*)(*(uint8_t**)(initializationData + i) + 4));

            byteData.Byte[3] = 0;
            byteData.Byte[2] = 0;
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 5));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 6));

            uint16_t sampleRateInit = byteData.Word[0];

            byteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 7));
            byteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 8));
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 9));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 10));

            uint32_t upperLimitRawInit = byteData.intData;

            byteData.Byte[3] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 11));
            byteData.Byte[2] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 12));
            byteData.Byte[1] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 13));
            byteData.Byte[0] = (*(uint8_t*)(*(uint8_t**)(initializationData + i) + 14));

            uint32_t lowerLimitRawInit = byteData.intData;

            if(i == 2)
                joint0 = jointConstruct(jointEncoderSSIBaseInit, SSIEncoderBrandInit, sampleRateInit,
                                        upperLimitRawInit, lowerLimitRawInit);
            else
                joint1 = jointConstruct(jointEncoderSSIBaseInit, SSIEncoderBrandInit, sampleRateInit,
                                        upperLimitRawInit, lowerLimitRawInit);
        }
    }

    athena->joint0 = joint0;
    athena->joint1 = joint1;
    athena->actuator0 = actuator0;
    athena->actuator1 = actuator1;

    athena->initialized = true;
}

void tivaInitEtherCAT()
{

    SSI3_Config_SPI(); // Configure SSI3 for SPI for use with EtherCAT
    int ret = EtherCAT_Init();
    printf("%d\n", ret);
}

/**
 * loadDataForMaster
 *
 * serializes the data to send back to the master
 * over ethercat and puts all the data in its corresponding
 * index within the frame to send back.
 *
 * @param athena: a pointer to the athena structure,
 * the data to send is serialized from this structure
 */
void loadDataForMaster(AthenaLowLevel* athena)
{
    athena->signalToMaster = athena->signalFromMaster;
    TivaToMaster.Byte[SIGNAL_INDEX] = (uint8_t)athena->signalToMaster;

    // Notice how the signals determine how the data gets serialized
    if(athena->signalFromMaster == LOCATION_DEBUG_SIGNAL && athena->location == athena->masterLocationGuess)
        TivaToMaster.Byte[MASTER_LOCATION_GUESS] = (uint8_t)(athena->location);
    if(athena->signalFromMaster == CONTROL_SIGNAL)
    {

        FloatByteData tempConversion;

        // Package force sensor 0 Newton value
        tempConversion.floatData = athena->actuator0.forceSensor.newtons;
        TivaToMaster.Byte[FORCE0_B1] = tempConversion.Byte[3];
        TivaToMaster.Byte[FORCE0_B2] = tempConversion.Byte[2];
        TivaToMaster.Byte[FORCE0_B3] = tempConversion.Byte[1];
        TivaToMaster.Byte[FORCE0_B4] = tempConversion.Byte[0];

        // Package force sensor 1 Newton value
        tempConversion.floatData = athena->actuator1.forceSensor.newtons;
        TivaToMaster.Byte[FORCE1_B1] = tempConversion.Byte[3];
        TivaToMaster.Byte[FORCE1_B2] = tempConversion.Byte[2];
        TivaToMaster.Byte[FORCE1_B3] = tempConversion.Byte[1];
        TivaToMaster.Byte[FORCE1_B4] = tempConversion.Byte[0];

        // Package encoder 0 radian value
        tempConversion.floatData = athena->joint0.encoder.angleRads;
        TivaToMaster.Byte[ENCODER0_B1] = tempConversion.Byte[3];
        TivaToMaster.Byte[ENCODER0_B2] = tempConversion.Byte[2];
        TivaToMaster.Byte[ENCODER0_B3] = tempConversion.Byte[1];
        TivaToMaster.Byte[ENCODER0_B4] = tempConversion.Byte[0];

        // Package encoder 1 radian value
        tempConversion.floatData = athena->joint1.encoder.angleRads;
        TivaToMaster.Byte[ENCODER1_B1] = tempConversion.Byte[3];
        TivaToMaster.Byte[ENCODER1_B2] = tempConversion.Byte[2];
        TivaToMaster.Byte[ENCODER1_B3] = tempConversion.Byte[1];
        TivaToMaster.Byte[ENCODER1_B4] = tempConversion.Byte[0];
    }
    TivaToMaster.Byte[PROCESS_ID_INDEX] = athena->processIdFromMaster;
}

/**
 * storeDataFromMaster
 *
 * Deserializes the data received from the master
 * and stores the data accordingly.
 *
 * @param athena: a pointer to the athena structure,
 * all of the data from the master gets stored in this
 * structure
 */
void storeDataFromMaster(AthenaLowLevel* athena)
{
    athena->signalFromMaster = MasterToTiva.Byte[SIGNAL_INDEX];
    // notice how the different control signals effect
    // how the data gets stored
    if (athena->signalFromMaster == CONTROL_SIGNAL)
    {
        FloatByteData tempConversion;

        // Set joint 0 direction
        athena->actuator0.direction = MasterToTiva.Byte[DIRECTION0];

        // Set joint 0 duty cycle
        tempConversion.Byte[3] = MasterToTiva.Byte[DUTYCYCLE0_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[DUTYCYCLE0_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[DUTYCYCLE0_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[DUTYCYCLE0_B4];
        athena->actuator0.dutyCycle = tempConversion.floatData;

        // Set joint 1 direction
        athena->actuator1.direction = MasterToTiva.Byte[DIRECTION1];

        // Set joint 1 duty cycle
        tempConversion.Byte[3] = MasterToTiva.Byte[DUTYCYCLE1_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[DUTYCYCLE1_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[DUTYCYCLE1_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[DUTYCYCLE1_B4];
        athena->actuator1.dutyCycle = tempConversion.floatData;
    }
    else if (athena->signalFromMaster == LOCATION_DEBUG_SIGNAL)
    {
        athena->masterLocationGuess = (TivaLocations)MasterToTiva.Byte[MASTER_LOCATION_GUESS];
    }
    else if (athena->signalFromMaster == MODIFY_FORCES)
    {
        // Set force sensor calibration parameters given by master

        FloatByteData tempConversion;

        // Set joint 0 force sensor offset
        tempConversion.Byte[3] = MasterToTiva.Byte[FORCE_OFFSET0_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[FORCE_OFFSET0_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[FORCE_OFFSET0_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[FORCE_OFFSET0_B4];
        athena->actuator0.forceSensor.offset = tempConversion.floatData;

        // Set joint 0 force sensor slope
        tempConversion.Byte[3] = MasterToTiva.Byte[FORCE_SLOPE0_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[FORCE_SLOPE0_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[FORCE_SLOPE0_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[FORCE_SLOPE0_B4];
        athena->actuator0.forceSensor.slope = tempConversion.floatData;

        // Set joint 1 force sensor offset
        tempConversion.Byte[3] = MasterToTiva.Byte[FORCE_OFFSET1_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[FORCE_OFFSET1_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[FORCE_OFFSET1_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[FORCE_OFFSET1_B4];
        athena->actuator1.forceSensor.offset = tempConversion.floatData;

        // Set joint 1 force sensor slope
        tempConversion.Byte[3] = MasterToTiva.Byte[FORCE_SLOPE1_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[FORCE_SLOPE1_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[FORCE_SLOPE1_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[FORCE_SLOPE1_B4];
        athena->actuator1.forceSensor.slope = tempConversion.floatData;
    }
    else if (athena->signalFromMaster == MODIFY_LIMITS)
    {
        // Set joint limits given by master

        FloatByteData tempConversion;

        // Receive joint 0 upper joint limit in degrees
        tempConversion.Byte[3] = MasterToTiva.Byte[MAX_LIM_RAW0_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[MAX_LIM_RAW0_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[MAX_LIM_RAW0_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[MAX_LIM_RAW0_B4];
        athena->joint0.upperJointLimitRaw = tempConversion.floatData * 65535 / 180; // Convert degrees to raw

        // Receive joint 0 lower joint limit in degrees
        tempConversion.Byte[3] = MasterToTiva.Byte[MIN_LIM_RAW0_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[MIN_LIM_RAW0_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[MIN_LIM_RAW0_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[MIN_LIM_RAW0_B4];
        athena->joint0.lowerJointLimitRaw = tempConversion.floatData * 65535 / 180;

        // Receive joint 1 upper joint limit in degrees
        tempConversion.Byte[3] = MasterToTiva.Byte[MAX_LIM_RAW1_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[MAX_LIM_RAW1_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[MAX_LIM_RAW1_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[MAX_LIM_RAW1_B4];
        athena->joint1.upperJointLimitRaw = tempConversion.floatData * 65535 / 180;

        // Receive joint 1 lower joint limit in degrees
        tempConversion.Byte[3] = MasterToTiva.Byte[MIN_LIM_RAW1_B1];
        tempConversion.Byte[2] = MasterToTiva.Byte[MIN_LIM_RAW1_B2];
        tempConversion.Byte[1] = MasterToTiva.Byte[MIN_LIM_RAW1_B3];
        tempConversion.Byte[0] = MasterToTiva.Byte[MIN_LIM_RAW1_B4];
        athena->joint1.lowerJointLimitRaw = tempConversion.floatData * 65535 / 180;

        // Initialize Joint position to middle of joint limits
        athena->joint0.encoder.raw = (athena->joint0.lowerJointLimitRaw + athena->joint0.upperJointLimitRaw)/2;
        athena->joint1.encoder.raw = (athena->joint1.lowerJointLimitRaw + athena->joint1.upperJointLimitRaw)/2;
    }
    else if(athena->signalFromMaster == INITIALIZATION_SIGNAL)
    {
        storeInitFrame(athena);
        if(athena->initializationData.initializationFrameNumberToReceive == NUMBER_OF_INITIALIZATION_FRAMES)
        {
            PandoraInit(athena);
        }
    }
}

/**
 * checkMotorDisable
 *
 * Checks if the motors should be disabled.
 *
 * @param athena: a pointer to the athena structure
 */
void checkMotorDisable(AthenaLowLevel* athena)
{
    // DO NOT EDIT THIS CODE (please)
    /////
    if(athena->signalFromMaster != CONTROL_SIGNAL)
    {
        // STOP MOTORS
        athena->actuator0.dutyCycle = 0;
        athena->actuator1.dutyCycle = 0;
        SendPWMSignal(&athena->actuator0);
        SendPWMSignal(&athena->actuator1);
    }
    /////
}

/**
 * processDataFromMaster
 *
 * Processes the data received form the master
 * Acts on received master data according to desired TIVA mode
 * Return 1 if estop timer should be run, otherwise return 0.
 *
 * @param athena: a pointer to the athena structure
 */
bool processDataFromMaster(AthenaLowLevel* athena)
{
    int run_estop = false;
    // DO NOT REMOVE THIS LINE
    // Ensures motor PWMs are set to 0 in the case signalFromMaster != CONTROL_SIGNAL but motors are still moving
    checkMotorDisable(athena);

    // Check if we have switched into control mode
    if (athena->prevSignalFromMaster != CONTROL_SIGNAL && athena->signalFromMaster == CONTROL_SIGNAL)
    {
        // Reconfigure port F to use SSI1
        disableCheckLocationLEDs();
        if (!athena->joint1.encoder.enabled)
        {
            enableSSIEncoder(&athena->joint1.encoder);
        }
    }

    // Check if we have switched out of control mode
    if (athena->prevSignalFromMaster == CONTROL_SIGNAL && athena->signalFromMaster != CONTROL_SIGNAL)
    {
        if (athena->joint1.encoder.enabled)
        {
            disableSSIEncoder(&athena->joint1.encoder);
        }
        debugLEDSConfig();
    }



    if (athena->signalFromMaster == NOT_CONNECTED)
    {
        // Waiting for master to connect
        notConnectedLEDS(); // Flash red
    }
    else if (athena->signalFromMaster == HALT_SIGNAL || athena->signalToMaster == HALT_SIGNAL_TM)
    {
        // Stop motors signal received from master
        haltLEDS(); // Solid red
    }
    else if (athena->signalFromMaster == LOCATION_DEBUG_SIGNAL)
    {
        // Check if master's guessed TIVA location is the same as TIVA's actual location
        checkLocationLEDS(athena->masterLocationGuess, athena->location);
        // If guess is wrong, solid yellow
        // If guess is correct, flash blue
    }
    else if (athena->signalFromMaster == IDLE_SIGNAL)
    {
        // Master is active but not sending or receiving anything
        idleLEDS(); // Solid purple
    }
    else if (athena->signalFromMaster == MODIFY_FORCES || athena->signalFromMaster == MODIFY_LIMITS)
    {
        // Master is modifying force sensor parameters or joint limits
        modifyLEDs();
    }
    else if (athena->signalFromMaster == CONTROL_SIGNAL)
    {
        // Send motor PWMs and directions to motor controllers
        SendPWMSignal(&athena->actuator0);
        SendPWMSignal(&athena->actuator1);

        // Run estop interrupt
        run_estop = true;
    }

    athena->prevSignalFromMaster = athena->signalFromMaster;

    return run_estop;
}

/**
 * disableSSI1
 *
 * Disables SSI1 on the Tiva.
 *
 */
void disableSSI1()
{
    // The SSI0 peripheral must be enabled for use.
    SSIDisable(SSI1_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) &= 0XFE; // Enable PF0 AFS
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0; // Relock
}

/**
 * disableCheckLocationLEDs
 *
 * Disables the LEDs used for debugging.
 */
void disableCheckLocationLEDs()
{
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
    SysCtlPeripheralDisable(LED_PERIPH);
}

/**
 * sendSignal
 *
 * Updates motor controllers with new PWM and direction for each motor.
 *
 * @param athena: a pointer to the athena structure
 */
/*
void sendSignal(AthenaLowLevel* athena)
{
    // Send to motor 0 and motor 1
    setPulseWidth(0,20000,athena->joint0.dutyCycle,SysCtlClockGet(), athena->joint0.direction);
    setPulseWidth(1,20000,athena->joint1.dutyCycle,SysCtlClockGet(), athena->joint1.direction);
}
*/
/**
 * updateForces
 *
 * Read load cells and update corresponding joint variables.
 *
 * @param athena: a pointer to the athena structure
 */
/*
void updateForces(AthenaLowLevel* athena)
{
    readLoadCell(&athena->joint0.forceSensor);
    readLoadCell(&athena->joint1.forceSensor);

    // Convert raw force sensor reading to Newtons using known force sensor calibration values
    athena->joint0.forceSensor.newtons = athena->joint0.forceSensor.raw * athena->joint0.forceSensor.slope + athena->joint0.forceSensor.offset;
    athena->joint1.forceSensor.newtons = athena->joint1.forceSensor.raw * athena->joint1.forceSensor.slope + athena->joint1.forceSensor.offset;
}
*/

/**
 * updateJointAngles
 *
 * Read encoders and update corresponding joint variables.
 *
 * @param athena: a pointer to the athena structure
 */
/*
void updateJointAngles(AthenaLowLevel* athena)
{
    readAbsEncoder(&athena->joint0.encoder);
    readAbsEncoder(&athena->joint1.encoder);

    // Convert raw encoder sensor reading to radians
    if (athena->joint0.encoder.encoderBrand == Gurley_Encoder){
        athena->joint0.encoder.angleRads = athena->joint0.encoder.raw * M_PI / 65535;
    }
    else if (athena->joint0.encoder.encoderBrand == Orbis_Encoder){
        athena->joint0.encoder.raw &= 16383;
        athena->joint0.encoder.angleRads = athena->joint0.encoder.raw * ((2 * M_PI) / 16383);
        athena->joint0.encoder.angleDegrees = athena->joint0.encoder.raw * (360 / 16383);
    }
    if (athena->joint1.encoder.encoderBrand == Gurley_Encoder){
        athena->joint1.encoder.angleRads = athena->joint1.encoder.raw * M_PI / 65535;
    }
    else if (athena->joint1.encoder.encoderBrand == Orbis_Encoder){  // joint1
        athena->joint1.encoder.raw &= 16383;
        athena->joint1.encoder.angleRads = athena->joint1.encoder.raw * ((2 * M_PI) / 16383);
        athena->joint1.encoder.angleDegrees = athena->joint1.encoder.raw * (360 / 16383);
    }

}
*/
/**
 * updateMotorPositions
 *
 * Read encoders and update corresponding joint variables.
 *
 * @param athena: a pointer to the athena structure
 */
/*
void updateMotorPositions(AthenaLowLevel* athena)
{
    readMotorPosition(&athena->joint0.motorEncoder);
    readMotorPosition(&athena->joint1.motorEncoder);
}
*/
/**
 * updateMotorVelocities
 *
 * Responsible for populating the motorEncoders for each joint to the most
 * recently calculated quadrature velocity output. The calculation is done at the HAL level within the
 * TivaWare QEI Library.
 *
 * @param athena pointer to the athena structure
 * @param sample_rate how quickly is the quadrature encoder checked to calculate velocity
 * @param countsPerRotation the number of quadrature encoder counts within a 360 degree rotation
 */
/*
void updateMotorVelocities(AthenaLowLevel* athena, int32_t sample_rate, int32_t countsPerRotation)
{
    readMotorVelocity(&athena->actuator0.motorEncoder, sample_rate, countsPerRotation);
    readMotorVelocity(&athena->actuator1.motorEncoder, sample_rate, countsPerRotation);
}*/

