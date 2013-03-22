//todo:
//    endurance power limit w/ dial
//    compensate for engine torque curve in autocross mode: map power to function
//    boost button: overrides any mode and puts motor => 100%, esp to help w/ launch control.  and, motor still runs even when clutch is enabled
//    refactor to run based on dashboard button values rather than "modes"

/*

### FSAE PROGRAM FOR ARDUINO ###

--------ABOUT-------------------------------------------------------------------

by Jan Kolmas, Geoffrey Litt, Stephen Hall, Alex Carrillo, and others...

Written for the Yale FSAE formula racing team.

The code is designed to run on an Arduino Mega 2560.
It is written in Arduino programming language.

It is designed ro run a hybrid car with an electric motor and a gas engine
capable of regen.

Basic description:
The purpose of the code is to take data from sensors, evaluate it and input
power to the motor and the engine. It also handles some of the dashboard
indicators and telemetry communication. 

The Arduino program has two modes:

Autocross: Fast as possible. Engine and motor both powered to max
Endurance: Save energy. Limit engine and motor to certain % power, based on
					 value of the endurance dial input

There is also a boost button that delivers max motor power in any scenario.

--------CODE STRUCTURE----------------------------------------------------------

1. DEFINITIONS

1.1 Pin number nicknames
1.2 Critical and limit values definitions
1.3 Communication IDs
1.4 Other definitions

2. INITIALIZATION

2.1 Variable initialization
2.2 Communication initialization
2.3 Servo initialization 

3. FUNCTIONS

3.1 Main program body functions
3.2 Communication functions
3.3 Debugging functions
3.4 Other functions  

4. EXECUTION

4.1 Setup function (executed once)
4.1 Loop Function  (executed repeatedly)

*/

#include <Servo.h>    //Give access to the Arduino Servo library

//---------------------------------------------------------------------------------------------
// 1. DEFINITIONS
//------------------------------------------------------------------------------
//{

//------------------------------------------------------------------------------
// 1.1 Pin nicknames
//------------------------------------------------------------------------------
//{

//input pins (sensors)
//analog input pins

#define rpmPin             A0 //RPM 
#define throttlePin        A4 //throttle amount - from the pedal
#define radiatorTempPin    A1 //Temperature of the radiator 
#define fuelPin            A3 //fuel level sensor
#define enduranceDialPin   A6 //sets endurance power limit

//digital input pins  

//sensors
#define hiVoltageLoBattPin 22 //HIGH when the High voltage battery is critical
#define BMSFaultPin        23 //HIGH when there is a problem with the BMS (critical temperature for example)
#define clutchPin          24 //HIGH when the clutch is pressed
#define brakePin           26 //HIGH when the brake is pressed
#define reedPin            29 //HIGH when reed switch is on (magnet adjacent to sensor)

//dashboard buttons
#define boostPin           25 //HIGH when the boost button is pressed
#define engineEnableInPin  27 //HIGH when the gas engine enable button is pressed
#define hvEnableInPin      28 //HIGH when the electric motor enable button is pressed
#define endurancePin       39 //HIGH when the endurance mode enable button is pressed

//output pins
//PWM output pins

#define servoPin           2 //PWM output to servo handled by the Arduino Servo library
#define kellyPin           3 //PWM output to kelly
#define energyLevelPin     5 //PWM output to energy level LED bar

//digital output pins

#define powerIndicatorPin  51 //LED on the panel, which is on when the code is running
#define criticalPin        33 //LED on the panel, tells if something is wrong
#define engineEnableOutPin 36 //Connected to a relay, needs to be HIGH in order to accelerate
#define hvEnableOutPin     37 //Is HIGH when the high voltage system is supposed to be on
#define moduleSleepPin     38 //Pauses the onboard telemetry module

#define sevenSeg0Pin       44 // velocity sevenseg output bit 0
#define sevenSeg1Pin       45 // velocity sevenseg output bit 1
#define sevenSeg2Pin       46 // velocity sevenseg output bit 2
#define sevenSeg3Pin       47 // velocity sevenseg output bit 3
#define sevenSeg4Pin       40 // velocity sevenseg output bit 4
#define sevenSeg5Pin       41 // velocity sevenseg output bit 5
#define sevenSeg6Pin       42 // velocity sevenseg output bit 6
#define sevenSeg7Pin       43 // velocity sevenseg output bit 7

//}
//------------------------------------------------------------------------------
// 1.2 Critical  and limit values definitions
//------------------------------------------------------------------------------
//{

//If any of the variables reaches a critical value, the car will stop all
//motor and engine input until the values drop below limit values. Limit
//values were introduced to prevent oscillations.

const int CRITICAL_RPM =    4000;    //In revolutions per minute !adjust
const int CRITICAL_VELOCITY = 45;    //In miles per hour !adjust 
const int CRITICAL_BATT =     75;    //In volts !adjust
const int CRITICAL_FUEL =     10;    //In percent !adjust
const int CRITICAL_TEMP =    230;    //In degrees Fahrenheit !adjust

const int LIMIT_RPM =    3500;       //In revolutions per minute !adjust
const int LIMIT_VELOCITY = 35;       //In miles per hour !adjust
const int LIMIT_TEMP =    210;       //In degrees Fahrenheit !adjust


//}
//------------------------------------------------------------------------------
// 1.3 Communication IDs
//------------------------------------------------------------------------------
//{

//Are sent along with respective values through telemetry to a program that will
//understand them. The variables are named according to Apple standards, because
//the telemetry programs runs on a Mac

const int kTelemetryDataTypeNone =                         -1;
const int kTelemetryDataTypeSpeed =                         0;
const int kTelemetryDataTypeRadiatorTemperature =           1;
const int kTelemetryDataTypeBMSFault =                      2;
const int kTelemetryDataTypeEngineRPM =                     3;
const int kTelemetryDataTypeMotorRPM =                      4;
const int kTelemetryDataTypeClutchPedal =                   5;
const int kTelemetryDataTypeBrakePedal =                    6;
const int kTelemetryDataTypeGasPedal =                      7;
const int kTelemetryDataTypeGear =                          8;
const int kTelemetryDataTypeCritical =                      9;
const int kTelemetryDataTypeHighVoltageBatteryLevel =      10;
const int kTelemetryDataTypeFuelLevel =                    11;
const int kTelemetryDataTypeDemoSin =                      12;
const int kTelemetryDataTypeDemoSpecial =                  13;
const int kTelemetryDataTypeMAX =                          14;

const int kTelemetryDataCommandSetCarEnableState =         15;

//}
//------------------------------------------------------------------------------
// 1.4 Other definitions
//------------------------------------------------------------------------------
//{
const int AUTOCROSS_MODE = 1;        //Shortcuts for the mode selector
const int ENDURANCE_MODE = 2;

const int FULL_PWM =      255;           //Maximum PWM output 
const int ENDURANCE_ASSIST = 0;
const int AUTOCROSS_ASSIST = FULL_PWM;

const int LOWER_EFFICIENCY_LEVEL = 1000; //In rounds per minute, used in endurance mode !adjust
const int UPPER_EFFICIENCY_LEVEL = 3000; //!adjust

const int RPM_SCALE_MAX =         4000; //!adjust
const int VELOCITY_SCALE_MAX =      50; //!adjust
const int RADIATORTEMP_SCALE_MIN = 180; //In degrees Fahrenheit !adjust
const int RADIATORTEMP_SCALE_MAX = 300; //In degrees Fahrenheit !adjust

const int THROTTLE_SCALE_MIN = 555;  //Boundary values that the throttle pot sends,
//the effective path of the throttle pedal
const int THROTTLE_SCALE_MAX = 602;

const int SHORT_COMM_INTERVAL = 50;  // for high frequency data in ms !adjust
const int LONG_COMM_INTERVAL = 1000; //for low frequency data in ms !adjust

const int SERVO_MIN =       900;     // pulse width range for the servo in ms for the HS-805bb currently used
const int SERVO_MAX =      2100;
const int SERVO_MIN_ANGLE =   0;     //Limits of throttle servo output angle. !adjust  
const int SERVO_MAX_ANGLE = 160;

const float WHEEL_CIRCUMFERENCE = 66; // in inches
const float VELOCITY_SCALAR = 56.82;  //This converts from feet/ms to mph
//}
//}
//---------------------------------------------------------------------------------------------

// 2. INITIALIZATION

//---------------------------------------------------------------------------------------------
//{
//---------------------------------------------------------------------------------------------
// 2.1 Variable initialization
//---------------------------------------------------------------------------------------------
//{

//These values are not the inputs, but are calculated from the analog data to
//have the right scale and units
int rpm =           0;               //In revolutions per minute
int velocity =      0;               //In mph.
int mode =          AUTOCROSS_MODE;  //Autocross mode by default
int fuel =          0;               //In percent
int throttle =      0;               //For the servo, scaled from 0 to 180 (degrees)
int throttleKelly = 0;               //For the kelly, scaled from 0 to 255 (PWM)
int radiatorTemp =  0;               //In degrees Fahrenheit
int enduranceLimit = 0;              //endurance dial limit, scaled from 0 to 100

//Logical binary variables
//Inputs
boolean hiVoltageLoBatt =     false; //Is true if the battery level is low
boolean BMSFault =            false; //Is true if there is a problem with the Lithium-Ions. 
boolean clutchPressed =       false; //Is true if the clutch is pressed
boolean boost =              false; //Is true if the assist/boost button is pressed
boolean brake =               false; //Is true if the brake pedal is pressed
boolean servoEnable =         false; //Is true if the servoEnable switch is on
boolean kellyEnable =         false; //Is true if the kellyEnable switch is on
boolean engineEnable =        false; //Is true if engine enable switch is on
boolean hvEnable =            false; //Is true if HV enable switch is on
boolean modeEndurance =       false; //Is true if the mode selector is on Endurance
boolean modeElectric =        false; //Is true if the mode selector is on Electric
boolean telemetryEnable =     false; //Telemetry on/off switch

//Outputs
boolean moduleSleep =         false; //True value sets the telemetry module asleep

boolean reedOffPrevious =     false; //Is true when the reed was HIGH (not at the magnet) in the previous loop,
//so velocity calculation will occur immediately
//after it is turned HIGH
boolean assisting          =  false;

//analog variables are in the scope of 0-1023, as read from the sensors.
int rpmAnalog =          0;        
int fuelAnalog =         0;
int throttleAnalog =     0;
int battTempAnalog =     0;
int radiatorTempAnalog = 0;
int enduranceDialAnalog = 0;

//These variables are sent to the servo and kelly
int servoOut =           0;
int kellyOut =           0;

//Logical switches controlling the program flow
boolean virtualBigRedButton = false;   //Received via serial
boolean criticalCycle =       false;   //Tells if one of the security limits has overflown
boolean endloop =             false;   //Goes to end of runTheCar.

//Time variables controlling the telemetry
unsigned long currentTime =            0;   //Timestamp corresponding to the start of te loop
unsigned long previousShortTime =      0;   //Time in ms when last short time transmission occured
unsigned long previousLongTime =       0;   //Time in ms when last long time transmission occured
unsigned long previousVelocityTime =   0;   //Last time when velocity was measured in ms

//}
//---------------------------------------------------------------------------------------------
// 2.2 Communication initialization
//---------------------------------------------------------------------------------------------
//{

const int MAX_SEND_LENGTH = 64;    //Set a maximum packet size just to define the buffer array.
char writeBuffer[MAX_SEND_LENGTH]; //Write buffer of chars to be sent to a serial port.
//Though it has space up to MAX_SEND_LENGTH,
//right now we only add about 10-20 values to the array and send those.

int writeIndex = 0;                //The current array index new data will be written to. This increments until data is written.

char readBuffer[MAX_SEND_LENGTH];
char readBufferIndex = 0;
int  processingSerialBuffer = 0;
char sendID[3];
char sendValue[2];
int  storeIndex = 0; //0 is for ID, 1 is for value
int  storeVariable = 0;

//}
//---------------------------------------------------------------------------------------------
// 2.3 Servo initialization
//---------------------------------------------------------------------------------------------
//{


Servo throttleServo;  //This is the instance of our servo


//}
//}
//---------------------------------------------------------------------------------------------

// 3 FUNCTIONS

//---------------------------------------------------------------------------------------------
//{

//---------------------------------------------------------------------------------------------
// 3.1 Main program body functions
//---------------------------------------------------------------------------------------------
//{

void readInputs(){
    //analog pins

    rpmAnalog =          analogRead(rpmPin);           
    fuelAnalog =         analogRead(fuelPin); 
    throttleAnalog =     analogRead(throttlePin);
    radiatorTempAnalog = analogRead(radiatorTempPin);
    enduranceDialAnalog = analogRead(enduranceDialPin);

    //digital pins
    //Most of the variables are set true when pins are driven LOW. Refer to Ports_2011 on Google Docs
    hiVoltageLoBatt =   (digitalRead(hiVoltageLoBattPin) == LOW);
    BMSFault =          (digitalRead(BMSFaultPin) ==        LOW);
    clutchPressed =     (digitalRead(clutchPin) ==          LOW);
    boost =             (digitalRead(boostPin) ==          HIGH);
    servoEnable =       (digitalRead(servoEnablePin) ==     LOW);
    kellyEnable =       (digitalRead(kellyEnablePin) ==     LOW);
    engineEnable =      (digitalRead(engineEnableInPin)) == LOW);
    hvEnable =          (digitalRead(hvEnableInPin)) ==     LOW);

    if(hvEnable==true) brake =(digitalRead(brakePin) ==     LOW);  //the brake is only checked if HV is enabled,
    else brake = false;                                                     //otherwise set to false

    modeEndurance =     (digitalRead(modeEndurancePin) ==   LOW);
    telemetryEnable =   (digitalRead(telemetryEnablePin) == LOW);

}


void processInputs(){

    //transform the analog 0-1023 data to actual values    
    rpm       =    map(rpmAnalog,         0,1023, 0,RPM_SCALE_MAX);         //in Rounds Per Minute (RPM)          
    radiatorTemp = map(radiatorTempAnalog,0,1023, RADIATORTEMP_SCALE_MAX, 0);//in degrees F  (yes this is correct, increased temp -> lower signal)
    fuel      =    map(fuelAnalog,        0,1023, 0,100);                   //in percent
    enduranceLimit = map(enduranceDialAnalog, 0, 1023, 0, 100); 

    //endurance mode button determines mode
    if(modeEndurance == false)      mode = AUTOCROSS_MODE;
    else if (modeEndurance == true) mode = ENDURANCE_MODE;

    //constrain throttle analog value between calibrated values
    throttleAnalog = constrain(throttleAnalog, THROTTLE_SCALE_MIN, THROTTLE_SCALE_MAX);

    //map throttle analog to servo and Kelly-PWM appropriate values
    throttle = map(throttleAnalog,THROTTLE_SCALE_MIN,THROTTLE_SCALE_MAX,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE); 
    throttleKelly = map(throttleAnalog,THROTTLE_SCALE_MIN,THROTTLE_SCALE_MAX,0,FULL_PWM);

    //Calculation of velocity from the reed switch on the wheel
    if (digitalRead(reedPin) == LOW)
    {
        if (reedOffPrevious == true)
        {
            velocity = (WHEEL_CIRCUMFERENCE / (currentTime - previousVelocityTime)) * VELOCITY_SCALAR; 
            previousVelocityTime = currentTime;
            reedOffPrevious = false;
        }

        else if (currentTime - previousVelocityTime > 300) velocity = 0;

    }

    else {
        reedOffPrevious = true;
        if (currentTime - previousVelocityTime > 2000) velocity = 0;    
    } 

}



void runSecurityBlock(){
    //Security block is executed in ever loop and if a variable reaches a critical
    //limit, the servo and kelly outputs are both set to 0, until the conditions are
    //under the limit again.


    if (//rpm > CRITICAL_RPM ||  Disclaimer: this car has been made less safe by Sid, don't blame Jan and Geoffrey if things go wrong ;) 
        //velocity > CRITICAL_VELOCITY || 
        // radiatorTemp > CRITICAL_TEMP ||
        BMSFault ==            true )
            // || virtualBigRedButton == true

        {
            criticalCycle = true;
            endloop = true;
            digitalWrite(criticalPin,HIGH);
        }

    if (criticalCycle == true){
        kill();
        endloop = true;
        digitalWrite(criticalPin,HIGH);
        //set servo and kelly output to zero

        if (//rpm <                  LIMIT_RPM &&
                //velocity <             LIMIT_VELOCITY && 
                //radiatorTemp <         LIMIT_TEMP &&
                BMSFault ==            false )
            // && virtualBigRedButton == false

            //Checks if the conditions are OK again
            //the reason for separate CRITICAL and LIMIT values is
            //to prevent oscillations from critical cycle to normal and back
        {criticalCycle = false;
            endloop = false;
            digitalWrite(criticalPin,LOW);}       

            //if the conditions are still critical, do not execute the main program body         
    }
}


void runCommunication(){
    //Important data are sent at a shorter interval than less important data, which is
    //the reason for high and low frequency communication.
    //It is not frequency of the radio, but of the sending period.

    digitalWrite(moduleSleepPin,HIGH);

    //SENDING

    //High frequency communication
    if(currentTime - previousShortTime > SHORT_COMM_INTERVAL)
    {
        previousShortTime = currentTime;     //Reset time

        serialWriteBegin();                  //Initiate serial sending
        serialWriteValue(     velocity,      kTelemetryDataTypeSpeed);
        serialWriteValue(     rpm,           kTelemetryDataTypeEngineRPM);
        serialWriteValue((int)clutchPressed, kTelemetryDataTypeClutchPedal);
        serialWriteValue((int)brake,         kTelemetryDataTypeBrakePedal);
        serialWriteValue(     throttle,      kTelemetryDataTypeGasPedal);

        //Low frequency communication
        if(currentTime - previousLongTime > LONG_COMM_INTERVAL)
        {
            previousLongTime = currentTime;

            serialWriteValue(     radiatorTemp,      kTelemetryDataTypeRadiatorTemperature);
            serialWriteValue((int)hiVoltageLoBatt,   kTelemetryDataTypeHighVoltageBatteryLevel);
            serialWriteValue((int)criticalCycle,     kTelemetryDataTypeCritical);
            serialWriteValue((int)BMSFault,          kTelemetryDataTypeBMSFault);
            serialWriteValue(     mode,              kTelemetryDataTypeDemoSpecial);
        }

        //actually send the data
        serialWriteCommit(1); // 0 for USB, 1 for tranceiver
        serialWriteCommit(0);
    }

    //RECEIVING
    serialPortReadInBackgroundToBuffer(1);
}


void runTheCar(){

    float enduranceMultiplier = enduranceLimit / 100.0;

    //---------------------------------------------------------------------------------------------
    // runTheCar MODES
    //---------------------------------------------------------------------------------------------
    //{
    
    // ===================
    // MAIN MODE LOGIC
    // ===================

    // Autocross mode logic
    if (mode == AUTOCROSS_MODE && endloop == false)
    {
        servoOut = throttle;
        kellyOut = throttleKelly;
    }

    // Endurance mode logic
    else if (mode == ENDURANCE_MODE && endloop == false)
    {       
        servoOut = throttle * enduranceMultiplier;
        kellyOut = throttleKelly * enduranceMultiplier;
                   
    }

    // ===================
    // OVERRIDES
    // ===================

    // These override the basic output logic based on various inputs.
    // The order in which the overrides are applied is very important! Be careful switching order.

    // Clutch override
    // if clutch pressed, don't drive the motor
    if (clutchPressed){
        kellyOut = 0;
    }

    // Boost overrides
    // if boost button pressed, drive max motor, and scale throttle to max val
    // note: this overrides the clutch override, must come afterwards!
    if (boost){
        servoOut = throttle;
        kellyOut = FULL_PWM;
    }

    // Braking overrides
    // if braking, don't drive engine or motor
    // this override should come LAST in the override order for safety reasons
    if (brake == true){
        kellyOut = 0;
        servoOut = 0;
    }

    //}
    //---------------------------------------------------------------------------------------------
    // runTheCar OUTPUT
    //---------------------------------------------------------------------------------------------
    //{

    //send velocity to seven seg display
    sevenSegOut();

    //Turn the engine relay on if engine enable switch is on
    if (engineEnable == true) {digitalWrite(engineEnableOutPin, HIGH);}
    else                      {digitalWrite(engineEnableOutPin, LOW);}

    //Turn HV on if HV enable switch is on
    if (hvEnable == true) {digitalWrite(hvEnableOutPin, HIGH);}
    else                  {digitalWrite(hvEnableOutPin, LOW);}

    //Send output to servo
    if(servoEnable==true&&engineOn==true)  throttleServo.write(servoOut);
    else throttleServo.write(SERVO_MIN_ANGLE); //Reset the servo if servoEnable is false

    //Send output to kelly
    if(kellyEnable==true && hvEnable==true && hiVoltageLoBatt == false){                 
        analogWrite(kellyPin,kellyOut);
    }
    else {
        analogWrite(kellyPin,0);
    }

    //}
}


//}
//---------------------------------------------------------------------------------------------
// 3.2 Communication functions 
//---------------------------------------------------------------------------------------------
//{

// 3.2.1 Sending functions

//Call this method to initiate a new serial write operation. 
void serialWriteBegin() {
    writeIndex = 0;        //Reset the write index, and add the initial characters for the protocol.
    writeBuffer[0] = '<';
    writeIndex++;
}

//Add a value to the buffer. 
//Format of the buffer string is <ID=value,ID=value>, like <0=23,1=100,2=0>
void serialWriteValue(int value,int ID) {
    if (writeIndex>1) {
        writeBuffer[writeIndex] = ',';//Here we add a comma to separate the entries
        writeIndex++;
    }

    char c[10];//Use this array to convert the value and IDs into individual chars.
    itoa(ID,c,10);//This function does the integer to string conversion. 
    writeBuffer[writeIndex] = c[0];//Write the ID, which corresponds to the data type
    writeIndex++;

    //If the ID has two digits, move the index one place forward
    if (ID>9) {
        writeBuffer[writeIndex] = c[1];
        writeIndex++;
    }
    //Write the '=' to the buffer par the protocol
    writeBuffer[writeIndex] = '=';
    writeIndex++;

    //We convert the value from an integer to a string, then write that to the buffer.
    //Note that c[]is overwritten here
    itoa(value,c,10);
    writeBuffer[writeIndex]= c[0];
    writeIndex++;
    if (value>9) {//Move the index acccordingly to the number of digits
        writeBuffer[writeIndex]= c[1];
        writeIndex++;
    }
    if (value>99) {
        writeBuffer[writeIndex]= c[2];
        writeIndex++;
    }
    if (value>999) {
        writeBuffer[writeIndex]= c[3];
        writeIndex++;
    }
}

//This function does the actual sending. It writes the buffer to the given serial port
//up until write index (after adding in the final protocol characters). 
//You can pass in which serial port to send over.
void serialWriteCommit(int serial) {
    writeBuffer[writeIndex] = '>';//Close the buffer string
    writeIndex++;
    int i=0;

    for (i;i<writeIndex;i++) {
        if      (serial == 0) Serial.write(  writeBuffer[i]);
        else if (serial == 1) Serial1.write( writeBuffer[i]);
    }

    if      (serial == 0) Serial.write(  '\n');//Indicate that sending is over
    else if (serial == 1) Serial1.write( '\n');

}

// 3.2.2. Receiving functions

//Set the global variables here
int telemetrySendSetGlobal(int id, int val) {
    switch(id) {
        case kTelemetryDataCommandSetCarEnableState:
            if(val==1 || virtualBigRedButton == true) virtualBigRedButton = true;
            else virtualBigRedButton = false;
            break;
            //...
            //...
        default:
            break; 
    }
}

void processSerialBuffer() { //Processes a string in the read buffer and then clears the buffer
    processingSerialBuffer = 1;

    if (readBuffer[0] == '<') {
        //First byte is okay. Let's try and read a command
        for (int i=1;i<MAX_SEND_LENGTH;i++) {
            //Serial.write(".");Serial.write(i);Serial.write(".");
            if (readBuffer[i] == '=') {
                //Serial.print("Found=\n");
                storeIndex=0;
                storeVariable = 1;
                sendID[2]='\o';
                continue;
            } else if (readBuffer[i] == ',' | readBuffer[i]=='>') {
                //Serial.print("Foundbreak\n");
                int currentID = atoi(sendID);
                int currentValue = atoi(sendValue);
                telemetrySendSetGlobal(currentID,currentValue);
                storeIndex=0;
                storeVariable = 0;
                sendID[0] = sendID[1] = 0;
                sendValue[0]=sendValue[1]=0;
                if (readBuffer[i]=='>') break;
                continue;
            } else {
                // Store an id or value
                if (storeVariable == 0) {//store ID
                    //Serial.print("StoringID:");
                    //Serial.write(readBuffer[i]);
                    sendID[storeIndex] = readBuffer[i];
                    storeIndex++;
                } else {//store value
                    //Serial.print("StoringValue:");
                    //Serial.write(readBuffer[i]);
                    sendValue[storeIndex] = readBuffer[i];
                    storeIndex++;
                }
            }
        }
    }
    //Clear the array
    for (int i=0;i<readBufferIndex;i++) {
        readBuffer[i] = ' ';
    }
    readBufferIndex = 0;
    processingSerialBuffer = 0;
}


//Call this in the background to read from the serial port. Pass in the serial port number to read from that port.
void serialPortReadInBackgroundToBuffer(int port) {
    //If there's no serial available, do nothing in this function
    int serialAvailable = 0;
    if (port==0) serialAvailable = Serial.available() > 0;
    if (port==1) serialAvailable = Serial1.available() > 0;
    if (serialAvailable>0 && processingSerialBuffer == 0) {

        int newByte;
        if (port==0) {
            newByte = Serial.read();
        } else if (port == 1) {
            newByte = Serial1.read(); 
        } else {
            return; 
        }

        readBuffer[readBufferIndex] = newByte;
        readBufferIndex++;
        if (newByte == '>') {
            //This is the end byte of the communication protocol. We should have a complete string in the buffer to parse.
            processSerialBuffer(); 
        }
    }

}

void sevenSegOut() {  // takes velocity and outputs appropriate binary signals

    int tensDigit = (velocity - (velocity % 10))/10;
    int onesDigit = velocity % 10;


    if(tensDigit % 2 == 1) {digitalWrite(sevenSeg3Pin, HIGH);}
    else {digitalWrite(sevenSeg3Pin, LOW);}

    if(tensDigit % 4 >= 2) {digitalWrite(sevenSeg2Pin, HIGH);}
    else {digitalWrite(sevenSeg2Pin, LOW);}

    if(tensDigit % 8 >= 4) {digitalWrite(sevenSeg1Pin, HIGH);}
    else {digitalWrite(sevenSeg1Pin, LOW);}

    if(tensDigit >= 8) {digitalWrite(sevenSeg0Pin, HIGH);}
    else {digitalWrite(sevenSeg0Pin, LOW);}

    if(onesDigit % 2 >= 1) {digitalWrite(sevenSeg7Pin, HIGH);}
    else {digitalWrite(sevenSeg7Pin, LOW);}

    if(onesDigit % 4 >= 2) {digitalWrite(sevenSeg6Pin, HIGH);}
    else {digitalWrite(sevenSeg6Pin, LOW);}

    if(onesDigit % 8 >= 4) {digitalWrite(sevenSeg5Pin, HIGH);}
    else {digitalWrite(sevenSeg5Pin, LOW);}

    if(onesDigit >= 8) {digitalWrite(sevenSeg4Pin, HIGH);}
    else {digitalWrite(sevenSeg4Pin, LOW);}

}


//}
//---------------------------------------------------------------------------------------------
// 3.3 Debugging functions
//---------------------------------------------------------------------------------------------
//{

void testTheCar(){

    servoOut = throttle;
    kellyOut = throttleKelly;

    digitalWrite(engineEnableOutPin, HIGH);

    if (mode == ELECTRIC_MODE) digitalWrite(hvEnableOutPin, HIGH);  //Allow High Voltage to be ON for testing if required
    else digitalWrite(hvEnableOutPin, LOW);                         //Might need it for programming BMS/Kelly

    if (brake == true)
    {
        servoOut = SERVO_MIN_ANGLE;
    }
    if(servoEnable==true)  {
        throttleServo.write(servoOut);
        analogWrite(kellyPin,0);
    }    //If Servo Enable is ON, then use servo
    else{
        throttleServo.write(SERVO_MIN_ANGLE);                   //Otherwise reset the Servo
        analogWrite(kellyPin,kellyOut);
    }

}


//Overloaded debug() function: 4 different ones to handle 4 input types
//Delay can be added to be able to freeze the debugging stream for a while
void debug(String outstring){
    Serial.println(outstring);
    delay(0);
}

void debug(float outfloat){
    Serial.println(outfloat);
    delay(0);
}

void debug(boolean outboolean){
    if (outboolean == true)  Serial.println("true");
    else                     Serial.println("false");
    delay(0);

}

void debug(int outint){
    Serial.println(outint);
    delay(0);
}

//Custom debugging delay function exist to be able to find it using find "debug"
//No delay() functions are supposed to be in the program flow
void debugDelay(int ms) {delay(ms);}

//}
//---------------------------------------------------------------------------------------------
// 3.4 Other functions
//---------------------------------------------------------------------------------------------
//{

//steals power from motor and engine, used to stop the car when
//Virtual Big Red Button is activated or in critical cycle
void kill()
{
    digitalWrite(engineEnableOutPin, LOW);
    digitalWrite(hvEnableOutPin, HIGH);
    servoOut = SERVO_MIN_ANGLE;
    kellyOut = 0;
    throttleServo.write(servoOut);
    digitalWrite(kellyPin,kellyOut);
}

//Creates various kill scenarios, 4 types, occur timeInSeconds after program initiation
//Used for testing, add after the processInputs() function
void doScenario(int type, int timeInSeconds)
{
    unsigned long time = timeInSeconds * 1000;
    if (millis() > time)
    {
        switch (type) {
            case 1: //Virtual big red button is pressed
                virtualBigRedButton = true;
                break;

            case 2: //Rpm is maxed out
                rpm = CRITICAL_RPM + 1;
                break;

            case 3: //Velocity is maxed out
                velocity = CRITICAL_VELOCITY + 1;
                break;

            case 4: //Temperature is maxed out
                radiatorTemp = CRITICAL_TEMP + 1;
                break;
        }
    }
}


//}
//}
//---------------------------------------------------------------------------------------------

// 4. EXECUTION

//---------------------------------------------------------------------------------------------
//{

//---------------------------------------------------------------------------------------------
// 4.1 Setup function (executed once)
//---------------------------------------------------------------------------------------------
//{

//Digital pins are set to input or output, serial communication initialized
void setup()
{


    //Input pins setup
    pinMode(hiVoltageLoBattPin,INPUT);
    pinMode(BMSFaultPin,       INPUT);
    pinMode(clutchPin,         INPUT);
    pinMode(assistPin,         INPUT);
    pinMode(brakePin,          INPUT);
    pinMode(servoEnablePin,    INPUT);
    pinMode(kellyEnablePin,    INPUT);
    pinMode(reedPin,           INPUT);
    pinMode(telemetryEnablePin,INPUT);

    //Output pins setup (set some to low to begin, for safety)
    pinMode(powerIndicatorPin,   OUTPUT);

    pinMode(criticalPin,       OUTPUT);
    digitalWrite(criticalPin, LOW);

    pinMode(engineEnableOutPin,     OUTPUT);
    digitalWrite(engineEnableOutPin, LOW);

    pinMode(hvEnableOutPin,OUTPUT);
    digitalWrite(hvEnableOutPin, LOW);

    pinMode(moduleSleepPin,    OUTPUT);

    pinMode(sevenSeg0Pin, OUTPUT);
    pinMode(sevenSeg1Pin, OUTPUT);
    pinMode(sevenSeg2Pin, OUTPUT);
    pinMode(sevenSeg3Pin, OUTPUT);
    pinMode(sevenSeg4Pin, OUTPUT);
    pinMode(sevenSeg5Pin, OUTPUT);
    pinMode(sevenSeg6Pin, OUTPUT);
    pinMode(sevenSeg7Pin, OUTPUT);




    //Initialize serial communications at 9600 bps:
    Serial.begin(9600);   //Serial - first serial port of the Arduino board connected to
    //the built in serial to USB converter. Calls to this serial port 
    //will communicate over USB to a computer.
    Serial1.begin(9600);  //Serial1 - second serial port of the Arduino board connected to 
    //the RF transceiver. Data sent to this port will be added to the transceiver's queue. 


    //Setup the servo (set to min angle to begin)
    throttleServo.attach(servoPin, SERVO_MIN, SERVO_MAX);
    throttleServo.write(SERVO_MIN_ANGLE);



}//end of setup() function

//}
//---------------------------------------------------------------------------------------------
// 4.2 Loop function (executed repeatedly)
//---------------------------------------------------------------------------------------------
//{

void loop()
{
    digitalWrite(powerIndicatorPin, HIGH);

    currentTime = millis(); //Time is reset at the beginning of the loop because various procedures
    //like velocity measuring and telemetry timing use it
    endloop = false; // resets "end loop" condition

    //Read Inputs
    readInputs();

    //Process Inputs
    processInputs();

    //Run Communication
    if(telemetryEnable == true) {runCommunication();}
    else {digitalWrite(moduleSleepPin,LOW);}  //If not communicating, set the module asleep

    //Run Security Block
    runSecurityBlock();


    //Modes, servo and kelly output commands
    if(endloop == false){
        runTheCar();
    }


    //testTheCar();





}//End of loop() function

//}
//}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

