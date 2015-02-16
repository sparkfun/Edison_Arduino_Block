/****************************************************************
   Edison Firmata Client
   by: Jim Lindblom @ SparkFun Electronics
   created on: Februrary 12, 2015
   github: https://github.com/sparkfun/Edison_Arduino_Block
   
   This is an Firmata client sketch for the Edison. It can
   communicate with an Arduino running Firmata over a Serial
   connection.
   
   Support for the following functions is written:
     firmata_init() -- set up firmata and pin reporting
     firmata_pinMode([pin], [0, 1, 2, 3, 4, 5, 6])
     firmata_digitalWrite([pin], [LOW/HIGH])
     firmata_analogWrite([pin], [0-255])
     firmata_analogRead([0-7])
     firmata_digitalRead([pin])
     firmata_servoWrite([pin], [value])
   
   Development Environment Specifics:
   Arduino 1.5.3 (for Edison)
   Intel Edison rev C
   Arduino Block for Edison
     Arduino should be running StandardFirmata
   
   This sketch is based on Firmata's processing client:
   https://github.com/firmata/processing
   As such, it is released under the same, free license. You can 
   redistribute it and/or modify it under the terms of the GNU 
   Lesser General Public License as published by the Free 
   Software Foundation; either version 2.1 of the License, or 
   (at your option) any later version.

   Distributed as-is; no warranty is given. 
****************************************************************/
// SerialEvent1 isn't defined in the Edison core (I think).
// To get some form of interrupt-driven Serial input, we'll read
// serial in on a timer.
#include <TimerOne.h>

#define MAX_DATA_BYTES 4096
#define MAX_PINS 128

// Pin Mode definitons:
// Use any of these six values to set a pin to INPUT, OUTPUT,
// ANALOG, PWM, SERVO, SHIFT, or I2C.
enum const_pin_mode {
  MODE_INPUT,  // 0
  MODE_OUTPUT, // 1
  MODE_ANALOG, // 2
  MODE_PWM,    // 3
  MODE_SERVO,  // 4
  MODE_SHIFT,  // 5
  MODE_I2C     // 6
};

// Message Types
// Used by the low-level Firmata functions to set up the 
// Firmata messages.
#define ANALOG_MESSAGE  0xE0
#define DIGITAL_MESSAGE 0x90
#define REPORT_ANALOG   0xC0
#define REPORT_DIGITAL  0xD0
#define START_SYSEX     0xF0
#define SET_PIN_MODE    0xF4
#define END_SYSEX       0xF7
#define REPORT_VERSION  0xF9
#define SYSTEM_RESET    0xFF

// Extended Commands:
// Used by the low-level Firmata functions to set up the 
// Firmata messages.
#define SERVO_CONFIG            0x70
#define STRING_DATA             0x71
#define SHIFT_DATA              0x75
#define I2C_REQUEST             0x76
#define I2C_REPLY               0x77
#define I2C_CONFIG              0x78
#define EXTENDED_ANALOG         0x6F
#define PIN_STATE_QUERY         0x6D
#define PIN_STATE_RESPONSE      0x6E
#define CAPABILITY_QUERY        0x6B
#define CAPABILITY_RESPONSE     0x6C
#define ANALOG_MAPPING_QUERY    0x69
#define ANALOG_MAPPING_RESPONSE 0x6A
#define REPORT_FIRMWARE         0x79
#define SAMPLING_INTERVAL       0x7A
#define SYSEX_NON_REALTIME      0x7E
#define SYSEX_REALTIME          0x7F

// Flags and variables to keep track of message reading status:
boolean parsingSysex = false;
int waitForData = 0;
int storedInputData[MAX_DATA_BYTES];
int sysexBytesRead = 0;
int executeMultiByteCommand = 0;
int multiByteChannel = 0;
// Variable arrays to keep track of pin values read in.
int digitalInputData[] = {0, 0, 0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0, 0, 0};
int analogInputData[] = {0, 0, 0, 0, 0, 0, 0, 0, 
                         0, 0, 0, 0, 0, 0, 0, 0};
int analogChannel[MAX_PINS];
boolean blinkFlag = false;

void setup()
{
  // Debug messages are sent out Serial. Use the Serial monitor
  // at 9600 bps to read pin values.
  Serial.begin(9600);
  
  // firmata_init sets up our firmata client. It tells the 
  // Firmata device to begin streaming analog values and any
  // digital pin value changes.
  firmata_init();
  
  // Use firmata_pinMode([pin], [value]) to set up pins on the
  // Firmata host:
  firmata_pinMode(13, MODE_OUTPUT); // LED tied to pin 13
  firmata_pinMode(4, MODE_INPUT);   // Digital input on pin 4
  firmata_pinMode(A0, MODE_ANALOG); // Analog input on pin 0
  firmata_pinMode(3, MODE_PWM);     // PWM LED on pin 3
}

void loop() 
{
  // Print the value of our Firmata Arduino's A0 pin:
  int a0Value = firmata_analogRead(0);
  Serial.print("A0: ");
  Serial.println(a0Value);
  
  
  // Print the value of our digital input on pin 4:
  int d4Value = firmata_digitalRead(4);
  Serial.print("Pin 4: ");
  Serial.println(d4Value);
  
  if (d4Value == LOW)
  {
    // Scale the value of A0 to write a PWM output on pin 3:
    firmata_analogWrite(3, firmata_analogRead(0) / 4);
  }
  else
  {
    // Scale the value of A0 to write a PWM output on pin 3:
    firmata_analogWrite(3, 0);
  }
  
}

///////////////////////////////////
// Upper Level Firmata Functions //
///////////////////////////////////
// Firmata functions that you should use in your sketch above.
// If this was a class, these'd be public functions.

// firmata_init() -- 
// - Initialize our Firmata Serial port. 
// - Set up a timer to read in Serial messages outside of loop().
// - Configure our Firmata Arduino to report all digital outputs
// - Configure our Firmata Arduino to report all analog outputs
void firmata_init()
{
  Serial1.begin(57600);
  // set a timer of length 100,000 microseconds ( 0.1 sec - or 10Hz)
  Timer1.initialize(1000); 
  Timer1.attachInterrupt( checkSerial ); // attach the service routine here
  
  // Turn on reporting for all digital ports
  for (int i=0; i<16; i++)
  {
    Serial1.write(REPORT_DIGITAL | i);
    Serial1.write(1);
  }
  // This function will check for analog channels and set them
  // to REPORTING
  firmata_queryAnalogMapping();
}

// firmata_digitalRead([pin]) --
// - Returns the latest digital input value we've read on the
//   requested pin.
// - digitalInputData[] is updated in firmata_processInput()
//   as serial messages come in.
int firmata_digitalRead(int pin)
{
  return (digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01;
}

// firmata_analogRead([pin])
// - Returns the latest analog value we've read on the requested
//   pin.
// - analogInputData[] is updated in firmata_processInput()
//   as serial messages come in.
int firmata_analogRead(int pin)
{
  return analogInputData[pin];
}

// firmata_pinMode([pin], [mode])
// - Set an Arduino pin to input, output, analog in, PWM, servo,
//   shift register, or i2c.
// - [pin] - can be any Arduino pin 0-13, A0-A7
// - [mode] - should be one of these defined values:
//          - MODE_INPUT  - Digital input
//          - MODE_OUTPUT - Digital output
//          - MODE_ANALOG - Analog input
//          - MODE_PWM    - Analog output
//          - MODE_SERVO  - Servo output
//          - MODE_SHIFT  - Shift register output
void firmata_pinMode(int pin, int mode)
{
  Serial1.write(SET_PIN_MODE);
  Serial1.write(pin);
  Serial1.write(mode);
}

// firmata_digitalWrite([pin], [value])
// - Set an Arduino digital pin to HIGH or LOW
// - [pin] - Any digital pin 0-18
// - [value] - LOW or HIGH
void firmata_digitalWrite(int pin, int value)
{
  int port = (pin >> 3) & 0x0F;
  int data;
  
  if (value)
    data |= (1 << (pin & 0x07));
  else
    data &= ~(1 << (pin & 0x07));
    
  Serial1.write(DIGITAL_MESSAGE | port); // Digital data
  Serial1.write(data & 0x7F); // Digital pins 0-6 bitmask
  Serial1.write(data >> 7);   // Digital pin 7 bitmask
}

// firmata_analogWrite([pin], [value])
// - Set an Arduino pin - CONFIGURED AS PWM (!) - to an
//   analog output value.
// - [pin] - Any analog output capable pin (3, 5, 6, 9, 10, 11
// - [value] - 0-255
void firmata_analogWrite(int pin, int value)
{
    
  Serial1.write(ANALOG_MESSAGE | (pin& 0x0F)); // Analog pin
  Serial1.write(value & 0x7F); // Analog LS 7 bits
  Serial1.write(value >> 7);   // Analog MS 7 bits
}

// firmata_servoWrite([pin], [value])
// - Set an Arduino pin - CONFIGURED AS SERVO (!) - to output
//   a servo signal.
void firmata_servoWrite(int pin, int value)
{
  Serial1.write(ANALOG_MESSAGE | (pin & 0x0F));
  Serial1.write(value & 0x7F);
  Serial1.write(value >> 7);
}

/////////////////////////////////
// Low level Firmata functions //
/////////////////////////////////
// Firmata helper functions you probably won't need to call in
// your sketch. If this was a class, these'd be private functions.

// checkSerial()
// Interrupt-recurring function. Checks for available serial data
// and processes any serial messages that come in.
void checkSerial()
{
  while (Serial1.available())
  {
    //Serial.write(Serial1.read());
    firmata_processInput((unsigned char) Serial1.read());
  }
  if (blinkFlag)
  {
    firmata_digitalWrite(13, HIGH); // Tell Arduino to write 13 HIGH
    blinkFlag = false;
  }
  else
  {
    firmata_digitalWrite(13, LOW); // Tell Arduino to write 13 HIGH
    blinkFlag = true;
  }
}

// firmata_processInput([inputData])
// Handles all Firmata messages - everything from version checks
// to analog and digital readings.
void firmata_processInput(unsigned char inputData)
{
  int command;
  
  if (parsingSysex) // If we're parsing a system message
  {
    if (inputData == END_SYSEX)
    { // Received end of system message, process it
      parsingSysex = false;
      firmata_processSysexMessage();
    }
    else
    { // In the system message, add to it
      storedInputData[sysexBytesRead] = inputData;
      sysexBytesRead++;
    }
  }
  else if (waitForData > 0 && inputData < 128)
  { // Else waiting for data
    waitForData--; // Decrement wait for data
    storedInputData[waitForData] = inputData;
    
    if (executeMultiByteCommand != 0 && waitForData == 0)
    {
      switch(executeMultiByteCommand)
      {
      case DIGITAL_MESSAGE:
        firmata_setDigitalInputs(multiByteChannel, 
                 (storedInputData[0] << 7) + storedInputData[1]);
        break; 
      case ANALOG_MESSAGE:
        firmata_setAnalogInput(multiByteChannel, 
                 (storedInputData[0] << 7) + storedInputData[1]);
        break; 
      case REPORT_VERSION:
        firmata_setVersion(storedInputData[1], storedInputData[0]);
        break; 
      }
    }
  }
  else // Beginning of a message
  {
    if (inputData < 0xF0)
    {
      command = inputData & 0xF0;
      multiByteChannel = inputData & 0x0F;
    }
    else
    {
      command = inputData;
    }
    switch (command)
    {
    case DIGITAL_MESSAGE:
    case ANALOG_MESSAGE:
    case REPORT_VERSION:
      waitForData = 2;
      executeMultiByteCommand = command;
      break;
    case START_SYSEX:
      parsingSysex = true;
      sysexBytesRead = 0;
      break;
    }
  }
}

// firmata_processSysexMessage()
// - Process a system message.
// - Mostly just handles defining which pins are analog channels.
void firmata_processSysexMessage()
{
  switch (storedInputData[0])
  {
  case ANALOG_MAPPING_RESPONSE:
    // Begin by setting every channel to NOT analog (127)
    for (int pin = 0; pin < sizeof(analogChannel); pin++)
      analogChannel[pin] = 127;
    // Enumerate analog channels:
    for (int i = 1; i < sysexBytesRead; i++)
      analogChannel[i - 1] = storedInputData[i];
    // Set each analog output to reporting
    for (int pin = 0; pin < sizeof(analogChannel); pin++)
    {
      if (analogChannel[pin] != 127)
      {
        Serial1.write(REPORT_ANALOG | analogChannel[pin]);
        Serial1.write(1);
      }
    }
    break;
  }
}

// firmata_queryAnalogMapping()
// - Send the ANALOG_MAPPING_QUERY request message.
// - Called in init to keep track of which pins are analog ins
void firmata_queryAnalogMapping()
{
  Serial1.write(START_SYSEX);
  Serial1.write(ANALOG_MAPPING_QUERY);
  Serial1.write(END_SYSEX);
}

// firmata_setDigitalInputs()
// - Set a value in the digitalInputData array to portData
void firmata_setDigitalInputs(int portNumber, int portData)
{
  digitalInputData[portNumber] = portData;
}

// firmata_setAnalogInput
// - Set an analog pin value to [value].
void firmata_setAnalogInput(int pin, int value)
{
  analogInputData[pin] = value;
}

void firmata_setVersion(int majorVersion, int minorVersion)
{
  //! Todo
}
