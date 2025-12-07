#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Nano Wiring details
//Pins D0-D1 - (RS-485 RX/TX lines)

//Pins D2-D7 5-pin DMX address offset (16 channel chunks)
//Pin D7 - additional DIP switch channel (unused)
#define PORTD_BITMASK B01111100
uint16_t readdmx_offset(){
  DDRD &= ~PORTD_BITMASK; //All inputs, we'll initialize the serial port later.
  PORTD = PORTD_BITMASK; //enable pullups
  return ((~PIND & PORTD_BITMASK) << 2); //read the setting
  // PORTD &= ~PORTD_BITMASK; //MEH: disable pullups to save power
}

//Pin D11/PB3 - DMX LED
//Pin D12/PB4 - Power LED
#define DMX_LED 11
#define PWR_LED 12

//Pin D10/PB2 - DMX Mode Switch (Active Low)
#define DMX_MODE_IN 10

//Analog In Channels
//A0-A3 - Potentiometer Axes 1-4
//A4/A5 - (I2C Bus)
//A6-A7 - Potentiometer Axes 5-6
#define NUM_POTS 6
const uint8_t POT_PINS[NUM_POTS] = {A0,A1,A2,A3,A6,A7};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// ----- DMXSerial Private variables -----
// Copied from DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// Copyright (c) 2011-2014 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx

// State of receiving DMX Bytes
typedef enum {
  IDLE = 1, // wait for a BREAK condition.
  BREAK = 2, // BREAK was detected.
  DATA = 3, // DMX data.
} __attribute__((packed)) DMXReceivingState;

uint8_t dmxRecvState; // Current State of receiving DMX Bytes

#define DMXSERIAL_MAX 512 ///< max. number of supported DMX data channels

#define DMXSPEED 250000L
// It implements rounding of ((clock / 16) / baud) - 1.
#define CalcPreScale(B) (((((F_CPU) / 8) / (B)) - 1) / 2)
const int32_t dmxdmxPreScale = CalcPreScale(DMXSPEED); // BAUD prescale factor for DMX speed.

volatile unsigned long dmxLastPacket = 0; // the last time (using the millis function) a packet was received.
bool dmxUpdated = true; // is set to true when new data arrived.

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
uint8_t dmxData[16]; //[DMXSERIAL_MAX + 1];
uint16_t dmxCurrChannel; // This pointer will point to the next byte in dmxData;

// uint8_t *dmxDataPtr; // This pointer will point to the next byte in dmxData;
// uint8_t *dmxDataLastPtr; // This pointer will point to the last byte in dmxData;

// This Interrupt Service Routine is called when a byte or frame error was received.
ISR(USART_RX_vect)
{
  uint8_t frameerror = (UCSR0A & (1 << FE0)); // get state before data!
  uint8_t data = UDR0; // get data
  if (frameerror) { // break condition detected.
    dmxRecvState = BREAK;
    return;
  }
  uint8_t DmxState = dmxRecvState; //just load once from SRAM to increase speed
  switch(DmxState) {
    case BREAK: // first byte after a break was read.
      if (data != 0) { // RDM or customer DMX commands are not implemented
        dmxRecvState = IDLE;
        break;
      }
      dmxRecvState = DATA;
      dmxCurrChannel = 0;
      dmxLastPacket = millis();
      break;
    case DATA:
      // check for new data
      if (dmxCurrChannel >= DMXSERIAL_MAX) { //all 512 bytes recieved
        dmxRecvState = IDLE;
        break;
      }
      if (dmxCurrChannel >= 16) break; //TODO: calculate and apply channels offset 
      dmxUpdated = (dmxData[dmxCurrChannel] != data);
      dmxData[dmxCurrChannel++] = data;
      break;
    case IDLE:
    default:
      break;
  } //state switch
} // ISR(USARTn_RX_vect)

void setup() {
  //Initalize DMX Library
  {
    // initialize global variables
    dmxCurrChannel = 0;
    dmxRecvState = IDLE; // initial state
    dmxLastPacket = millis(); // remember current (relative) time in msecs.

    // initialize the DMX buffer
    for (int n = 0; n < 16; n++)
      dmxData[n] = 0;

    //Setup UART hardware for recieving (Atmega328p UART0)
    UCSR0A = 0; // void dmx_init()
    UBRR0H = dmxdmxPreScale >> 8;
    UBRR0L = dmxdmxPreScale;
    UCSR0C = SERIAL_8N1; // accept data packets after first stop bit
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0); //enable UART Reciever and Recieve interrupt

    //Flush the UART Hardware Buffer
    uint8_t voiddata;
    while (UCSR0A & (1 << RXC0)) voiddata = UDR0; // get data
  }

  //Initialize Adafruit PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //Enable Power LED
  pinMode(PWR_LED, OUTPUT);
  digitalWrite(PWR_LED, HIGH);
  pinMode(DMX_LED, OUTPUT);
  digitalWrite(DMX_LED, LOW);

  //analogRead configures analog pins, no initialization required for A0-A7
}

//DMX Mode loop function()
void dmx_loop() {
  uint16_t dmx_offset = readdmx_offset();
  unsigned long last_packet = millis() - dmxLastPacket; //DMXSerialClass::noDataSince()

  if(last_packet > 50) { //timeout
    digitalWrite(DMX_LED, (last_packet >> 7) & 1); //128ms blink period ~=10 Hz
    return;
  }

  digitalWrite(DMX_LED, HIGH);
  if(dmxUpdated) { // DMXSerial.dataUpdated()) {
    for(int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, map(dmxData[channel], 0, 255, SERVOMIN, SERVOMAX));
      }
    dmxUpdated = false; //DMXSerial.resetUpdated();
  }
}

void manual_loop() {
  digitalWrite(DMX_LED, LOW);
  for(int channel = 0; channel < NUM_POTS; channel++)
    pwm.setPWM(channel, 0, map(analogRead(POT_PINS[channel]), 0, 1023, SERVOMIN, SERVOMAX));
}

void loop() {
  if (!digitalRead(DMX_MODE_IN)) //DMX Mode (Active Low)
    dmx_loop();
  else
    manual_loop();
}
