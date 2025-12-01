#include <Arduino.h>

#include <DMXSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Nano Wiring details
//Pins D0-D1 - (RS-485 RX/TX lines)

//Pins D2-D7 5-pin DMX address offset (16 channel chunks)
//Pin D7 - additional DIP switch channel (unused)
#define PORTD_BITMASK B01111100
uint16_t read_dmx_offset(){
  DDRD &= ~PORTD_BITMASK; //All inputs, we'll initialize the serial port later.
  PORTD = PORTD_BITMASK; //enable pullups
  return ((~PIND & PORTD_BITMASK) << 2); //read the setting
  // PORTD &= ~PORTD_BITMASK; //MEH: disable pullups to save power
}

//Pin D11/PB3 - DMX LED
//Pin D12/PB4 - Power LED
#define DMX_LED 11
#define PWR_LED 12

//Pin D10/PB2 - DMX Shield Enable (Active Low)
//Pin D9/PB1 - DMX Mode Switch (Active Low)
#define DMX_MODE_OUT 10
#define DMX_MODE_IN 9
uint8_t dmx_mode_select = 1; // 1 = Manual Mode

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


void setup() {
  //Enable/Disable DMX Shield based on switch state
  //TODO: DOING THIS IN SOFTWARE BREAKS NANO FLASHING! NEED TO MAKE THIS A PHYSICAL SPLICE!
  dmx_mode_select = digitalRead(DMX_MODE_IN);
  pinMode(DMX_MODE_OUT, OUTPUT);
  digitalWrite(DMX_MODE_OUT, dmx_mode_select); // set high by default to disable

  //Initalize DMX Library
  DMXSerial.init(DMXReceiver);

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
  uint16_t dmx_offset = read_dmx_offset();
  unsigned long last_packet = DMXSerial.noDataSince();
  if(last_packet > 50) { //timeout
    digitalWrite(DMX_LED, (last_packet >> 7) & 1); //128ms blink period ~=10 Hz
    return;
  }

  digitalWrite(DMX_LED, HIGH);
  if(DMXSerial.dataUpdated()) {
    for(int channel = 0; channel < 16; channel++)
      pwm.setPWM(channel, 0, map(DMXSerial.read(dmx_offset + channel + 1), 0, 255, SERVOMIN, SERVOMAX));
    DMXSerial.resetUpdated();
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

  
