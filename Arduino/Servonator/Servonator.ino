#include <Arduino.h>
#include <servonator_hal.h>

#include <DMXSerial.h>
uint16_t dmxStartAddress = 1;

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


void setup() {
  //Initalize DMX Library
  DMXSerial.init(DMXReceiver);

  //Initialize Adafruit PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //Initialize discrete hardware
  Servonator.setup();
}


void loop() {
  dmxStartAddress = Servonator.read_dmx_address();
  if (DMXSerial.noDataSince() < 50){
    digitalWrite(Servonator.dmx_led, HIGH);
    
    if(DMXSerial.dataUpdated()) {
      for(int channel = 0; channel < 16; channel++)
        pwm.setPWM(channel, 0, map(DMXSerial.read(dmxStartAddress + channel), 0, 255, SERVOMIN, SERVOMAX));
      DMXSerial.resetUpdated();
    }
  } else { //timeout, Manual Controls! */
    digitalWrite(Servonator.dmx_led, LOW);
    for(int channel = 0; channel < Servonator.num_pots; channel++)
      pwm.setPWM(channel, 0, map(Servonator.read_pot(channel), 0, 1023, SERVOMIN, SERVOMAX));
  }
}

  
