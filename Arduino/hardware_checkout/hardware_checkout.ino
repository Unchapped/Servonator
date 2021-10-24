#include <Arduino.h>
#include <servonator_hal.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


#include <FancyDelay.h>
FancyDelay oneSec(1000);

void setup() {
  //Initalize Serial Debug channel
  Serial.begin(9600);

  //Initialize Adafruit PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //Initialize discrete hardware
  Servonator.setup();
}


void loop() {
  Serial.write('\r'); //ANSI Escape Code
  
  Serial.print("DMX Read Address: ");
  Serial.print(Servonator.read_dmx_address());

  Serial.print("  Potentiometer Values: ");
  for(int channel = 0; channel < Servonator.num_pots; channel++) {
    Serial.print(Servonator.read_pot(channel));
    Serial.print(", ");
  }

  static int power_state = HIGH;
  static int dmx_state = LOW;
  static int servo_0_pos = SERVOMIN;
  static int servo_15_pos = SERVOMAX;

  if(oneSec.ready()){
    power_state = HIGH - power_state;
    dmx_state = HIGH - dmx_state;
    servo_0_pos = SERVOMAX - servo_0_pos + SERVOMIN;
    servo_15_pos = SERVOMAX - servo_15_pos + SERVOMIN;
  }

  Serial.print("  LED states: Power: ");
  Serial.print(power_state);
  digitalWrite(Servonator.power_led, power_state);

  Serial.print(",  DMX: ");
  Serial.print(dmx_state);
  digitalWrite(Servonator.dmx_led, dmx_state);

  Serial.print("  Servo Channels: 0: ");
  Serial.print(servo_0_pos);
  pwm.setPWM(0, 0, servo_0_pos);

  Serial.print(",  16: ");
  Serial.print(dmx_state);
  Serial.print(servo_15_pos);
  pwm.setPWM(15, 0, servo_15_pos);

  Serial.flush(); //wait for Serial to complete transmission
}

  
