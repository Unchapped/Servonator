
#include <Arduino.h>
#include <servonator_hal.h>

#include <FancyDelay.h>
FancyDelay oneSec(1000);

void setup() {
  //Initalize DMX Library
  Serial.begin(9600);

  //Initialize Adafruit PWM
  //pwm.begin();
  //pwm.setOscillatorFrequency(27000000);
  //pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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

  if(oneSec.ready()){
    power_state = HIGH - power_state;
    dmx_state = HIGH - dmx_state;
  }

  Serial.print("  LED states: Power: ");
  Serial.print(power_state);
  digitalWrite(Servonator.power_led, power_state);

  Serial.print(",  DMX: ");
  Serial.print(dmx_state);
  digitalWrite(Servonator.dmx_led, dmx_state);

  //TODO: PWM checkouts
  
  Serial.flush(); //wait for Serial to complete transmission
}

  
