
#include <DMXSerial.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  110 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  470 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//Das blinkenlights
#define POWER_LED 13
#define DMX_LED 12


/* Read PORTD 3-7 (Arduino D3-D7) for upper 5 bits of starting DMX Address
 * Assumes we get a 1-indexed aligned 16 channel segment of the DMX buffer
 * therefore if D2-D6 are set to b00001, that's equivalent to DMX addresses
 * 17 through 32, and so forth. Since we're using the internal pullups for
 * simplicity, we need to tie down "1" bits, and invert the read value.
 * NONPORTABLE: This is UNO specific and not portable, but we do this raw for
 *              speed. If/when we migrate to a new architecture we'll need to
 *              add compiler directives for other boards. */
#define PORTD_BITMASK B11111000
unsigned int read_dmx_address(){
  DDRD &= ~PORTD_BITMASK; //All inputs, we'll initialize the serial port later.
  PORTD = PORTD_BITMASK; //enable pullups
  unsigned int addr = ((~PIND & PORTD_BITMASK) << 1) | 1; //read the setting
  PORTD &= ~PORTD_BITMASK; //disable pullups to save power
  return addr;
}

unsigned int dmxStartAddress = 1; //Default DMX bus address to start with.

struct potentiometer {
  int pin;
  bool inverted;
  } pots[] = {
    {A0, false},
    {A1, false},
    {A2, false},
    {A3, false}
  };
  
  /* ERRATA: A4/A5 Clobber the I2C bus on the Uno, full functionality would require a Mega or similar...
    {A4, true},
    {A5, true}
  }; */


void setup() {
  //Initalize DMX Library
  DMXSerial.init(DMXReceiver);

  //Initialize Adafruit PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //Initialize LEDs
  pinMode(POWER_LED, OUTPUT);
  pinMode(DMX_LED, OUTPUT);

  //Setup complete, turn on Power LED
  digitalWrite(POWER_LED, HIGH);
}


void loop() {
  dmxStartAddress = read_dmx_address();
  if (DMXSerial.noDataSince() < 100){
    digitalWrite(DMX_LED, HIGH);
    
    if(DMXSerial.dataUpdated()) {
      for(int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, map(DMXSerial.read(dmxStartAddress + channel), 0, 255, SERVOMIN, SERVOMAX));
      }
      DMXSerial.resetUpdated();
    }
  } else { //timeout, Manual Controls! */
    digitalWrite(DMX_LED, LOW);
    for(int channel = 0; channel < 4; channel++) pwm.setPWM(channel, 0, map(read_potentiometer(channel), 0, 1023, SERVOMIN, SERVOMAX));
  }
}

  
