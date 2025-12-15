#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
// #define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// Default PCA9685 I2C Slave Address
#define PCA9685_I2C_ADDRESS 0x40
// #define PCA9685_OSC 25000000

// uint8_t prescale = ((PCA9685_OSC / (SERVO_FREQ * 4096.0)) + 0.5) - 1; = 121
#define SERVO_PRESCALE 121

uint8_t _read8(uint8_t addr) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(PCA9685_I2C_ADDRESS, (uint8_t)1);
  return Wire.read();
}

void _write8(uint8_t addr, uint8_t d) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
}

void inline _beginPWM() {
  Wire.begin();
  //void reset();
  {
    // _write8(PCA9685_MODE1, MODE1_RESTART);
    // delay(10);
  }
  {
    _write8(PCA9685_MODE1, MODE1_SLEEP); // go to sleep
    _write8(PCA9685_PRESCALE, SERVO_PRESCALE); // set the prescaler
    _write8(PCA9685_MODE1, MODE1_RESTART | MODE1_AI); // Wake up
  }
}

uint8_t inline _setPWM(uint8_t num, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(PCA9685_LED0_ON_L + 4 * num);
  Wire.write(on);
  Wire.write(on >> 8);
  Wire.write(off);
  Wire.write(off >> 8);
  return Wire.endTransmission();
}


void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  _beginPWM();
  //pwm.begin();
  //pwm.setOscillatorFrequency(25000000); //THIS IS WHERE THE FUCKING BUG CAME FROM!
  //pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}



uint16_t pulselen = SERVOMIN;
int16_t dir = 1;

void loop() {
  if (pulselen == SERVOMIN) dir = 1;
  if (pulselen == SERVOMAX) dir = -1;
  pulselen += dir;

  for (uint8_t servonum = 0; servonum < 16; servonum++) {
    _setPWM(servonum, 0, pulselen);
  }
}
