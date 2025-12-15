// #include <Wire.h>
#include <I2C.h>

//Necessary PWM Defines
#define PCA9685_I2C_ADDRESS 0x40
#define PCA9685_MODE1 0x00
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_RESTART 0x80 /**< Restart enabled */
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_PRESCALE 0xFE

// #define PCA9685_OSC 25000000
// #define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
// uint8_t prescale = ((PCA9685_OSC / (SERVO_FREQ * 4096.0)) + 0.5) - 1; = 121
#define SERVO_PRESCALE 121

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)

// void _write8(uint8_t addr, uint8_t d) {
//   Wire.beginTransmission(PCA9685_I2C_ADDRESS);
//   Wire.write(addr);
//   Wire.write(d);
//   Wire.endTransmission();
// }

void inline _beginPWM() {
  I2c.begin();
  // _write8(PCA9685_MODE1, MODE1_SLEEP); // go to sleep
  I2c.write(PCA9685_I2C_ADDRESS, PCA9685_PRESCALE, SERVO_PRESCALE); // set the prescaler
  I2c.write(PCA9685_I2C_ADDRESS, PCA9685_MODE1, MODE1_RESTART | MODE1_AI); // Wake up
}

uint8_t inline _setPWM(uint8_t num, uint16_t on, uint16_t off) {
  uint8_t data[] = {on, (on >> 8), off, (off >> 8)};
  return I2c.write(PCA9685_I2C_ADDRESS, PCA9685_LED0_ON_L + 4 * num, data);
  // Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  // Wire.write(PCA9685_LED0_ON_L + 4 * num);
  // Wire.write(on);
  // Wire.write(on >> 8);
  // Wire.write(off);
  // Wire.write(off >> 8);
  // return Wire.endTransmission();
}


void setup() {
  _beginPWM();
}


void loop() {
  static uint16_t pulselen = SERVOMIN;
  static int16_t dir = 1;

  if (pulselen == SERVOMIN) dir = 1;
  if (pulselen == SERVOMAX) dir = -1;
  pulselen += dir;

  for (uint8_t servonum = 0; servonum < 16; servonum++) {
    _setPWM(servonum, 0, pulselen);
  }
}
