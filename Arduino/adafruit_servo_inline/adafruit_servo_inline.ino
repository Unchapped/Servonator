//#include <Wire.h>
#include <Arduino.h>

// Copied form i2cmaster.c because Arduino IDE kinda sux at path configuration
/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.4 2015/01/17 12:16:05 peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
extern "C" {
#include <avr/io.h>
#include <compat/twi.h>
}

#define SCL_CLOCK  100000L
/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */



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


void _write8(uint8_t addr, uint8_t d) {
  i2c_start(PCA9685_I2C_ADDRESS);
  i2c_write(addr);
  i2c_write(d);
  i2c_stop();
}

// TODO: consider porting this to pfleury's twimaster library
// uint8_t _write8(uint8_t addr, uint8_t d) {
//   uint8_t twi_buf[] = {addr,d};
//   return twi_writeTo(PCA9685_I2C_ADDRESS, twi_buf, 2, 1, true);
// }

void inline _beginPWM() {
  i2c_init(); //Wire.begin();
  // _write8(PCA9685_MODE1, MODE1_SLEEP); // go to sleep
  _write8(PCA9685_PRESCALE, SERVO_PRESCALE); // set the prescaler
  _write8(PCA9685_MODE1, MODE1_RESTART | MODE1_AI); // Wake up
}

//TODO: since we have a local buffer, we should just do all 16 channels in one transaction
void inline _setPWM(uint8_t num, uint16_t on, uint16_t off) {
  i2c_start(PCA9685_I2C_ADDRESS);
  i2c_write(PCA9685_LED0_ON_L + 4 * num);
  i2c_write(on);
  i2c_write(on >> 8);
  i2c_write(off);
  i2c_write(off >> 8);
  i2c_stop();
  //return Wire.endTransmission();
}


void setup() {
  Serial.begin(9600);
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
