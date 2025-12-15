#include <Arduino.h>
#include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

//Nano Wiring details
//Pins D0-D1 - (RS-485 RX/TX lines)

//Pins D2-D7 5-pin DMX address offset (16 channel chunks)
//Pin D7 - additional DIP switch channel (unused)
#define PORTD_OFFSET_MASK B01111100
#define PORTD_OFFSET_PINS ((~PIND & PORTD_OFFSET_MASK) << 2)

//configure DMX Offset DIP switch pins
inline void setup_offset_pins() {
  DDRD &= ~PORTD_OFFSET_MASK; //All inputs, we'll initialize the serial port later.
  PORTD = PORTD_OFFSET_MASK; //enable pullups
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

// Array of DMX values (Only 16 channels, adjusted by offset)
uint8_t dmxData[16];

// Using GPIOR0 for DMX channel counter (0-255 only), overflow_bit in GPIOR1[0]
#define DMX_CHANNEL_REG_L GPIOR0

// Using GPIOR1 for DMX reciving State
// Bitflags 7[RECV_STATE_H, RECV_STATE_L, UPDATE_FLAG, UNUSED(4 bits), DMX_CHANNEL_OVERFLOW_FLAG]1
#define DMX_STATE_REG GPIOR1

//DMX Recieve State
#define DMX_RECV_MASK  0xC0
#define DMX_RECV_IDLE 0x00
#define DMX_RECV_BREAK 0x40
#define DMX_RECV_DATA 0x80
#define setDMXState(STATE) DMX_STATE_REG = (DMX_STATE_REG & ~DMX_RECV_MASK) | STATE

#define DMX_UPDATE_FLAG 0x20

#define DMX_CHANNEL_OVF_FLAG  0x01

// https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
// ((F_CPU / 16) / baud) - 1 per table 19-1
#define DMX_BAUD 250000L
#define UartPrescale(B) (((((F_CPU) / 8) / (B)) - 1) / 2)

uint32_t dmx_last_packet; //uint32 overflows every 49 days :shrug:
#define DMX_TIMEOUT 50

inline void setup_dmx() {
  //Initalize DMX Registers/Peripherals
  DMX_CHANNEL_REG_L = 0;
  DMX_STATE_REG = DMX_RECV_IDLE;
  dmx_last_packet = millis(); // remember current (relative) time in msecs.

  // initialize the DMX buffer
  for (uint8_t n = 0; n < 16; n++)
    dmxData[n] = 0;

  //Setup UART hardware for recieving (Atmega328p UART0)
  UCSR0A = 0;
  const int32_t pre_scale = UartPrescale(DMX_BAUD);
  UBRR0H = pre_scale >> 8;
  UBRR0L = pre_scale;
  UCSR0C = SERIAL_8N1; // accept data packets after first stop bit
  UCSR0B = (1 << RXEN0) | (1 << RXCIE0); //enable UART Reciever and Recieve interrupt

  //Flush the UART Hardware Buffer
  uint8_t voiddata;
  while (UCSR0A & (1 << RXC0)) voiddata = UDR0; // get data  
}

ISR(USART_RX_vect)
{
  uint8_t frame_error = (UCSR0A & (1 << FE0)); // get state before data!
  uint8_t data = UDR0; // get data
  if (frame_error) {
    setDMXState(DMX_RECV_BREAK);
    return;
  }
  switch(DMX_STATE_REG & DMX_RECV_MASK) {
    case DMX_RECV_BREAK: // first byte after a break was read.
      if (data != 0) { // RDM or customer DMX commands are not implemented
        setDMXState(DMX_RECV_IDLE);
        break;
      }
      setDMXState(DMX_RECV_DATA);
      DMX_CHANNEL_REG_L = 0; //reset counter to zero
      DMX_STATE_REG &= ~DMX_CHANNEL_OVF_FLAG; //clear overflow bit
      break;
    case DMX_RECV_DATA:
      uint16_t target_offset = PORTD_OFFSET_PINS;
      uint16_t current_offset = ((DMX_STATE_REG & DMX_CHANNEL_OVF_FLAG) << 8) | (DMX_CHANNEL_REG_L & 0xF0);

      if(current_offset == target_offset) { //within the specified offset range
        uint8_t channel = DMX_CHANNEL_REG_L & 0x0F;
        dmxData[channel] = data;
        if (channel == 0x0F) { //all 16 channels updated, don't bother reading any more data
          DMX_STATE_REG = DMX_RECV_IDLE | DMX_UPDATE_FLAG; //set DMX Update flag and clear state and offset counter
          break;
        }
      }
      DMX_CHANNEL_REG_L++;
      DMX_STATE_REG |= (DMX_CHANNEL_OVF_FLAG & (DMX_CHANNEL_REG_L == 0)); //An overflow happened
      break;
    case DMX_RECV_IDLE:
    default:
      break;
  }
}

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*!
 *  lots of these #defines copied from the Adafruit_PWMServoDriver library
 *
 *  This is a library for our Adafruit 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit 16-channel PWM & Servo
 * driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

//TODO: These may need rejiggered with the different oscillator frequency
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)


/******************* Low level I2C interface */
// #define _i2c Wire
// #define _i2caddr PCA9685_I2C_ADDRESS

// Default PCA9685 I2C Slave Address
#define PCA9685_I2C_ADDRESS 0x40


#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
// #define MODE1_ALLCALL 0x01  /**< respond to LED All Call I2C-bus address */
// #define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
// #define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
// #define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
// #define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */

// PCA9685 Datasheet Default
// #define MODE1_DEFAULT (MODE1_SLEEP | MODE1_ALLCALL)
//My custom Default (remove Allcall), configure autoIncrement, same as the Adafruit Lib
#define MODE1_INIT (MODE1_SLEEP | MODE1_AI)

// #define PCA9685_MODE2 0x01      /**< Mode Register 2 */
// #define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
// #define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
// #define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
// #define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
// #define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
// #define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
// #define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
// #define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */

#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */

// 25 MHz Internal PCA9685 Osclillator Frequency
//#define PCA9685_OSC 25000000
//#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
// uint8_t prescale = ((PCA9685_OSC / (SERVO_FREQ * 4096.0)) + 0.5) - 1; = 121
#define SERVO_PRESCALE 121

uint8_t pwm_read8(uint8_t addr) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PCA9685_I2C_ADDRESS, (uint8_t)1);
  return Wire.read();
}

void pwm_write8(uint8_t addr, uint8_t d) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
}

uint8_t pwm_setPWM(uint8_t num, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(PCA9685_LED0_ON_L + 4 * num);
  Wire.write(on);
  Wire.write(on >> 8);
  Wire.write(off);
  Wire.write(off >> 8);
  return Wire.endTransmission();
}

//Initialize Adafruit PWM Servo board
inline void setup_pwm() {
  Wire.begin();
  uint8_t oldmode = pwm_read8(PCA9685_MODE1);
  pwm_write8(PCA9685_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP); // go to sleep
  //NATE BROKEN: pwm_write8(PCA9685_MODE1, MODE1_INIT);
  pwm_write8(PCA9685_PRESCALE, SERVO_PRESCALE); // set the prescaler
  pwm_write8(PCA9685_MODE1, oldmode);
  delay(5); //TODO: wait for I2C bus to empty instead of dumb delay!
  // "Restart", but I don't think restart does what Adafruit thinks it does, probably don't need this
  //NATE BROKEN: pwm_write8(PCA9685_MODE1, MODE1_INIT | MODE1_RESTART); 
  // This sets the MODE1 register to turn on auto increment.
  pwm_write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

//End Adafruit lib copy/refines

void setup() {
  setup_dmx();
  setup_offset_pins();
  setup_pwm();
  

  //Enable Power LED
  pinMode(PWR_LED, OUTPUT);
  digitalWrite(PWR_LED, HIGH);
  pinMode(DMX_LED, OUTPUT);
  digitalWrite(DMX_LED, LOW);

  //analogRead configures analog pins, no initialization required for A0-A7
}

//DMX Mode loop function()
void dmx_loop() {
  uint32_t last_packet = (uint32_t) millis() - dmx_last_packet;
  if(DMX_STATE_REG & DMX_UPDATE_FLAG) {
    digitalWrite(DMX_LED, HIGH);
    for(int channel = 0; channel < 16; channel++) {
        pwm_setPWM(channel, 0, map(dmxData[channel], 0, 255, SERVOMIN, SERVOMAX));
      }
    dmx_last_packet = millis();
    DMX_STATE_REG &= ~DMX_UPDATE_FLAG; //clear DMX Update Flag
  } else if(last_packet > DMX_TIMEOUT) { //timeout
    digitalWrite(DMX_LED, (last_packet >> 7) & 1); //128ms blink period ~=10 Hz
    return;
  }
}

void manual_loop() {
  digitalWrite(DMX_LED, LOW);
  for(int channel = 0; channel < NUM_POTS; channel++)
    pwm_setPWM(channel, 0, map(analogRead(POT_PINS[channel]), 0, 1023, SERVOMIN, SERVOMAX));
}

void loop() {
  if (!digitalRead(DMX_MODE_IN)) //DMX Mode (Active Low)
    dmx_loop();
  else
    manual_loop();
}
