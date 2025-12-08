#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


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


void setup() {
  setup_dmx();
  setup_offset_pins();

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
  uint32_t last_packet = (uint32_t) millis() - dmx_last_packet;
  if(DMX_STATE_REG & DMX_UPDATE_FLAG) {
    digitalWrite(DMX_LED, HIGH);
    for(int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, map(dmxData[channel], 0, 255, SERVOMIN, SERVOMAX));
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
    pwm.setPWM(channel, 0, map(analogRead(POT_PINS[channel]), 0, 1023, SERVOMIN, SERVOMAX));
}

void loop() {
  if (!digitalRead(DMX_MODE_IN)) //DMX Mode (Active Low)
    dmx_loop();
  else
    manual_loop();
}
