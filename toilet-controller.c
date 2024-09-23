/* Toilet controller
*
* Created:   2024-09-08
* Author:    Stefan Andersson
* Hardware:	Atmel ATtinyx5 on custom PCB

PINOUT + SPI programming info
        _____
RESET 1|o    |8 VCC
      2|     |7 SPI-SCK
      3|     |6 SPI-MISO
  GND 4|_____|5 SPI-MOSI

PIN#	NAME   description (matces ECAD schematic)
2		  PB3    BUTTON-INPUT-FILTERED (0/5VDC input from external pushbutton)
3     ADC2   TRIM-POT (0-5VDC input from PCB potentiometer)
5     PBO    DRIVER-CONTROL-2 (to IN2 @ IC DRV8231DDAR)
6		  PB1    DRIVER-CONTROL-1 (to IN1 @ IC DRV8231DDAR)

Model number: SCHELL 01 594 00 99, 0513 1164, WC-Deckeneinbau-Steuerung EDITION E

Data from reverse engineering
0 ms: Button pressed down
290 ms: Solenoid coil activated
305 ms (15 ms after activation): Solenoid coil de-activated
1650 + [setpoint seconds * 1000] ms: Solenoid coil activated in reverse
1665 + [setpoint seconds * 1000] ms: Sonenoid coil de-activated

Comments:
Flushing time/function is not affected by how long the pushbutton is pressed.
Solenoid data: 6 VDC, 1.2 W

FUSE SETTINGS
CLOCK FREQUENCY:	1 MHZ INTERNAL CLOCK
BOD:				      2.7V
STARTUP TIME:     6CK + 14CK + 64ms
AVRdude fuse bits: avrdude -c avrispmkII -p t85 -e -U lfuse:w:0x62:m -U
hfuse:w:0xdd:m -U efuse:w:0xff:m https://www.engbedded.com/fusecalc/
*/

/* INCLUDE LIBRARIES */
#include <avr/io.h>
#include <avr/interrupt.h> // must be included to enable/disable interrupts??
#define F_CPU 1000000UL    // must be defined for delay library!
#include <util/delay.h>

/* MACROS */
#define TRUE 1                // for readability
#define FALSE 0               // for readability
#define PUSHBUTTON PB3        // pushbutton pin
#define TRIM_POT ADC2         // trim-pot pin
#define DRIVER_CONTROL_2 PBO  // driver-control pin 2
#define DRIVER_CONTROL_1 PB1  // driver-control pin 1

#define TIMER1_1MS                                                             \
  while (!tickMs) {}                                                           \
  tickMs = FALSE // wait until tick is activated (1/ms) and then reset the variable

// reset TIMER1 and prescaler to 0
#define TIMER1_RESET                                                           \
  TCNT1 = 0;                                                                   \
  GTCCR |= (1 << PSR1);                                                        \
  tickMs = FALSE

#define COIL_FORWARD                                                           \
  PORTB |= (1 << PB1);                                                         \
  PORTB &= ~(1 << PB0)  // motor driver IN1 = HIGH, IN2 = LOW
#define COIL_REVERSE                                                           \
  PORTB &= ~(1 << PB1);                                                        \
  PORTB |= (1 << PB0)   // motor driver IN1 = LOW, IN2 = HIGH
#define COIL_COAST                                                             \
  PORTB &= ~(1 << PB1);                                                        \
  PORTB &= ~(1 << PB0)  // motor driver IN1 = LOW, IN2 = LOW
#define COIL_BRAKE                                                             \
  PORTB |= (1 << PB1);                                                         \
  PORTB |= (1 << PB0)   // motor driver IN1 = HIGH, IN2 = HIGH
#define START_ADC_CONVERSION                                                   \
  ADCSRA |= (1 << ADSC) // start a new ADC conversion
#define ADC_CONVERSION_COMPLETED                                               \
  ADCSRA & (1 << ADIF)  // check if conversion result is available
#define READ_ADC_RESULT                                                        \
  ADCH  // 8 most significant bits from 10-bit ADC result
#define RESET_ADC_RESULT                                                       \
  ADCSRA |= (1 << ADIF)

//#define bitset(byte,nbit)   ((byte) |=  (1<<(nbit)))
//#define bitclear(byte,nbit) ((byte) &= ~(1<<(nbit)))
//#define bitflip(byte,nbit)  ((byte) ^=  (1<<(nbit)))
//#define bitcheck(byte,nbit) ((byte) &   (1<<(nbit)))


/* DEFINE USER SETTING VARIABLES (all variables unsigned) */
#define TIME_COIL_ACTIVE        15    // [ms] time coil is active when activated. both normal + reverse operation.
#define TIME_INITIAL_DELAY      290   // [ms] delay after pressing button before coil is activated
#define TIME_MIN_FLUSHING       1650  // [ms] default minimum delay after pressing button before coil is activated in reverse

/* DECLARE GLOBAL VARIABLES */
uint16_t buttonFilter = 0; // stores an array of binary input values
uint8_t buttonStatus = 0;  // current filtered status of the pushbutton input
volatile uint8_t tickMs = FALSE;  // set by timer1 every ms. Used as ms-"tick" function. Declared volatile since it is used by ISR.


// Description:   Setup I/O port
// Return value:  None
void SetupIO() {
  /* SET I/O REGISTERS */

  // set port direction (input/output)
  // coil driver pins are set as output
  DDRB = (0 << PB5) | (0 << PB4) | (0 << PB3) | (0 << PB2) | (1 << PB1) | (1 << PB0);
  // set internal pull-up resistors
  // PB2 (SPI-SCK) internal pullup is activated to define pin potential (not floating)
  PORTB = (0 << PB5) | (0 << PB4) | (0 << PB3) | (1 << PB2) | (0 << PB1) | (0 << PB0);
}


// Description:   Setup ADC for ADC2 (pin 3) trim-potentiometer input
// Return value:  None
void SetupADC() {
  ADMUX = (0 << REFS1)  // set reference Voltage to VCC
        | (0 << REFS0)  // set reference Voltage to VCC
        | (1 << ADLAR)  // set to left adjusted result data
        | (0 << REFS2)  // set reference Voltage to VCC
        | (0 << MUX3)   // select input channel ADC2
        | (0 << MUX2)   // select input channel ADC2
        | (1 << MUX1)   // select input channel ADC2
        | (0 << MUX0);  // select input channel ADC2

  ADCSRA = (1 << ADEN)    // enable ADC
         | (0 << ADSC)    // ADC Start Conversion
         | (0 << ADATE)   // auto triggering disabled
         | (0 << ADIF)    // ADC Interrupt Flag
         | (0 << ADIE)    // disable ADC interrupt
         | (1 << ADPS2)   // division factor for prescaler
         | (1 << ADPS1)   // division factor for prescaler
         | (1 << ADPS0);  // division factor for prescaler
  // ADPS[2..0] = 111, division factor 128 (slowest, most accurate)

  DIDR0 = (1 << ADC2D); // disable digital input buffer on pin ADC2

  // Perform an initial ADC conversion. Initial conversion takes longer time.
  START_ADC_CONVERSION;
  while (ADC_CONVERSION_COMPLETED) {}
  RESET_ADC_RESULT;
}


// Description:   Read input status and filter signal.
// Return value:  uint8_t, Push button state. True = pressed
// Comments:      ~0.5mS delay (hardware RC filter) on pushbutton input pin.
//                Function stores input state values in a bit array and counts
//                amount of high/low bits and compares against current status
//                and hysteresis value.
uint8_t ReadPushbutton() {
  // software filter settings
  uint8_t hysteresis = 3; // bits of hysteresis required for button status change

  buttonFilter <<= 1; // shift bit array one bit left (make space for new reading)
  if (!(PINB & (1 << PUSHBUTTON))) {  // pushbutton input signal is inverted.
    buttonFilter |= 1;
  }

  // Count amount of high bits in bit-sequence
  uint8_t matches = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (buttonFilter & (1 << i)) {
      matches++;
    }
  }

  // Check if status should be changed
  if ((buttonStatus == TRUE) && (matches <= (8 - hysteresis))) {
    buttonStatus = FALSE;
  }
  else if ((buttonStatus == FALSE) && (matches >= (8 + hysteresis))) {
    buttonStatus = TRUE;
    //buttonFilter = 0; // Reset bit array. Prevent false activation after flush cycle is complete.
  }
  _delay_ms(1); // function will run at 1kHz maximum.
  return(buttonStatus);
}


// Description:   Check position of trim potentiometer
// Return value:  uint16_t, Flushing time [ms]
// Comments:      Trip-pot has 270 degree travel
uint16_t SetFlushingTime() {
  RESET_ADC_RESULT; // reset ADC flag
  _delay_ms(1);     // this might not be necessary. evaluate!!
  START_ADC_CONVERSION;

  // Wait until conversion is completed
  while (!(ADC_CONVERSION_COMPLETED)) {}

  // Get result from
  uint16_t tmp = READ_ADC_RESULT;
  tmp >>= 1;                    // convert to 7 bit value (0-127)
  if (tmp < 8)   { tmp = 8; }   // verfiy minimum value
  if (tmp > 119) { tmp = 119; } // verify maximum value
  tmp -= 8;                     // tmp range is now [0-111]
  tmp *= 100;                   // Convert range from [0-111] to [0-11100]
  tmp += TIME_MIN_FLUSHING;     // Add minimum flushing time to result.
  return tmp;
}


// Description:   wait requested amount of time [ms]
// Return value:  None
// Comments:      Not very accurate! How accurate??
void Delay(uint16_t ms) {
  while (ms--) {
    _delay_ms(1);
  }
}


int main(void) {
  SetupIO();
  SetupADC();
  //SetupTimers();
  //SetFlushingTime();
  //sei(); // enable global interrupts.

  // Don't be hasty master Meriadoc!
  // Let everything stabilize after a black-out.
  Delay(3000);

  // Loop starts here
  //TIMER1_RESET;
  while (1) {
    // Wait for button to be pressed
    if (ReadPushbutton()) {
      // Check position of flushing time trim potentiometer
      uint16_t flushingTime = SetFlushingTime();

      Delay(TIME_INITIAL_DELAY);

      // Start water flushing
      COIL_FORWARD; // activate coil
      Delay(TIME_COIL_ACTIVE);
      COIL_BRAKE;   // deactivate coil

      Delay(flushingTime);

      // End water flushing
      COIL_REVERSE; // activate coil
      Delay(TIME_COIL_ACTIVE);
      COIL_BRAKE;   // deactivate coil
      Delay(100);   // discharge coil??
      COIL_COAST;   // deactivate coil

      // Additional end-of-sequence delay
      Delay(1900);

      // Wait for user to release pushbutton
      while (ReadPushbutton()) { }
    }
  }
}
