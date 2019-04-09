/* Two-Digit Thermometer

   David Johnson-Davies - www.technoblogy.com - 9th April 2019
   ATtiny84 @ 8 MHz (internal oscillator; BOD disabled)
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

#include <avr/sleep.h>
#include <avr/power.h>

// Comment out for a common anode display
#define commoncathode

// Seven-segment definitions
uint8_t charArray[] = {
//  ABCDEFG  Segments
  0b1111110, // 0
  0b0110000, // 1
  0b1101101, // 2
  0b1111001, // 3
  0b0110011, // 4
  0b1011011, // 5
  0b1011111, // 6
  0b1110000, // 7
  0b1111111, // 8
  0b1111011, // 9
  0b0000000, // 10  Space
  0b0000001, // 11  '-'
  0b0110001, // 12  '-1'
  0b1001111, // 13  'E'
  0b0001110, // 14  'L'
  0b0011101, // 15  'o'
  0b0110111, // 16  'H'
  0b0010000, // 17  'i'
  0x80,      // 18  '.'
};

const int Blank = 10;
const int Minus = 11;
const int Minus1 = 12;
const int Error = 13;
const int Lo = 14;
const int Hi = 16;
const int DP = 18;

volatile char Buffer[] = {2, 2};

// One Wire Protocol **********************************************

// Buffer to read data or ROM code
static union {
  uint8_t DataBytes[9];
  unsigned int DataWords[4];
};

const int Digit0Pin = 0;              // PB0
const int Digit1Pin = 1;              // PB1
const int OneWirePin = 2;             // PB2

const int ReadROM = 0x33;
const int MatchROM = 0x55;
const int SkipROM = 0xCC;
const int ConvertT = 0x44;
const int ReadScratchpad = 0xBE;

inline void PinLow () {
  DDRB = DDRB | 1<<OneWirePin;
}

inline void PinRelease () {
  DDRB = DDRB & ~(1<<OneWirePin);
}

// Returns 0 or 1
inline uint8_t PinRead () {
  return PINB>>OneWirePin & 1;
}

void DelayMicros (unsigned int micro) {
  TCNT1 = 0; TIFR1 = 1<<OCF1A;
  OCR1A = micro;
  while ((TIFR1 & 1<<OCF1A) == 0);
}

void LowRelease (int low, int high) {
  PinLow();
  DelayMicros(low);
  PinRelease();
  DelayMicros(high);
}

void OneWireSetup () {
  TCCR1A = 0<<WGM10;                      // Normal mode
  TCCR1B = 0<<WGM12 | 2<<CS10;            // Normal mode, divide clock by 8
}

uint8_t OneWireReset () {
  uint8_t data = 1;
  LowRelease(480, 70);
  data = PinRead();
  DelayMicros(410);
  return data;                            // 0 = device present
}

void OneWireWrite (uint8_t data) {
  int del;
  for (int i = 0; i<8; i++) {
    if ((data & 1) == 1) del = 6; else del = 60;
    LowRelease(del, 70 - del);
    data = data >> 1;
  }
}

uint8_t OneWireRead () {
  uint8_t data = 0;
  for (int i = 0; i<8; i++) {
    LowRelease(6, 9);
    data = data | PinRead()<<i;
    DelayMicros(55);
  }
  return data;
}

// Read bytes into array, least significant byte first
void OneWireReadBytes (int bytes) {
  for (int i=0; i<bytes; i++) {
    DataBytes[i] = OneWireRead();
  }
}

// Calculate CRC over buffer - 0x00 is correct
uint8_t OneWireCRC (int bytes) {
  uint8_t crc = 0;
  for (int j=0; j<bytes; j++) {
    crc = crc ^ DataBytes[j];
    for (int i=0; i<8; i++) crc = crc>>1 ^ ((crc & 1) ? 0x8c : 0);
  }
  return crc;
}

// Display multiplexer **********************************************

uint8_t digit = 0;

void DisplayNextDigit () {
#if defined(commoncathode)
  PORTB = PORTB | 1<<digit;               // Turn old digit off
  digit = digit ^ 1;                      // Toggle between 0 and 1
  char segs = charArray[Buffer[digit]];
  PORTA = segs;                           // Lit segments high
  PORTB = PORTB & ~(1<<digit);            // Turn new digit on
#else
  PORTB = PORTB & ~(1<<digit);            // Turn old digit off
  digit = digit ^ 1;                      // Toggle between 0 and 1
  char segs = charArray[Buffer[digit]];
  PORTA = ~segs;                          // Lit segments low
  PORTB = PORTB | 1<<digit;               // Turn new digit on
#endif
}

// Display a two-digit number
void Display (int n) {
  int units = n % 10;
  int tens = n / 10;
  int temp0 = tens;
  int temp1 = abs(units);
  if (tens < -1) {temp0 = Lo; temp1 = Lo+1; }
  else if (tens > 9) {temp0 = Hi; temp1 = Hi+1; }
  else if (tens == -1) temp0 = Minus1;
  else if ((tens == 0) && (units >= 0)) temp0 = Blank;
  else if ((tens == 0) && (units < 0)) temp0 = Minus;
  Buffer[0] = temp0;
  Buffer[1] = temp1;
}

// Display Error
void DisplayError (int no) {
  Buffer[0] = Error;
  Buffer[1] = no;
}

volatile uint8_t Ticks = 0;

// Flash the display on for n ticks
void DisplayOn (uint8_t n) {
  Ticks = n;
  TIMSK0 = 1<<OCIE0A;                     // Compare match interrupt
  while (Ticks > 0);
  TIMSK0 = 0;                             // Interrupts off
#if defined(commoncathode)
  PORTB = PORTB | 1<<Digit1Pin | 1<<Digit0Pin;    // Both digits off
#else
  PORTB = PORTB & ~(1<<Digit1Pin | 1<<Digit0Pin); // Both digits off
#endif
}

// Timer/Counter0 interrupt - multiplexes display - 125Hz
ISR(TIM0_COMPA_vect) {
  DisplayNextDigit();
  Ticks--;
}

// Temperature sensor **********************************************

// Display temperature of a single DS18B20 on the bus in 1/16ths of a degree
void DisplayTemperature () {
  cli();                                  // No interrupts
  if (OneWireReset() != 0) {
    sei();
    DisplayError(0);                      // Device not found
  } else {
    OneWireWrite(SkipROM);
    OneWireWrite(ConvertT);
    while (OneWireRead() != 0xFF);
    OneWireReset();
    OneWireWrite(SkipROM);
    OneWireWrite(ReadScratchpad);
    OneWireReadBytes(9);
    sei();                                // Interrupts
    if (OneWireCRC(9) == 0) {
      int temp = DataWords[0];
      Display((temp+8)>>4);               // Round to nearest degree
    } else DisplayError(1);               // CRC error
  }
}

// Watchdog timer **********************************************

// Use Watchdog for time delay; n=0 is 16ms; n=6 is 1sec ; n=9 is 8secs, 
void WDDelay(int n) {
  WDTCSR = 1<<WDIE | (n & 0x8)<<2 | (n & 0x7);
  sleep_enable();
  sleep_cpu();
}

ISR(WDT_vect) {
  WDTCSR = 0<<WDIE;
}

// Setup **********************************************

void setup () {
  DDRA = 0xFF;                            // All outputs
  DDRB = 1<<Digit1Pin | 1<<Digit0Pin;     // PB0 & PB1 outputs
#if defined(commoncathode)
  PORTB = 0<<OneWirePin | 1<<Digit1Pin | 1<<Digit0Pin; // Both digits off
#else
  PORTB = 0<<OneWirePin | 0<<Digit1Pin | 0<<Digit0Pin; // Both digits off
#endif
  OneWireSetup();
  //
  // Set up Timer/Counter0 to multiplex the display
  TCCR0A = 2<<WGM00;                      // CTC mode; count up to OCR0A
  TCCR0B = 0<<WGM02 | 4<<CS00;            // Divide by 256
  OCR0A = 250-1;                          // Compare match at 125Hz
  TIMSK0 = 0;                             // Interrupts initially off
  //
  ADCSRA &= ~(1<<ADEN);                   // Disable ADC to save power
  PRR = 1<<PRUSI | 1<<PRADC;              // Turn off clocks to USI & ADC to save power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop () {
  Buffer[0] = DP; Buffer[1] = Blank;
  DisplayOn(12);
  WDDelay(6);                             // Sleep for 1 second
  Buffer[0] = Blank; Buffer[1] = DP;
  DisplayOn(12);
  WDDelay(6);                             // Sleep for 1 second
  DisplayTemperature();
  DisplayOn(12);
  WDDelay(9);                             // Sleep for 8 seconds
  WDDelay(9);                             // Sleep for 16 seconds
  WDDelay(9);                             // Sleep for 24 seconds
}
