
// Reduce power consumption of an arduino.
// From https://thecavepearlproject.org/2015/11/05/a-diy-arduino-data-logger-build-instructions-part-4-power-optimization/
//
// Disable the digital input buffers you aren’t using with DIDR0
// Never leave floating pins – always use a pullup or pulldown resistor.
// Disable unused module clocks using the PRR register
// Especially the ADC with ADCSRA &= ~_BV(ADEN);
// Shut off the analog comparator: ACSR |=_BV(ACD);

#include <avr/power.h>

// from  http://heliosoph.mit-links.info/arduino-powered-by-capacitor-reducing-consumption/
//defines for DIDR0 setting
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// create variables to restore the SPI & ADC register values after shutdown
byte keep_ADCSRA;
byte keep_SPCR;

// then be sure to store the default register values during setup:
keep_ADCSRA = ADCSRA;
keep_SPCR=SPCR; 

//  1) where the ADC pins are being used, disable the digital input buffers 
//  from http://www.instructables.com/id/Girino-Fast-Arduino-Oscilloscope/step10/Setting-up-the-ADC/

sbi(DIDR0,ADC0D);   
sbi(DIDR0,ADC1D);   
sbi(DIDR0,ADC2D);   
//sbi(DIDR0,ADC3D);  //A3 not used as analog in
//sbi(DIDR0,ADC4D);  //A4= I2C data
//sbi(DIDR0,ADC5D);  //A5= I2C scl
//not needed for A6&A7 because they have no digital capability
//2) set unused analog pins to digital input & pullup to prevent floating

pinMode(A3, INPUT); digitalWrite(A3, HIGH);

//3) And pull up any unused digital pins:
pinMode(pin#, INPUT_PULLUP);

// Disable internal peripherals that you are not using:
// Note to self: Don’t mess with timer 0!
power_timer1_disable();   // (0.12mA) controls PWM 9 & 10 , Servo library
power_timer2_disable();   // (0.12mA) controls PWM 3 & 11
power_adc_disable();      // disable the clock to the ADC module 
ADCSRA = 0;               // disable ADC by setting ADCSRA reg. to 0  (0.23mA)
ACSR = B10000000;         // disable comparator by setting ACD bit7 of ACSR reg. to one.
 // from http://www.fiz-ix.com/2012/01/introduction-to-arduino-interrupts-and-the-atmega328-analog-comparator/
power_spi_disable();                   // disable the clock to the SPI module
SPCR = 0;                                        //disable SPI by setting SPCR register to 0 (0.15mA)

#ifndef ECHO_TO_SERIAL
power_usart0_disable();   //(0.08mA) but only if your are not sending any output to the serial monitor!
#endif

// power_twi_disable();    // (0.18mA) I don’t usually turn off I2C because I use the bus frequently
// also see: https://harizanov.com/2013/02/power-saving-techniques-on-the-atmega32u4/
// Shutting down ADC & SPI requires you to wake them up again if you need them:
// Before any SD card data saving:
power_spi_enable(); SPCR=keep_SPCR;  delay(1); 
// Before ADC reading:
power_adc_enable(); ADCSRA = keep_ADCSRA;  // & throw away the first reading or two…void setup() {
  // put your setup code here, to run once:

void loop() {
  // put your main code here, to run repeatedly:

}
