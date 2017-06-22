/*  MusselGapeTrackerlib.h
	A set of functions used with the MusselGapeTracker project
	on RevA hardware (ATmega328p)
	Luke Miller June 2017

*/

#ifndef MusselGapeTracker_H
#define MusselGapeTracker_H

#include <Arduino.h> // to get access to pinMode, digitalRead etc functions
#include "SdFat.h"	// https://github.com/greiman/SdFat
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
// Various additional libraries for access to sleep mode functions
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
#include <math.h>

class ShiftReg {
	public:
		ShiftReg();
		~ShiftReg();
		void begin(byte CS_SHIFT_REG, byte SHIFT_CLEAR); // set up pinMode for the sensor
		word shiftChannelSet(byte channel); // Pull single channel high		
		void clear(void); // clear all channels (puts sensors to sleep)
		
	private: 
	uint8_t m_CS_SHIFT_REG;
	uint8_t m_SHIFT_CLEAR;
};

class Mux {
	public:
		Mux();
		~Mux();
		void begin(uint8_t MUX_EN, uint8_t MUX_S0, uint8_t MUX_S1, uint8_t MUX_S2, uint8_t MUX_S3); 
		void muxChannelSet(byte channel);
		
	private:
	uint8_t m_MUX_EN;
	uint8_t m_MUX_S0;
	uint8_t m_MUX_S1;
	uint8_t m_MUX_S2;
	uint8_t m_MUX_S3;
	
};


//--------- Public functions


// Print formatted Date + Time to Serial monitor
void printTimeSerial(DateTime now); 

// Print formatted Date + Time to SD card csv file. Notice that this passes the
// SdFile object by reference (SdFile& mylogfile) instead of making a copy and
// passing by value (which SdFile mylogfile would do).
void printTimeToSD(SdFile& mylogfile, DateTime now); 

// Put the AVR to sleep until a TIMER2 interrupt fires to awaken it
void goToSleep();

// Initialize a new output csv file. Note that this writes a header row
// to the file, so you may want to tweak the column labels in this function.
void initFileName(SdFat& sd, SdFile& logfile, DateTime time1, char *filename, bool serialValid, char *serialNumber);

// Start the TIMER2 timer, using a 32.768kHz input from a DS3231M 
// real time clock as the signal. 
DateTime startTIMER2(DateTime currTime, RTC_DS3231& rtc, byte SPS);

// Take a set of 4 readings on the analog input, average the values, and 
// return the average value as an unsigned int (should be 0 to 1023)
unsigned int readHall(byte ANALOG_IN);

// Read 16 Hall effect channels at once, updating values in the 16-element
// array hallAverages. Also supply the shift register object and multiplexer
// object
void read16Hall(byte ANALOG_IN, unsigned int *hallAverages, ShiftReg& shiftReg, Mux& mux);

// Show 4 channels of Hall measurements on the screen.
void OLEDscreenUpdate (byte ScreenNum, unsigned int *hallAverages, SSD1306AsciiWire& oled1,byte I2C_ADDRESS1);

// Set up watchdog timer
void watchdogSetup(void);


#endif