/* MusselGapeTracker.cpp
	A set of functions used with the MusselGapeTracker project
	on RevC hardware (ATmega328p)
	Luke Miller June 2017

*/


#include "MusselGapeTrackerlib.h"

//---------------ShiftReg------------------------
ShiftReg::ShiftReg(){}
ShiftReg::~ShiftReg(){}

void ShiftReg::begin(uint8_t CS_SHIFT_REG, uint8_t SHIFT_CLEAR){
	m_CS_SHIFT_REG = CS_SHIFT_REG; // aka LATCH pin
	m_SHIFT_CLEAR = SHIFT_CLEAR;
	// Set pinMode to output
	pinMode(m_CS_SHIFT_REG, OUTPUT);
	pinMode(m_SHIFT_CLEAR, OUTPUT);
  // To use SHIFT_CLEAR, set it low, then pull CS_SHIFT_REG high,
  // and then set SHIFT_CLEAR high. 
  // Initially clear the shift registers to put all hall 
  // effect chips to sleep (they sleep when their sleep pin
  // is pulled low). Do this by pulling SHIFT_CLEAR low
  // while CS_SHIFT_REG is low, then send CS_SHIFT_REG high
  // to trigger the clear. 
  digitalWrite(m_SHIFT_CLEAR, HIGH);
  digitalWrite(m_CS_SHIFT_REG, LOW);
  digitalWrite(m_SHIFT_CLEAR, LOW);
  digitalWrite(m_CS_SHIFT_REG, HIGH);
  digitalWrite(m_SHIFT_CLEAR, HIGH); // reset high
}

uint16_t ShiftReg::shiftChannelSet (uint8_t channel) {
    // Send a signal to the appropriate shift register
    // channel to go high (set 1) to wake that hall sensor
    digitalWrite(m_CS_SHIFT_REG, LOW); // Set low to begin entering register values
    // Calculate the appropriate hex value to put a 1 in 
    // the correct channel's location (bit 0-15)
    // Do this by taking a hex 1 and left-shifting it the 
    // appropriate number of bits. To turn on Hall 0, you 
    // need a 1 in bit 0, to turn on Hall 15, you need
    // a 1 in bit 15 position.
    uint16_t hexChannel = 0x01 << channel;
    // Now split into lowByte and highByte and send them both
    // in order
    SPI.transfer(highByte(hexChannel)); // 
    SPI.transfer(lowByte(hexChannel)); // 
    digitalWrite(m_CS_SHIFT_REG, HIGH); // Set high to LATCH register values
    return hexChannel; 
} // end of shiftChannelSet


void ShiftReg::clear(void){
  // To use SHIFT_CLEAR, set it low, then pull CS_SHIFT_REG high,
  // and then set SHIFT_CLEAR high. 
//  digitalWrite(m_SHIFT_CLEAR, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(m_CS_SHIFT_REG, LOW);
//  digitalWrite(m_SHIFT_CLEAR, LOW);
//  delayMicroseconds(10);
//  digitalWrite(m_CS_SHIFT_REG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(m_SHIFT_CLEAR, HIGH); // reset high

	// Set shift register LATCH pin low to begin clocking in bits
    digitalWrite(m_CS_SHIFT_REG, LOW);
    uint16_t hexChannel = 0x00 ; // send all 0's to clear all channels
    // Now split into lowByte and highByte and send them both
    // in order
    SPI.transfer(highByte(hexChannel)); // 
    SPI.transfer(lowByte(hexChannel)); // 
    digitalWrite(m_CS_SHIFT_REG, HIGH); // Pull high to set LATCH
  
} // end of clear() function


//-------------Mux----------------------------------
Mux::Mux(){}
Mux::~Mux(){}

void Mux::begin(uint8_t MUX_EN, uint8_t MUX_S0, uint8_t MUX_S1, uint8_t MUX_S2, uint8_t MUX_S3) {
	m_MUX_EN = MUX_EN;
	m_MUX_S0 = MUX_S0;
	m_MUX_S1 = MUX_S1;
	m_MUX_S2 = MUX_S2;
	m_MUX_S3 = MUX_S3;
	// Set the pinModes
	pinMode(m_MUX_EN, OUTPUT);
	pinMode(m_MUX_S0, OUTPUT);
	pinMode(m_MUX_S1, OUTPUT);
	pinMode(m_MUX_S2, OUTPUT);
	pinMode(m_MUX_S3, OUTPUT);
	
	digitalWrite(m_MUX_EN, LOW); // enable mux by setting LOW
}

void Mux::muxChannelSet (byte channel) {
  // select correct MUX channel
  digitalWrite (m_MUX_S0, (channel & 1) ? HIGH : LOW);  // low-order bit
  digitalWrite (m_MUX_S1, (channel & 2) ? HIGH : LOW);
  digitalWrite (m_MUX_S2, (channel & 4) ? HIGH : LOW);  
  digitalWrite (m_MUX_S3, (channel & 8) ? HIGH : LOW);  // high-order bit
}  // end of muxChannelSet




void printTimeSerial(DateTime now){
//------------------------------------------------
// printTime function takes a DateTime object from
// the real time clock and prints the date and time 
// to the serial monitor. 
	Serial.print(now.year(), DEC);
    Serial.print('-');
	if (now.month() < 10) {
		Serial.print(F("0"));
	}
    Serial.print(now.month(), DEC);
    Serial.print('-');
    if (now.day() < 10) {
		Serial.print(F("0"));
	}
	Serial.print(now.day(), DEC);
    Serial.print(' ');
	if (now.hour() < 10){
		Serial.print(F("0"));
	}
    Serial.print(now.hour(), DEC);
    Serial.print(':');
	if (now.minute() < 10) {
		Serial.print("0");
	}
    Serial.print(now.minute(), DEC);
    Serial.print(':');
	if (now.second() < 10) {
		Serial.print(F("0"));
	}
    Serial.print(now.second(), DEC);
	// You may want to print a newline character
	// after calling this function i.e. Serial.println();

}

//---------------printTimeToSD----------------------------------------
// printTimeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void printTimeToSD (SdFile& mylogfile, DateTime tempTime) {
    // Write the date and time in a human-readable format
    // to the file on the SD card. 
    mylogfile.print(tempTime.year(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.month() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.month(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.day() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.day(), DEC);
    mylogfile.print(F(" "));
    if (tempTime.hour() < 10){
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.hour(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.minute() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.minute(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.second() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.second(), DEC);
}

//--------------------goToSleep-----------------------------------------------
// goToSleep function. When called, this puts the AVR to
// sleep until it is awakened by an interrupt (TIMER2 in this case)
// This is a higher power sleep mode than the lowPowerSleep function uses.
void goToSleep() {
  // Create three variables to hold the current status register contents
  byte adcsra, mcucr1, mcucr2;
  // Cannot re-enter sleep mode within one TOSC cycle. 
  // This provides the needed delay.
  OCR2A = 0; // write to OCR2A, we're not using it, but no matter
  while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated
  // Set the sleep mode to PWR_SAVE, which allows TIMER2 to wake the AVR
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  adcsra = ADCSRA; // save the ADC Control and Status Register A
  ADCSRA = 0; // disable ADC by zeroing out the ADC status register
  sleep_enable();
  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  noInterrupts ();
  
  // wdt_disable(); // turn off the watchdog timer
  
  //ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
  ATOMIC_BLOCK(ATOMIC_FORCEON) { 
    // Turn off the brown-out detector
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); 
    mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1; //timed sequence
    // BODS stays active for 3 cycles, sleep instruction must be executed 
    // while it's active
    MCUCR = mcucr2; 
  }
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts();  // one cycle, re-enables interrupts
  sleep_cpu(); //go to sleep
  //wake up here
  sleep_disable(); // upon wakeup (due to interrupt), AVR resumes here
  // watchdogSetup(); // re-enable watchdog timer
  ADCSRA = adcsra; // re-apply the previous settings to the ADC status register

}


//-------------- initFileName --------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter. 
// The character array 'filename' was defined as a global array 
// at the top of the sketch in the form "YYYYMMDD_HHMM_00.csv"
void initFileName(SdFat& sd, SdFile& logfile, DateTime time1, char *filename, bool serialValid, char *serialNumber) {
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filename array
	for (byte i = 0; i < 4; i++){
		filename[i] = buf[i];
	}
	// Insert the month value
	if (time1.month() < 10) {
		filename[4] = '0';
		filename[5] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filename[4] = (time1.month() / 10) + '0';
		filename[5] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filename[6] = '0';
		filename[7] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filename[6] = (time1.day() / 10) + '0';
		filename[7] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filename[8] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filename[9] = '0';
		filename[10] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filename[9] = (time1.hour() / 10) + '0';
		filename[10] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filename[11] = '0';
		filename[12] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filename[11] = (time1.minute() / 10) + '0';
		filename[12] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filename[13] = '_';
	// If there is a valid serialnumber, insert it into 
	// the file name in positions 17-20. 
	if (serialValid) {
		byte serCount = 0;
		for (byte i = 17; i < 21; i++){
			filename[i] = serialNumber[serCount];
			serCount++;
		}
	}
	// Next change the counter on the end of the filename
	// (digits 14+15) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[14] = i / 10 + '0';
		filename[15] = i % 10 + '0';
		
		if (!sd.exists(filename)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
//				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
//				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(5);
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	//------------------------------------------------------------
  // Write 1st header line
  logfile.print(F("POSIXt,DateTime"));
  for (byte i = 0; i < 16; i++){
	  // Cycle through channels to create headers for each column
	logfile.print(F(",Hall"));
	logfile.print(i);
  }
  logfile.print(F(",Battery.V"));
  logfile.println();
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close(); // force the data to be written to the file by closing it
} // end of initFileName function


//---------- startTIMER2 ----------------------------------------------------
// startTIMER2 function
// Starts the 32.768kHz clock signal being fed into XTAL1 from the
// real time clock to drive the
// quarter-second interrupts used during data-collecting periods. 
// Supply a current DateTime time value, the real time clock object, and
// a sample per second value (SPS) of 1, 2 , or 4
// This function returns a DateTime value that can be used to show the 
// current time when returning from this function. 
DateTime startTIMER2(DateTime currTime, RTC_DS3231& rtc, byte SPS){
	TIMSK2 = 0; // stop timer 2 interrupts

	rtc.enable32K();  // updated to newer RTClib version of function
	ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
	// The EXCLK bit should only be set if you're trying to feed the
	// 32.768 clock signal from the Chronodot into XTAL1. 

	ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid
	// clobbering the EXCLK bit that might already be set. This tells 
	// TIMER2 to take its clock signal from XTAL1/2
	TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
	
	// Set up TCCR2B register (Timer Counter Control Register 2 B) to use the 
	// desired prescaler on the external 32.768kHz clock signal. Depending on 
	// which bits you set high among CS22, CS21, and CS20, different 
	// prescalers will be used. See Table 18-9 on page 158 of the AVR 328P 
	// datasheet.
	//  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
	// no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)
	//  TCCR2B = _BV(CS20) ; 
	// prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
    if (SPS == 16){
	  TCCR2B = _BV(CS21) ;
    } else if (SPS == 4){
		// prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
		TCCR2B = _BV(CS21) | _BV(CS20); 
	} else if (SPS == 2) {
		TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
	} else if (SPS == 1){
		TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 second
	}


	// Pause briefly to let the RTC roll over a new second
	DateTime starttime = currTime;
	// Cycle in a while loop until the RTC's seconds value updates
	while (starttime.second() == currTime.second()) {
		delay(1);
		currTime = rtc.now(); // check time again
	}

	TCNT2 = 0; // start the timer at zero
	// wait for the registers to be updated
	while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} 
	TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
	TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
	// TIMER2 will now create an interrupt every time it rolls over,
	// which should be every 0.0625, 0.25, 0.5 or 1 seconds (depending on value
	// of SPS "SAMPLES_PER_SECOND") regardless of whether the AVR is awake or asleep.
	return currTime;
}


unsigned int readHall(byte ANALOG_IN){
	unsigned int rawAnalog = 0;
	analogRead(ANALOG_IN); // throw away 1st reading
	for (byte i = 0; i<4; i++){
	  rawAnalog = rawAnalog + analogRead(ANALOG_IN);
	  delay(1);
	}
	// Do a 2-bit right shift to divide rawAnalog
	// by 4 to get the average of the 4 readings
	rawAnalog = rawAnalog >> 2;   
	return rawAnalog;
}

void read16Hall(byte ANALOG_IN, unsigned int *hallAverages, ShiftReg& shiftReg, Mux& mux, uint8_t MUX_EN){
          digitalWrite(MUX_EN, LOW); // enable multiplexer
		  delayMicroseconds(2);
		  for (byte ch = 0; ch < 16; ch++){
              // Cycle through each channel
              //-------------------------------------------
              // Call function to set shift register bit to wake
              // appropriate sensor
              shiftReg.shiftChannelSet(ch); 
              //----------------------------------------------
              mux.muxChannelSet(ch); // Call function to set address bits             
              //----------------------------------------------
              // Take 4 analog readings from the same channel, average + store them
              hallAverages[ch] = readHall(ANALOG_IN);         
          }
		  shiftReg.clear();
		  digitalWrite(MUX_EN, HIGH); // disable multiplexer by pulling EN pin high
	
}



void OLEDscreenUpdate (byte ScreenNum, unsigned int *hallAverages, unsigned int *prevAverages, SSD1306AsciiWire& oled1, byte I2C_ADDRESS1, bool updateAll){

			  byte chIndex;
			  switch (ScreenNum){
				case 0:
					chIndex = 0; // screen 0 starts with channel 0
				break;
				case 1:
					chIndex = 4; // screen 1 starts with channel 4
				break;
				case 2:
					chIndex = 8; // screen 2 starts with channel 8
				break;
				case 3:
					chIndex = 12; // screen 3 starts with channel 12
				break;
			  }
			  byte chIndex2 = chIndex + 4; 
			  if (updateAll){
				  // Update all of the Ch labels and Hall sensor values
				  oled1.home();
				  oled1.clear();
				  oled1.set2X();
				  for (byte r = chIndex; r < chIndex2; r++){
					oled1.print(F("Ch"));
					oled1.print(r);
					oled1.print(F(": "));
					oled1.setCol(72);
					oled1.println(hallAverages[r]);
				  }				  
			  } else {
				  // Don't update the Chx: text, only changed Hall values
				  oled1.home();
				  oled1.set2X();
				  for (byte r = chIndex; r < chIndex2; r++){
					if (hallAverages[r] == prevAverages[r]){
						// Don't update if the value hasn't changed
						oled1.println();
					}  else {
						// Update when value has changed
						oled1.clear(72,128,oled1.row(), (oled1.row()+1)); 
						oled1.println(hallAverages[r]);
					}
				  }
			  }
			

}


//-----------------------------------------------------------------------------
// lowPowerSleep function
// This sleep version uses the watchdog timer to sleep for 8 seconds at a time

void lowPowerSleep(void){

	/* It seems to be necessary to zero out the Asynchronous clock status 
	 * register (ASSR) before enabling the watchdog timer interrupts in this
	 * process. 
	 */
	ASSR = 0;  
	TIMSK2 = 0; // stop timer 2 interrupts
	// Cannot re-enter sleep mode within one TOSC cycle. 
	// This provides the needed delay.
	OCR2A = 0; // write to OCR2A, we're not using it, but no matter
	while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated

	ADCSRA = 0;   // disable ADC
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // specify sleep mode
	sleep_enable();
	// Do not interrupt before we go to sleep, or the
	// ISR will detach interrupts and we won't wake.
	noInterrupts ();
	//--------------------------------------------------------------------------
	// Set up Watchdog timer for long term sleep

	// Clear the reset flag first
	MCUSR &= ~(1 << WDRF);

	// In order to change WDE or the prescaler, we need to
	// set WDCE (This will allow updates for 4 clock cycles).
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	// Enable watchdog interrupt (WDIE), and set 8 second delay
	WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); 
	wdt_reset();

	// Turn off brown-out enable in software
	// BODS must be set to one and BODSE must be set to zero within four clock 
	// cycles, see section 10.11.2 of 328P datasheet
	MCUCR = bit (BODS) | bit (BODSE);
	// The BODS bit is automatically cleared after three clock cycles
	MCUCR = bit (BODS);
	// We are guaranteed that the sleep_cpu call will be done
	// as the processor executes the next instruction after
	// interrupts are turned on.
	interrupts ();  // one cycle, re-enables interrupts
	sleep_cpu ();   // one cycle, going to sleep now, wake on interrupt
	// The AVR is now asleep. In SLEEP_MODE_PWR_DOWN it will only wake
	// when the watchdog timer counter rolls over and creates an interrupt
	//-------------------------------------------------------------------
	// disable sleep as a precaution after waking
	sleep_disable();
}

//-------------- checkMCUSR ------------------------------------------------
// Function checkMCUSR(byte mcusr, byte ERRLED)
// This function is supplied a byte value that represents the contents of the
// MCU status register. On a restart, this register's 4 lowest bits contain
// flags noting what might have caused the restart. If the restart was due
// to a brown-out, we want to go into a permanent sleep mode to avoid 
// corrupting the SD card. 
void checkMCUSR(byte mcusr, byte ERRLED) 
{	
	//----------------------------------------------------------------------
	// Check the MCU Status Register (MCUSR) to see why the
	// program has started/restarted. See section 11.9.1 of the AVR datasheet.
	// Check if the Brown-Out Reset Flag is 1 by &-ing MCUSR with a 1 in
	// the BORF position. MCUSR and BORF are defined internally in the
	// various avr libraries. 
	if (mcusr & _BV(BORF) ) { 
		// Check to see if the external reset flag EXTRF was set. If it 
		// is set, this next test will fail and normal bootup will proceed.
		if (!(mcusr & _BV(EXTRF))){	
			// Now check if the Power-On Reset Flag is 0. A full power-on
			// reset will usually have both BORF and PORF true, but a 
			// brown-out when already powered-up will not have PORF true. 
			if (!(mcusr & _BV(PORF))){ 
				// Uh-oh, PORF was not true, so we only have a BORF flag.
				// This likely signals that the power supply is getting low
				// enough that it is hitting the brown-out level (~2.7v).
				// In this case, we should put the unit into a permanent
				// sleep to avoid corrupting the SD card.

				// Go into a permanent sleep loop. This cannot be exited 
				// without a reset of some sort. Continued brown-outs should
				// simply end up back here. The REDLED should blink briefly
				// every 8 seconds to indicate that the program has ended 
				// up in this brown-out-induced loop.
				while(1){
					digitalWrite(ERRLED, HIGH);
					delay(3);
					digitalWrite(ERRLED, LOW);
					lowPowerSleep();  // go back to sleep
				}
			}
		}
	}
}

//-------------- printBits -----------------------------------------
// A simple function to print out a byte as 1's and 0's to Serial
void printBits(byte myByte){
	for(byte mask = 0x80; mask; mask >>= 1){
		if(mask  & myByte){
		   Serial.print('1');
		} else {
		   Serial.print('0');
		}
	}
}

//-----------printBitsOLED-----------------
// Function to print out a byte at 1's and 0's to an OLED display
void printBitsOLED(byte myByte, SSD1306AsciiWire& oled1){
	for(byte mask = 0x80; mask; mask >>= 1){
		if(mask  & myByte){
		   oled1.print('1');
		} else {
		   oled1.print('0');
		}
	}
}



//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage(byte BATT_MONITOR_EN, byte BATT_MONITOR, float dividerRatio, float refVoltage){
    // Turn on the battery voltage monitor circuit
    digitalWrite(BATT_MONITOR_EN, HIGH);
    delay(1);
    // Read the analog input pin
    unsigned int rawAnalog = 0;
    analogRead(BATT_MONITOR); // This initial value is ignored
    delay(3); // Give the ADC time to stablize
    // Take 4 readings
    for (byte i = 0; i<4; i++){
        rawAnalog = rawAnalog + analogRead(BATT_MONITOR);
        delay(2);
    }
    // Do a 2-bit right shift to divide rawAnalog
    // by 4 to get the average of the 4 readings
    rawAnalog = rawAnalog >> 2;
    // Shut off the battery voltage sense circuit
    digitalWrite(BATT_MONITOR_EN, LOW);
    // Convert the rawAnalog count value (0-1024) into a voltage
    // Relies on global variables dividerRatio and refVoltage
    float reading = rawAnalog * dividerRatio * refVoltage / 1024;
    return reading; // return voltage result
}
 
//-----------printHallToOLED--------------------------------------------------
// Update temperature values on the OLED screen. This function only updates
// elements that have changed, and leaves the rest of the screen static. 
/* void printHallToOLEDs (SSD1306AsciiWire& oled1, SSD1306AsciiWire& oled2, double *hallAverages, double *prevAverages){
	// Print stuff to screens
          oled1.home();
          oled1.set2X();
          //oled1.clearToEOL();
          for (byte j = 0; j < 4; j++){
            // Check to see if the value has changed and
            // only update if it's changed, and check that the 
			// new value isn't also NAN. 
			if ( isnan(prevAverages[j]) & isnan(tempAverages[j]) ){
				// If previous value was nan and current value
				// value is also nan, no need to update
				oled1.setRow(oled1.row()+2); // move to next row
			} else if ( (tempAverages[j] != prevAverages[j]) & !isnan(prevAverages[j]) ){
				// If old and new values don't match, and the previous value
				// wasn't nan, then the display value needs to be updated
              oled1.clear(60,128,oled1.row(),(oled1.row()+1));
              oled1.print(tempAverages[j]);
              oled1.println(F("C"));
            } else {
			   // If the two values match exactly, no update needed.
              // Skip to next row. At 2x size, each row is 2units tall
              oled1.setRow(oled1.row()+2);
            }
          }
          oled2.home();
          oled2.set2X();
          for (byte j = 4; j < 8; j++){
            // Check to see if the value has changed and
            // only update if it's changed, and check that the 
			// new value isn't also NAN. 
			if ( isnan(prevAverages[j]) & isnan(tempAverages[j]) ){
				// If previous value was nan and current value
				// value is also nan, no need to update
				oled2.setRow(oled2.row()+2); // move to next row
			} else if ( (tempAverages[j] != prevAverages[j]) & !isnan(prevAverages[j]) ){
				// If old and new values don't match, and the previous value
				// wasn't nan, then the display value needs to be updated
              oled2.clear(60,128,oled2.row(),(oled2.row()+1));
              oled2.print(tempAverages[j]);
              oled2.println(F("C"));
            } else {
			   // If the two values match exactly, no update needed.
              // Skip to next row. At 2x size, each row is 2units tall
              oled2.setRow(oled2.row()+2);
            }
          } 
}  


 */