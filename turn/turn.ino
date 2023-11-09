// -*- c++ -*- 
/* This program is meant to be executed on a small PCB
 * measuring a person turning around itself and holding
 * the PCB away, facing the display.
 * Each rotation is measured and displayed. After
 * TURNMAX rotations, a code is revealed.
 *
 * Developed on an Arduino Pro Mini, deployed on a Attiny84.
 * PCB is the Open-V2 board that fits together with a 3.6V/2400mA Lithium
 * battery into a preform tube (15cm)
 *
 * New version (3.x) with ATtiny1634 and a dot matrix display on the OpenV4a board. 
 *  ATtiny Fusebits: Divide clock by 8, internal OSC., Brown-out disabled
 *  Arduino board: ATtiny 1634 (no bootloader) / 1MHz
 *
 * For calibration, we use the IMU_Zero sketch of the MPU6050 library! 
 *
 */

//#define DEBUG
//#define TESTING
#define VARIANT 0

/* Changelog:
 * V0.1 (16.1.2017)
 * - first version, able to display numbers
 * V1.0 (17.1.2017)
 * - I2C communication to MPU
 * - state machine
 * - maintenance output
 * - calibration values for MPU to be deployed with ATtiny
 * V1.1 (18.1.2017)
 * - pin defs for ATtiny
 * - char defs using | instead of +
 * - added software serial for debugging on ATtiny
 * - added myDelay(30) in getVcc to get right measurement
 * V1.2 (18.1.2017)
 * - no "game over", simply "bye"
 * - longer period for initial waiting
 * - longer violation interval
 * V1.3 (19.1.2017)
 * - gy_integrated is now set to 0 before the measurement in UPRIGHT_WAITING<-STATE (so the first part of 
 *   the turn is also taken into account!
 * - new OFF_STATE: after the initial waiting state, we just shut off (makes restart faster)
 * - added condition abs(gy) > TURN_UPTHRESH for going into turning state and
 *   abs(gy) < TURN_DOWNTHRESH for leaving turning state. This avoids the problem that with a slightly tilted 
 *   board, you are considered upright but still turning, even if you are standing still.
 * - upright limits were tightend to [750,850]
 * - voltage re-display is reduced to 4
 * - Schwachvolt-Nachricht statt Bye (bei schwacher Batterie).
 * V1.4 (26.1.2017)
 * - final coordinates
 * V1.5 (14.2.2017)
 * - added option for Schermbeck cache
 * V1.6 (21.3.17)
 * - changed int to long in displayNum
 * - added shutDownDevices 
 * - added  __attribute__((used)) to wdt_init
 * V1.7
 * - added further option: CANBERRA, and cleaned it up now: we have 4 different variants
 * V1.8 (28.5.18)
 * - new maint sequence & faster exit from maint sequence! 
 * V1.9 (14.9.19)
 * - no more maint mode! Now you need to readout the EEPROM!
 * V2.0 (25.12.19)
 * - using now the new IMU boards GY-521 (voltage regulator and LED removed) with new I2C-address!!!!
 * - using now TXOnlySerial for debugging
 * V2.1 (25.12.19)
 * - dead code (HEADOVERSTATE and displayInfo) removed.
 * - added displayVolt in order to calibrate (needs CALIBRATING to be defined!)
 * - added stat struct, which is stored in EEPROM (use Python script statturn.py to interpret)
 * V2.2 (26.12.19)
 * - added display info after power-up: display all important data when power-up
 * - changed maxturns to 4 (meaning 5 full turns)
 * V2.3 (24.1.22)
 * - added config folder
 * V2.4 (21.2.22)
 * - "bYE" replaced by "FAIL"
 * - stat is shown only once
 * - curr volt is displayed first
 * V2.5 (26.5.23)
 * - new MPU (with new calibration values) 
 * V3.0 (26.5.23)
 * - new version with dot matrix display
 * V3.1 (30.5.23)
 * - corrected all obvious bugs, but I2C does not start yet
 * V3.2
 * - switched MPU power on before i2cinit!
 * - new Bye-State and Fail-State
 * - flashing the dot (after sucess or error) is done using new font (flashdot)
 * - OFF_STATE only if never a digit was shown
 * - only 2 variants, "0" uses the calib.h file while "1" uses the inline values
 * - use now definitions from PETPreformBoard.h
 * V3.3
 * - disabled WDIE bit after wdt_disable call; seems to be necessary on an ATtiny1634!
 * - changed to laststatechange instead of lastBumped() -- leads to faster shutdown
 * - LONGTIMEOUT is now 30 instead of 20 seconds
 */

#define VERSION "3.3"

/* Crucial paramaters */
/**********************/
#define ERROR_REPMESS 3 // repeated display of error messages
#define SUCCESS_REPMESS 5 // repeated display of success messages
#define WEAKBATT_REPMESS 2
#define WEAKBATT_VOLT 2650
#define EMPTYBATT_VOLT 2500
#define MAXTURNS  4            // how often we have to turn around (+1)

// Exemplar specific values
#if (VARIANT == 0)
#include "config/calib.h"
#elif (VARIANT == 1)
// The success message
#define SUCCMESS "Erfolg!"
// depends on how the sensor is oriented 
#define YTRANSFORM(y) (y)
// needs to be calibrated using a multimeter
#define INTREFVOLTAGE 1100
// needs to be calibrated using MPU6050_calibration
#define XACCELOFF 0
#define YACCELOFF 1
#define ZACCELOFF 2
#define XGYROOFF 3
#define YGYROOFF 4
#define ZGYROOFF 5
#else
#error "Unknown variant"
#endif

// I2C address
#define MPUADDR 0xD0  // means 0x68 as a 7-bit addr : GY-521
// power and I2C pins for MPU
#define MPUPWR POWER
#define MPUSDA SDA
#define MPUSCL SCL

// IMU thresholds
#define UPRIGHT_UPTHRESH 850
#define UPRIGHT_DOWNTHRESH 750
#define INERTIA_UPTHRESH 400
#define INERTIA_DOWNTHRESH 300
#define LAYDOWN_UPTHRESH 800
#define TURN_UPTHRESH (18L*164L) // = 180*16.4 means 180°/s
#define TURN_DOWNTHRESH (12L*164L) // means 120°/s


// Timing constants
#define MAXSECONDS 15*60  // after this amount of seconds, we finally shut down (suspecting a problem)
#define DISPLAY_ON_MS 800
#define DISPLAY_OFF_MS 300
#define MPU_TIMEOUT_MS 500 // actually, there should be new measurement after 200ms!
#define MAX_LONGWAIT_MS 30000UL // wait this time after last movement before complaining
#define MAX_VIOLATION_MS 10000UL // after this amount of ms of violation, game is over
#define DEAD_MS 120000UL // 120 secs dead after success or error
#define SHOW_MS 650
#define SCROLL_MS 50
#define SCROLL_COL 1

/* Wiring 
   Arduino(Attiny1634) MCU Pin 
   D0                  1                  TX
   D1                  2                  vibration switch
   D2                  3                  Row 7 (pin 6)
   D3                  4                  Row 6 (pin 5)
   D4                  5                  Row 5 (pin 4)
   D5                  6                  Row 3 (pin 2)
   D6                  7                  Col 2 (pin 3)
   D7                  8                  MPU6050 power
   D8                  9                  Col 1 (pin 1)
   GND                10
   Vcc                11                 
   D9                 12                  Col 5 (pin 8)
   D10                13                  Col 4 (pin 7)
   Reset              14
   D11                15                  Row 1 (pin 12)
   D12/SCL/PC1        16                  MPU6050 SCL 
   D13                17                  Row 2 (pin 11)
   D14                18                  Row 4 (pin 9)
   D15                19                  Col 3 (pin 10)
   D16/SDA/PB1        20                  MPU6050 SDA
*/

#include <PETPreformBoard.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <EEPROM.h>
#include <DotMatrix5x7.h>
#include <Vcc.h>
#ifdef DEBUG
#include <TXOnlySerial.h>
#endif

#define I2C_TIMEOUT 1000
#include <SoftI2CMaster.h>

#ifdef DEBUG
#define DEBPR(str) mySerialprint(str)
#define DEBPRF(num,len) mySerialprint(num,len)
#define DEBWR(c)   mySerialwrite(c)
#define DEBLN(str) mySerialprintln(str)
#define DEBLNF(num,len) mySerialprintln(num,len)
#else
#define DEBPR(str)
#define DEBPRF(num,len)
#define DEBWR(c) 
#define DEBLN(str)
#define DEBLNF(num,len) 
#endif



/* messages */
/************/
const char i2c_error[] PROGMEM = "I2C Fehler ";
const char mpu_error[] PROGMEM = "MPU Fehler ";
const char mpuid_error[] PROGMEM = "MPU-ID Fehler ";
const char mpu_reset_error[] PROGMEM = "MPU-Start Fehler ";
const char state_error[] PROGMEM = "Interner Fehler ";
const char lowbatt_error[] PROGMEM = "Batterie Leer ";
const char unknown_error[] PROGMEM = "Unbekannter Fehler ";
const char* const errtab[] PROGMEM =
  { i2c_error, mpu_error, mpuid_error, mpu_reset_error, state_error, lowbatt_error, unknown_error };
const char success_str[] PROGMEM = SUCCMESS;
const char weakbatt_str[] PROGMEM = "Batterie ist fast leer. ";
const char fail_str[] PROGMEM = " FAIL";
const char bye_str[] PROGMEM = " Tsch\x81ss ";
const char volt_str[] PROGMEM = "Volt= ";
const char lastvolt_str[] PROGMEM = "Lastvolt= ";
const char minutes_str[] PROGMEM = "Minutes= ";
const char visits_str[] PROGMEM = "Visits= ";
const char succ_str[] PROGMEM = "Success= ";
const char eq_str[] PROGMEM = "= ";

/* alternative font for single dot flashing */
const byte flashdot[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x40};

/* constants */
/*************/
// error types
#define FIRST_ERROR 1
#define NO_ERROR 0
#define I2C_ERROR 1
#define MPU_ERROR 2
#define MPUID_ERROR 3
#define MPU_RESET_ERROR 4
#define STATE_ERROR 5
#define EMPTYBATT_ERROR 6
#define UNKNOWN_ERROR 7
#define LAST_ERROR 7

// init types
#define NO_INIT 0
#define START_INIT 1
#define IMU_INIT 2
#define HIMU_INIT 3
#define ACC_INIT 4
#define SLEEP_INIT 5

// states
#define NO_STATE 0
#define INIT_STATE 1
#define ERROR_STATE 2
#define WAITING_STATE 3
#define UPRIGHT_WAITING_STATE 4
#define NONUPRIGHT_WAITING_STATE 5
#define TURNING_STATE 6
#define STOPPED_STATE 7
#define VIOLATION_STATE 8 
#define SUCCESS_STATE 10
#define BYE_STATE 11
#define FAIL_STATE 12
#define OFF_STATE 13

/* global vars */
/***************/
struct  { // resides in EEPROM
  unsigned long totalsecs;
  int totalstarts;
  int totalsucc;
  int lastvolt;
  int errcnt[LAST_ERROR];
} stat;

#define MAGICKEY 0x61F7A092UL
volatile unsigned long magiclock __attribute__ ((section (".noinit")));
int volt = 0; // voltage at the beginning
volatile unsigned int seconds = 0; // variable to increment every second in WDT handler
volatile unsigned long lastbump = 0; // last bump registered
boolean blinkchar = false;
char showchar = '\0';
unsigned long lastsense, senseperiod, laststatechange, violationtime;
byte csegix = 0;
byte error = NO_ERROR;
int ay, az, gy; // MPU readings
long gy_integrated; // integrated turn (1/10 of °)
long gy_delta; // delta of movement
byte state, last_state;
byte mpu_state = NO_INIT;
byte adcsra; // saved state of ADC control register
boolean longdead = false; // sleep for 2 minutes after success or error

#if defined(DEBUG) 
#include <TXOnlySerial.h>
TXOnlySerial Serial(0);
#endif

/* main routines: setup + loop */
/*******************************/
void setup(void)
{
#ifdef DEBUG
  mySerial.begin(2400);
  mySerial.println(F("\nTurn Version " VERSION));
#endif
  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,                  // column pins
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7,  // row pins
	       LOW,                   // value when row pin is active (default value)
	       HIGH);                 // value when column pin is active (default value)
  Dot5x7.setFramesPerSecond(50);      // display 50 frames per second (default value)

  wdt_enable(WDTO_8S); // enable WDT interrupt every 8 second
  WDTCSR |= (1<<WDIE);
  enablePinChangeIRQ(); // allow for PCI interrupt from vibration sensor
  pinMode(MPUPWR, OUTPUT);
  digitalWrite(MPUPWR, HIGH);
  myDelay(100); // MPU is up and running
  error = (i2c_init() ? NO_ERROR : I2C_ERROR);
  DEBLN((error ? F("I2C init failed") : F("I2C init done")));
  if (!error) {
    mpu_init(START_INIT);
    DEBLN((error ? F("MPU init failed") : F("MPU init done")));
  }
  mpu_init(IMU_INIT);
  myDelay(10); // MPU draws current
  volt = Vcc::measure(100,INTREFVOLTAGE);
  mpu_init(SLEEP_INIT);

  DEBPR(F("Voltage: "));
  DEBLN(volt);
  readStat();
  adcsra = ADCSRA;
  ADCSRA = 0; //switch off ADC
  state = INIT_STATE;
  last_state = NO_STATE;
  laststatechange = millis();
  stat.totalstarts++;  
  DEBLN(F("Setup done"));
  if (magiclock != MAGICKEY) {
    magiclock = MAGICKEY;
    displayInfo();
    state = OFF_STATE;
  }
}

void loop(void)
{
  byte thiserror = error;
  
  if (error) { // when error var is != 0, switch to error state
    state = ERROR_STATE;
  }
  if (state != last_state) {
    DEBPR(F("New state: "));
    DEBLN(state);
    last_state = state;
    laststatechange = millis();
  }
  switch (state) {
  case INIT_STATE:
    if (volt < EMPTYBATT_VOLT) error = EMPTYBATT_ERROR;
    else {
      state = WAITING_STATE;
    }
    break;
  case ERROR_STATE:
    longdead = true;
    setDisplayChar(' ',false);
    if (thiserror < FIRST_ERROR || thiserror > LAST_ERROR) thiserror = UNKNOWN_ERROR;
    displayMessage((const char*)pgm_read_word(&(errtab[thiserror-1])), ERROR_REPMESS);
    stat.errcnt[thiserror-1]++;
    mpu_init(ACC_INIT);
    getShortReading();
    goSleep();
    break;
  case WAITING_STATE:
    violationtime = 0;
    setDisplayChar('\x18',true);
    mpu_init(ACC_INIT); getShortReading();
    if (ay > UPRIGHT_UPTHRESH) state = UPRIGHT_WAITING_STATE;
    else if (millis() - laststatechange > MAX_LONGWAIT_MS) state = OFF_STATE;
    break;
  case UPRIGHT_WAITING_STATE:
    setDisplayChar('0', true);
    gy_integrated = 0;
    mpu_init(HIMU_INIT); getLongReading();
    if (ay > UPRIGHT_DOWNTHRESH && az > INERTIA_UPTHRESH && abs(gy) > TURN_UPTHRESH) state = TURNING_STATE;
    else if (ay < UPRIGHT_DOWNTHRESH) state = NONUPRIGHT_WAITING_STATE;
    else if (millis() - laststatechange > MAX_LONGWAIT_MS) state = FAIL_STATE;
    break;
  case NONUPRIGHT_WAITING_STATE:
    setDisplayChar('\x18', true);
    mpu_init(ACC_INIT); getShortReading();
    if (ay > UPRIGHT_UPTHRESH) state = UPRIGHT_WAITING_STATE;
    else if (millis() - laststatechange > MAX_LONGWAIT_MS) state = FAIL_STATE;
    break;
  case TURNING_STATE:
    setDisplayChar('0' + abs(gy_integrated)/3600, false);
    mpu_init(HIMU_INIT); getLongReading();
    if (ay > UPRIGHT_DOWNTHRESH && (az < INERTIA_DOWNTHRESH || abs(gy) < TURN_DOWNTHRESH)) state = STOPPED_STATE;
    else if (ay < UPRIGHT_DOWNTHRESH) state = VIOLATION_STATE;
    else if (abs(gy_integrated)/3600 > MAXTURNS) state = SUCCESS_STATE;
    break;
  case STOPPED_STATE:
    setDisplayChar('0' + abs(gy_integrated)/3600, true);
    gy_integrated -= gy_delta;
    violationtime += senseperiod;
    if (violationtime > MAX_VIOLATION_MS) state = FAIL_STATE;
    else {
      mpu_init(HIMU_INIT); getLongReading();
      if (az > INERTIA_UPTHRESH && abs(gy) > TURN_UPTHRESH && ay > UPRIGHT_DOWNTHRESH) state = TURNING_STATE;
      else if (ay < UPRIGHT_DOWNTHRESH) state = VIOLATION_STATE;
    }
    break;
  case VIOLATION_STATE:
    setDisplayChar('\x18', true);
    gy_integrated -= gy_delta;
    violationtime += senseperiod;
    if (violationtime > MAX_VIOLATION_MS) state = FAIL_STATE;
    else {
      mpu_init(HIMU_INIT); getLongReading();
      if (ay > UPRIGHT_UPTHRESH && az > INERTIA_UPTHRESH && abs(gy) > TURN_UPTHRESH) state = TURNING_STATE;
      else if (ay > UPRIGHT_UPTHRESH) state = STOPPED_STATE;
    }
    break;
  case SUCCESS_STATE:
    mpu_init(SLEEP_INIT);
    setDisplayChar(' ',false);
    displayMessage(success_str, SUCCESS_REPMESS);
    stat.totalsucc++;
    state = BYE_STATE;
    longdead = true;
    break;
  case FAIL_STATE:
    mpu_init(SLEEP_INIT);
    setDisplayChar(' ',false);
    if (volt < WEAKBATT_VOLT)  displayMessage(weakbatt_str, WEAKBATT_REPMESS);
    displayMessage(fail_str, 3);
    goSleep();
    break;
  case BYE_STATE:
    mpu_init(SLEEP_INIT);
    setDisplayChar(' ',false);
    if (volt < WEAKBATT_VOLT)  displayMessage(weakbatt_str, WEAKBATT_REPMESS);
    displayMessage(bye_str, 2);
    goSleep();
    break;
  case OFF_STATE:
    setDisplayChar(' ',false);
    goSleep();
    break;
  default:
    error = STATE_ERROR;
    break;
  }
  myDelay(150); // since sampling freq is 4Hz, we can safely sleep for 150ms (using POWER_IDLE)
}

/* interrupt routines etc. */
/***************************/

ISR(VIB_PCINT_vect)
{
  lastbump = millis();
}

ISR(WDT_vect)
{
  WDTCSR |= (1<<WDIE); // re-enable watchdog interrupt
  seconds += 8;
  if (seconds >= MAXSECONDS) goSleep();
}

// This guards against reset loops caused by resets
// is useless under Arduino's bootloader
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
  return;
}

void enablePinChangeIRQ(void) 
{
  GIMSK = (1<<PCIE0);  // allow for PCINT on PCINT0-7
  PCMSK0 = (1<<PCINT7); // on pin 1
  
  //VIB_PCICR = (1<<VIB_PCIE);  // allow for PCINT on PCINT0-7
  //VIB_MASK = (1<<VIB_PCINT); // on pin 1 
}

/* display routines */
/********************/

void setDisplayChar(char c, boolean blinking)
{
  if (c != showchar) {
    Dot5x7.show(c);
    showchar = c;
  }
  if (blinking != blinkchar) {
    if (blinking) Dot5x7.setBlinkFrames(15,15);
    else  Dot5x7.setBlinkFrames(0,0);
    blinkchar = blinking;
  }
}

void displayNum(long num)
{
  char buf[12]; 

  itoa(num, buf, 10);
  Dot5x7.scrollLeftString(buf, SHOW_MS, SCROLL_MS, SCROLL_COL);
}

void displayMessage(const char* mess, byte rep)
{
  setDisplayChar(' ', false);
  for (byte i=0; i < rep; i++) 
    Dot5x7.scrollLeftString_P(mess, SHOW_MS, SCROLL_MS, SCROLL_COL);
}

// only called after power-up 
void displayInfo(void)
{
  for (int i=0; i< 1; i++) {
    Dot5x7.scrollLeftString_P(volt_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
    displayNum(volt);
    myDelay(1000);
#ifndef TESTING
    Dot5x7.scrollLeftString_P(minutes_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
    displayNum(stat.totalsecs/60);
    myDelay(2000);
    Dot5x7.scrollLeftString_P(visits_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
    displayNum(stat.totalstarts);
    myDelay(2000);
    Dot5x7.scrollLeftString_P(succ_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
    displayNum(stat.totalsucc);
    myDelay(2000);
    Dot5x7.scrollLeftString_P(lastvolt_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
    displayNum(stat.lastvolt);
    myDelay(2000);
    for (int err=0; err < LAST_ERROR; err++)
      if (stat.errcnt[err]) {
	Dot5x7.scrollLeftString_P((const char*)pgm_read_word(&(errtab[err])), SHOW_MS, SCROLL_MS, SCROLL_COL);
	Dot5x7.scrollLeftString_P(eq_str, SHOW_MS, SCROLL_MS, SCROLL_COL);
	displayNum(stat.errcnt[err]);
	myDelay(2000);
      }
    myDelay(5000);
#endif
  }
}


/* auxiliary functions */
/***********************/
  
// idle delay, wake up each msec by Timer1
void myDelay(unsigned int msecs)
{
  unsigned long start = millis();

  while (millis() - start < msecs) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
}

unsigned long lastBumped()
{
  unsigned long temp;
  noInterrupts();
  temp = lastbump;
  interrupts();
  return temp;
}  

void terminateIO(void) {
  Dot5x7.sleep();
  digitalWrite(MPUSDA, LOW);
  digitalWrite(MPUSCL, LOW);
  digitalWrite(MPUPWR, LOW);
}

void goSleep(void)
{
  wdt_disable();
  WDTCSR &= ~(1<<WDIE);
  interrupts();
  mpu_init(SLEEP_INIT); // switch off MPU
  if (longdead) {
    Dot5x7.setFont(flashdot);
    for (byte i= 0; i<(DEAD_MS/5000UL); i++) { 
      myDelay(5000);
      Dot5x7.show(0);
      myDelay(50);
      Dot5x7.clear();
    }
    Dot5x7.setFont();
  }
  setDisplayChar(' ',false);
  ADCSRA = 0; // switch off ADC
  terminateIO();
  writeStat();
  myDelay(20);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  wdt_enable(WDTO_15MS); 
  while (true); // reset processor and restart
}

// Deal with stat struct
// **********************

void readStat(void)
{
  EEPROM.get(0, stat);
  if (stat.totalsecs == 0xFFFFFFFF) {
    clearStat();
  }
}

void writeStat(void)
{
  stat.totalsecs += (millis()/1000);
  stat.lastvolt = volt;
  EEPROM.put(0, stat);
}


void clearStat(void)
{
  stat.totalsecs = 0;
  stat.totalstarts = 0;
  stat.totalsucc = 0;
  stat.lastvolt = 0;
  for (int i = 0; i < LAST_ERROR; i++) stat.errcnt[i] = 0;
}



/* MPU6050 routines */
/********************/

#define MAXBUF 16
byte buf[MAXBUF];

// read mpu register contents to buffer var
void mpuToBuf(byte reg, byte (&temp)[MAXBUF], byte len)
{
  boolean locerr = false;
  byte ix;
  locerr = !i2c_start(MPUADDR | I2C_WRITE);
  if (!locerr) locerr = !i2c_write(reg);
  if (!locerr) locerr = !i2c_rep_start(MPUADDR | I2C_READ);
  if (!locerr) {
    for (ix = 0; ix < len; ix++) 
      temp[ix] = i2c_read(ix == len-1);
  }
  i2c_stop();
  if (locerr && error == NO_ERROR) error = MPU_ERROR;
}

// write buffer contents to mpu regs
void bufToMpu(byte reg, byte (&temp)[MAXBUF], byte len)
{
  boolean locerr = false;
  byte ix;
  locerr = !i2c_start(MPUADDR | I2C_WRITE);
  if (!locerr) locerr = !i2c_write(reg);
  if (!locerr) {
    for (ix = 0; ix < len; ix++) 
      locerr |= !i2c_write(temp[ix]);
  }
  i2c_stop();
  if (locerr && error == NO_ERROR) error = MPU_ERROR;
}

// write just one register (byte length)
void setMpuByte(byte reg, byte val)
{
  buf[0] = val;
  bufToMpu(reg, buf, 1);
}

// write an integer to two registers
void setMpuInt(byte reg, int val)
{
  buf[0] = (val>>8);
  buf[1] = (val&0xFF);
  bufToMpu(reg, buf, 2);
}

// get one byte from one register
byte getMpuByte(byte reg)
{
  mpuToBuf(reg, buf, 1);
  return buf[0];
}

// get a (signed) int from two consecutive registers
int getMpuInt(byte reg)
{
  mpuToBuf(reg, buf, 2);
  return (buf[0]<<8)|buf[1];
}

// check whether data is ready
boolean mpuDataReady(void)
{
  return(getMpuByte(0x3A)&1);
}

// wait for next reading 
void waitDataReady(void)
{
  int cnt = 0;
  while (!mpuDataReady()) {
    if (++cnt > MPU_TIMEOUT_MS) {
      error = MPU_ERROR;
      return;
    }
    myDelay(1);
  }
}

// read simply y-axis acc value (normalized to -1000 to +1000 for -1g to +1g)
void getShortReading(void)
{
  int raw_ay;
  
  waitDataReady();
  raw_ay = getMpuInt(0x3D);
  if (error) return;
  ay = (int)((1000L * (long)raw_ay)/4096L);
  ay = YTRANSFORM(ay);
  senseperiod = millis() - lastsense;
  gy_delta = 0;
  lastsense = millis();
  DEBPR(F("ay: "));
  DEBLN(ay);
}

// read y,z-axis for acc and y-axis for gyro
// presupposes that mpu_int(INIT_HIMU) has been called before
void getLongReading(void)
{
  waitDataReady();
  if (error) return;
  ay = (int)((1000L * (long)getMpuInt(0x3D))/4096L); // normalized to -1000-+1000 for -g-+1g
  ay = YTRANSFORM(ay);
  az = (int)((1000L * (long)getMpuInt(0x3F))/4096L);  // normalized to -1000-+1000 for -g-+1g
  gy = getMpuInt(0x45); // 16.4 LSB for 1°/msec
  senseperiod = millis() - lastsense;
  gy_delta = (10L * (long)gy * (long)senseperiod) / 16400L;
  gy_integrated += gy_delta; // the accumulated turn 1/10 of °
  lastsense = millis();
}
  

void mpu_init(byte init_type)
{
  int cnt = 0;
  if (init_type == mpu_state) return;
  DEBPR(F("MPU_INIT: "));
  DEBLN(init_type);
  switch (init_type) {
  case START_INIT:
    // initialize everything from scratch
    setMpuByte(0x6A, 1); // resets signal paths and sensor registers
    setMpuByte(0x6B, B10000000); // device reset
    while (getMpuByte(0x6B)&B10000000) {
      if (++cnt > MPU_TIMEOUT_MS) {
	error = MPU_RESET_ERROR;
	return;
      }
      myDelay(1); // wait for done
    }
    myDelay(10); // At least 1 ms is necessary!!! Otherwise the following writes do not work!!! 
    // now start
    setMpuByte(0x6B, 0); // clear sleep bit, set clock source to internal 8MHz
    setMpuByte(0x6C, 0); // enable everything 
    // set acc and gyro offset registers
    setMpuInt(0x06, XACCELOFF);
    setMpuInt(0x08, YACCELOFF);
    setMpuInt(0x0A, ZACCELOFF);
    setMpuInt(0x13, XGYROOFF);
    setMpuInt(0x15, YGYROOFF);
    setMpuInt(0x17, ZGYROOFF);
    // set sensitivity
    setMpuByte(0x1B, B00011000); // gyro sensitivity = 3 means 2000°/s for 16.4LSB/°/s
    setMpuByte(0x1C, B00010000); // acc sensitivity = 2 means +/- 8g
    // set interrupt
    setMpuByte(0x38, B00000001); // enable Data Ready IRQ
    // DLPF
    setMpuByte(0x1A, 6); // means filter as 5Hz, 5 means 10Hz, 4 21Hz
    // sample rate
    setMpuByte(0x19, 249); // means 1kHz/(249+1) = 4Hz
    break;
    
  case IMU_INIT:
    // enable gyro and acc, switch to gyro PLL clock
    setMpuByte(0x6C, 0); // enable all gyro and acc axis
    setMpuByte(0x6B, 2); // switch to gyro Y-axis clock
    break;
    
  case HIMU_INIT: // disabling 2 gyro axis save 2mA!
    // enable Y,Z-axis for acc and Y-axis gyro, switch to gyro clock
    setMpuByte(0x6C, B00100101); // disable X acc and  X,Z gyro 
    setMpuByte(0x6B, 2); // switch to gyro Y-axis clock
    break;
      
  case ACC_INIT: // disabling 2 acc axis saves 70uA!
    // disable everything but the Y-axis for acc, switch to internal clock
    setMpuByte(0x6B, 0); // switch to internal clock
    setMpuByte(0x6C, B00101111); // disable X,Z acc and  X,Y,Z gyro 
    break;
    
  case SLEEP_INIT:
    // put the IC to sleep
    setMpuByte(0x6B, B01000000); // switch to internal clock and sleep
    break;

  default:
    error = STATE_ERROR;
    break;
  }
  lastsense = millis();
  if (error == 0)
    if (getMpuByte(0x75) != 0x68) // "Who am I" register
      error = MPUID_ERROR;
  mpu_state = init_type;
}
