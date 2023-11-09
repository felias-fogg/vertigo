// -*-c++-*-

/* Runs on a Pro Min with 2Hz measuring */

/* Used pins:
 * Button 1 (start/stop capture): 11
 * Button 2 (start trans): 8
 * LED red (capture): 10
 * LED yellow (trans): 9
 * MPU SCL: SCL
 * MPU SDA: SDA
 */

#define VERSION "0.1"

#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

//#define DEBUG

#define BUTCAP 11
#define BUTTRA 8
#define LEDCAP 10
#define LEDTRA 9

#define MAGIC 0x7503AD4FUL

#define XACCELOFF -2750
#define YACCELOFF 643
#define ZACCELOFF 1867
#define XGYROOFF -60
#define YGYROOFF 0
#define ZGYROOFF 0

#ifdef DEBUG
#define DEBPR(str) Serial.print(str)
#define DEBPRF(num,len) Serial.print(num,len)
#define DEBWR(c)   Serial.write(c)
#define DEBLN(str) Serial.println(str)
#define DEBLNF(num,len) Serial.println(num,len)
#else
#define DEBPR(str)
#define DEBPRF(num,len)
#define DEBWR(c) 
#define DEBLN(str)
#define DEBLNF(num,len) 
#endif


boolean mpuinit = false;

// orientation/motion vars
int ax, ay, az;
int gx, gy, gz;
unsigned long lastread;
float gx_acc, gy_acc, gz_acc;
float ax_norm, ay_norm, az_norm, a_norm;
unsigned long timeout = 10000UL;


MPU6050 mpu(0x69);

unsigned long magic  __attribute__ ((section (".noinit")));
int buf[300] __attribute__ ((section (".noinit")));
unsigned int buffil __attribute__ ((section (".noinit")));

void setup(void)
{
  Serial.begin(57600);
  DEBLN(F("\nMeasure MPU 2Hz Version" VERSION));
  pinMode(BUTCAP, INPUT_PULLUP);
  pinMode(BUTTRA, INPUT_PULLUP);
  pinMode(LEDCAP, OUTPUT);
  pinMode(LEDTRA, OUTPUT);
  digitalWrite(LEDCAP, LOW);
  digitalWrite(LEDTRA, LOW);
  if (magic != MAGIC) {
    DEBLN(F("Fresh start"));
    buffil = 0;
    magic = MAGIC;
  } else {
    DEBLN(F("Restart!"));
  }
  mpu_init();
  DEBLN(F("MPU initialized"));
  DEBPR(F("Free RAM: "));
  DEBLN(freeRam());
}

void loop(void)
{
  unsigned long start, length;
  int i = 0, j = 0, k;
  byte button;
  // wait for button press and release

  button = waitButtonPress(0);
  switch (button) {
  case BUTCAP:
    digitalWrite(LEDCAP, HIGH);
    DEBLN(F("Capture started"));
    clear_acc();
    start = millis();
    lastread = millis();
    i = 0;
    while (!isButtonPressed(BUTCAP) && (millis() - start < 5000) && i < 280) {
      getNextReading();
      buf[i++] = (int)(ax_norm*1000);
      buf[i++] = (int)(ay_norm*1000);
      buf[i++] = (int)(az_norm*1000);
      buf[i++] = (int)(a_norm*1000);
      buf[i++] = (int)((float)gx/32.8);
      buf[i++] = (int)((float)gy/32.8);
      buf[i++] = (int)((float)gz/32.8);
      buf[i++] = (int)(gx_acc*10);
      buf[i++] = (int)(gy_acc*10);
      buf[i++] = (int)(gz_acc*10);
      buffil = i;
    }
    length = millis() - start;
    DEBPR(F("Stopped after "));
    DEBPR((int)length);
    DEBLN(F("ms"));
    DEBPR(buffil/10);
    DEBLN(F(" entries"));
    digitalWrite(LEDCAP, LOW);
    if (isButtonPressed(BUTCAP)) waitButtonPress(BUTCAP);
    break;
  case BUTTRA:
    digitalWrite(LEDTRA, HIGH);
    DEBLN(F("Uploading..."));
    j = 0;
    while (j < buffil) {
      Serial.print(j/10);
      for (k = 0; k < 10; k++) {
	Serial.print(", ");
	Serial.print(buf[j+k]);
      }
      Serial.println();
      j += 10;
    }
    Serial.println(F("END"));
    digitalWrite(LEDTRA, LOW);
    //    buffil = 0;
    break;
  default:
    DEBLN(F("Timeout while waiting for button"));
    digitalWrite(LEDCAP, HIGH);
    digitalWrite(LEDTRA, HIGH);
    delay(100);
    digitalWrite(LEDCAP, LOW);
    digitalWrite(LEDTRA, LOW);
    return;
  }
}
	
	
int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;

  int free_memory;
  int stack_here;

  if (__brkval == 0)
    free_memory = (int) &stack_here - (int) &__heap_start;
  else
    free_memory = (int) &stack_here - (int) __brkval; 

  return (free_memory);
}


boolean isButtonPressed(byte button)
{
  return(digitalRead(button) == LOW);
}

byte waitButtonPress(byte button_pressed)
{
  byte button = button_pressed;
  unsigned long last = millis();
  while (millis() - last < timeout) {
    if (button_pressed == 0) {
      if (isButtonPressed(BUTCAP)) 
	button = BUTCAP;
      else if (isButtonPressed(BUTTRA))
	button = BUTTRA;
      if (button) delay(30);
    }
    if (button) {
      if (isButtonPressed(button) || button == button_pressed) {
	while (millis() - last < timeout) {
	  if (!isButtonPressed(button)) {
	    delay(20);
	    if (!isButtonPressed(button)) return button;
	  }
	}
      } else {
	button = 0;
	continue;
      }
    }
  }
  return 0;
}


void mpu_init(void)
{
  if (mpuinit) return; // already done!
  Wire.begin();
  //TWBR = 24; // 200 kHz on 8MHz systems
  mpu.initialize();
  if (!mpu.testConnection()) {
    DEBLN(F("MPU initializiation error"));
    return;
  }
  mpu.setXAccelOffset(XACCELOFF);
  mpu.setYAccelOffset(YACCELOFF);
  mpu.setZAccelOffset(ZACCELOFF);
  mpu.setXGyroOffset(XGYROOFF);
  mpu.setYGyroOffset(YGYROOFF);
  mpu.setZGyroOffset(ZGYROOFF);
  mpu.setFullScaleGyroRange(2); // means +/- 1000°/s 31.8LSB/deg/s
  mpu.setFullScaleAccelRange(2); // means +/- 8g
  
  mpu.setDLPFMode(6); // means roughly 5Hz
  mpu.setIntDataReadyEnabled(true);
  mpuinit = true;
  DEBPR(F("Default sample rate: "));
  DEBLN(mpu.getRate());
  mpu.setRate(249);
  DEBPR(F("New sample rate: "));
  DEBLN(mpu.getRate());
}


boolean getNextReading(void)
{
  boolean ready = false;
  while (millis() - lastread < 400) {
    ready = mpu.getIntDataReadyStatus();
    if (ready) break;
  }
  if (ready)
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  else {
    digitalWrite(LEDTRA,HIGH);
    DEBLN(F("MPU timeout"));
    while (true);
  }
  ax_norm = (float)ax/(16384.0/4);
  ay_norm = (float)ay/(16384.0/4);
  az_norm = (float)az/(16384.0/4);
  a_norm = sqrt(pow(ax_norm,2)+pow(ay_norm,2)+pow(az_norm,2));
  gx_acc += (float)gx/32.8*(millis()-lastread)/1000.0;
  gy_acc += (float)gy/32.8*(millis()-lastread)/1000.0;
  gz_acc += (float)gz/32.8*(millis()-lastread)/1000.0;
  lastread = millis();
#ifdef DEBUG
  DEBPR(F("a: "));
  DEBPR(ax); DEBPR("\t");
  DEBPR(ay); DEBPR("\t");
  DEBPR(az); DEBPR("\t");
  DEBPR(a_norm); DEBPR("\tg:");
  DEBPR(gx);  DEBPR("\t");
  DEBPR(gy);  DEBPR("\t");
  DEBLN(gz);
#endif
  return true;
}

void clear_acc(void)
{
  lastread = millis();
  gx_acc = 0;
  gy_acc = 0;
  gz_acc = 0;
}
