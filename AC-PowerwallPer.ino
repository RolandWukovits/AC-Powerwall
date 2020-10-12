
// This is beta-software for my AC-powerwall controller
// Version 1.00.37 (complete edition), 11.10.2020
// Author and system designer: Roland Wukovits
// e-mail: acpw@thehillside.net
// The code, or parts of it, can be used for private consumption, as
// parts of the code (especially libraries and standard routines) 
// are originated from other Authors
// Code for the whole controller, is intellectual property of the 
// system designer
// If you need customized features, you can contact me for assistance,
// as well i can source components and build a controller board (anywhwere)
// (or even a whole powerwall (Thailand)) for you 
// If I was able to help you out with valueable information to 
// realize your project, please consider an adequate donation by using
// PayPal: sales@thehillside.net
// Each donation, even small ones, are very much appreciated. Thanks!


// Include Wire Library for I2C
#include <Wire.h>
// Include NewLiquidCrystal Library for I2C
#include <LiquidCrystal_I2C.h>
// Include DS3231 RTC Library
#include <DS3231.h>
// Include DHT11 Libraries
#include <Adafruit_Sensor.h>
#include <DHT.h>
// Include Modbus for SDM120 Library
#include "ModbusSensor.h"
// Include EEPROM
#include <EEPROM.h>
// Include ADS1115 Library
#include <Adafruit_ADS1015.h>
// Include libraries for the SD module
#include <SD.h>
#include <SPI.h>
// Watchdog
#include <avr/wdt.h>
//ESP8266 and Blynk
//#include <Blynk.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>


#define BLYNK_PRINT Serial
char auth[] = "your Auth";
char ssid[] = "your SSID";
char pass[] = "your Password";

// Hardware Serial on Mega, Leonardo, Micro...
#define EspSerial Serial2

// or Software Serial on Uno, Nano...
//#include <SoftwareSerial.h>
//SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);


// Define LCD pinout
const int  en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;
 // Define I2C Address - change if reqiuired
const int i2c_addr = 0x3F;
LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

// Modbus Eastron SDM120, X835
#define MODBUS_SERIAL_OUTPUT  // MODBUS messages and timing

#ifdef MODBUS_SERIAL_OUTPUT
#define MODBUS_SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__)
#define MODBUS_SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define MODBUS_SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define MODBUS_SERIAL_BEGIN(...)
#define MODBUS_SERIAL_PRINT(...)
#define MODBUS_SERIAL_PRINTLN(...)
#endif

// Finite state machine status
#define STOP                0
#define SEND                1
#define SENDING             2
#define RECEIVING           3
#define IDLE                4
#define WAITING_NEXT_POLL   5

#define READ_COIL_STATUS          0x01
#define READ_INPUT_STATUS         0x02
#define READ_HOLDING_REGISTERS    0x03
#define READ_INPUT_REGISTERS      0x04
#define FORCE_MULTIPLE_COILS      0x0F
#define PRESET_MULTIPLE_REGISTERS 0x10

#define MB_VALID_DATA     0x00
#define MB_INVALID_ID     0xE0
#define MB_INVALID_FC     0xE1
#define MB_TIMEOUT        0xE2
#define MB_INVALID_CRC    0xE3
#define MB_INVALID_BUFF   0xE4
#define MB_ILLEGAL_FC     0x01
#define MB_ILLEGAL_ADR    0x02
#define MB_ILLEGAL_DATA   0x03
#define MB_SLAVE_FAIL     0x04
#define MB_EXCEPTION      0x05

#define SERIAL_OUTPUT 1

#if SERIAL_OUTPUT
#   define SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__)
#   define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#   define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#   define SERIAL_BEGIN(...)
#   define SERIAL_PRINT(...)
#   define SERIAL_PRINTLN(...)
#endif

#define MB_SERIAL_PORT &Serial1   // Arduino has only one serial port, Mega has 3 serial ports.
#define MB_BAUDRATE 9600          // baud 2400 for SDM120, 9600 for X835
#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       41
#define ID_1  1                       // id 001  modbus id of the energy monitor
#define REFRESH_INTERVAL  5000        // refresh time, 5 SECONDS
#define POW_ADR 0x0010    // measured power X835/phase3  (0x000C for SDM120 or X835/phase1, 0x000E for X835/phase2

modbusMaster MBserial(MB_SERIAL_PORT, TxEnablePin);  // instance to collect data using Modbus protocol over RS485
modbusSensor pwr(&MBserial, ID_1, POW_ADR, CHANGE_TO_ZERO);

String holdpower;
float measuredpower;
float averagepower=0;
float averagepowerpool=0;
float lastaveragepower=0;
int powercounter=1;
int triggerpowermode1=50;  //  minimum power available for mode 1 / 10 (avoid import)
int triggerpowermode2=30;  //  minimum power available for mode 2 / 10 (allow import)
int settriggerpowermode1;
int settriggerpowermode2;
float realpower=0;
byte chargerone=0;
byte chargertwo=0;
byte inverter=0;
byte invertreadout=0;
byte setinvertreadout;
byte trueread;
 
// Rotary Encoder Inputs
 #define CLK 43
 #define DT 44
 #define SW 45
// Init the DS3231 using the hardware interface
DS3231 clock;
RTCDateTime dt;
// DHT11 sensor data
#define sensepin 3
DHT HT(sensepin,DHT11);
float humidity;
float tempC;
float fancounter;
// Relays module
#define relayPin1 36               // Charger 1
#define relayPin2 35               // Charger 2
#define relayPin3 34               // Inverter DC
#define relayPin4 37               // Fans
#define relayPin5 38               // Inverter Capacitor Precharging
#define relayPin6 39               // empty
byte relayAct1=0;
byte relayAct2=0;
byte relayAct3=0;
byte relayAct4=0;
int relaycycletime=200;    // set for the charger activation cycle time in loops
int prechargedelay=2500;   // set for inverter capacitor precharging in milliseconds
byte stopdelay;
// Mode Switch
#define Mode1Pin 22
#define Mode2Pin 23
#define Mode3Pin 24
#define Mode4Pin 25
#define Mode5Pin 26
#define Mode6Pin 27
#define Mode7Pin 28
#define Mode8Pin 29
#define Mode9Pin 30
#define Mode10Pin 31
#define Mode11Pin 32
#define Mode12Pin 33
byte mode;                  
// Rotary switch
int counter=1; 
int currentStateCLK=0;
int previousStateCLK=0; 
byte setpos=0;
byte switchpress=1;
byte switchpressstop=0;
byte setday=1;
byte setmonth=1;
int setyear=2020;
byte sethour=0;
byte setminute=0;
byte setsecond=0;
byte buttonrelease=0;
// Voltage divider
float calibrationvalue;
float voltread;
float battvoltage;
float fineadjustment=0;
float fineadjustmentbyte=20;          
float setfineadjustmentbyte;          
byte battSOC;
byte SOCfreeze;
byte idlestatus;
byte oldidlestatus;
byte minimumSOC=10;
byte setminimumSOC;
byte maximumSOC=90;
byte setmaximumSOC;
byte fanAct=0;
float setfanActtemp;                 
float fanActtemp=30;                 
long loopcounter=0;
float averagevolt;
float holdvolt;
int voltcounter=1;
float SOCvoltage;
byte showpower=1;
byte setshowpower;
byte inverteravailable;
byte invcurrsens;
byte setinvcurrsens;
byte chargerlock;
// EEPROM
byte EEPROMaddress = 0;
byte EEPROMvalue[40];         //Array accomodating 30 stored values, increase if needed
//LED announciators
long ledcounter=0;
byte LEDgreen;
byte LEDred;
byte LEDyellow;
byte greencounter=0;
byte chargercounter=0;
byte invertercounter=0;
#define greenpin 5
#define redpin 6
#define yellowpin 7
// Parameters
byte typeofbatt=1;  //0=LIPO, 1=LIFEPO, 2=NMC
byte ConeAmps=20;
byte CtwoAmps=30;
byte settypeofbatt;
byte setConeAmps;
byte setCtwoAmps;
byte corrindchg;
byte corrindinv;
byte corrindrest;
//Timer Variables
byte T1C1On=8;
byte T1C2On=10;
byte T1InvOn=18;
byte T1C1Off=15;
byte T1C2Off=16;
byte T1InvOff=7;
byte setT1C1On;
byte setT1C2On;
byte setT1InvOn;
byte setT1C1Off;
byte setT1C2Off;
byte setT1InvOff;
byte T2C1On=8;
byte T2C2On=10;
byte T2InvOn=18;
byte T2C1Off=15;
byte T2C2Off=16;
byte T2InvOff=7;
byte setT2C1On;
byte setT2C2On;
byte setT2InvOn;
byte setT2C1Off;
byte setT2C2Off;
byte setT2InvOff;
byte timermatch;
byte timer2match;
byte timer3match;
byte invertertime;
byte C1time;
byte C2time;
byte showC1On;
byte showC2On;
byte showInvOn;
byte showC1Off;
byte showC2Off;
byte showInvOff;
byte actC1Timer;
byte actC2Timer;
byte actInvTimer;
byte NormInvOn=18;
byte NormInvOff=7;
byte setNormInvOn;
byte setNormInvOff;
byte setDreset;
byte setDresethr;
byte setDresetmin;
byte Dreset=0;
byte Dresethr=7;
byte Dresetmin=5;
// ADS1115
Adafruit_ADS1115 ads;
// SD module
File ACPWdata;
const int CS=53;
byte SDavail;
float chargeronelog;
float chargertwolog;
float inverterlog;
float bothchargerslog;
// Current module
int currentcounter;
long maxCurrent=1;
long minCurrent=65535;
long diffCurrent;
long tempdiffCurrent;
long readCurrent;
int innercounter;
// Blynk/Wifi
byte wifidis=0;
byte wifiavail;
byte setwifiavail;
byte bly;

void setup()
{
  
  wdt_disable();
  Serial.begin(9600);
  //check for EEPROM data and read
  for (EEPROMaddress=0;EEPROMaddress<40;EEPROMaddress++) {
    EEPROMvalue[EEPROMaddress]=EEPROM.read(EEPROMaddress);
    delay(50);
  }
  //firstwrite EEPROM
  if (EEPROMvalue[0]==255) {
     EEPROM.write(0,1);                  // "1" shows EEPROM have data
     EEPROM.write(1,minimumSOC);
     EEPROM.write(2,maximumSOC);
     EEPROM.write(3,fanActtemp);
     EEPROM.write(4,typeofbatt);                 
     EEPROM.write(5,ConeAmps);
     EEPROM.write(6,CtwoAmps);
     EEPROM.write(7,T1C1On);
     EEPROM.write(8,T1C2On);                 
     EEPROM.write(9,T1InvOn);
     EEPROM.write(10,T1C1Off);
     EEPROM.write(11,T1C2Off);
     EEPROM.write(12,T1InvOff);
     EEPROM.write(13,T2C1On);
     EEPROM.write(14,T2C2On);
     EEPROM.write(15,T2InvOn);
     EEPROM.write(16,T2C1Off);
     EEPROM.write(17,T2C2Off);
     EEPROM.write(18,T2InvOff);
     EEPROM.write(19,triggerpowermode1);
     EEPROM.write(20,triggerpowermode2);
     EEPROM.write(21,showpower);
     EEPROM.write(22,invertreadout);
     EEPROM.write(23,fineadjustmentbyte);
     EEPROM.write(24,NormInvOn);
     EEPROM.write(25,NormInvOff);
     EEPROM.write(26,Dreset);                      
     EEPROM.write(27,Dresethr);                  
     EEPROM.write(28,Dresetmin);
     EEPROM.write(29,invcurrsens);
     EEPROM.write(30,wifiavail);
     EEPROM.write(31,1);               // spare slot
     EEPROM.write(32,1);
     EEPROM.write(33,1);
     EEPROM.write(34,1);
     EEPROM.write(35,1);
     EEPROM.write(36,1);                      
     EEPROM.write(37,1);                  
     EEPROM.write(38,1);
     EEPROM.write(39,1);
  }
  else {
     minimumSOC= EEPROMvalue[1];
     setminimumSOC=minimumSOC;
     maximumSOC= EEPROMvalue[2];
     setmaximumSOC=maximumSOC;
     fanActtemp= EEPROMvalue[3];
     setfanActtemp=fanActtemp;
     typeofbatt= EEPROMvalue[4];
     settypeofbatt=typeofbatt;
     ConeAmps= EEPROMvalue[5];
     setConeAmps=ConeAmps;
     CtwoAmps= EEPROMvalue[6];
     setCtwoAmps=CtwoAmps;
     T1C1On=EEPROMvalue[7];
     setT1C1On=T1C1On;
     T1C2On=EEPROMvalue[8];
     setT1C2On=T1C2On;
     T1InvOn=EEPROMvalue[9];
     setT1InvOn=T1InvOn;
     T1C1Off=EEPROMvalue[10];
     setT1C1Off=T1C1Off;
     T1C2Off=EEPROMvalue[11];
     setT1C2Off=T1C2Off;
     T1InvOff=EEPROMvalue[12];
     setT1InvOff=T1InvOff;
     T2C1On=EEPROMvalue[13];
     setT2C1On=T2C1On;
     T2C2On=EEPROMvalue[14];
     setT2C2On=T2C2On;
     T2InvOn=EEPROMvalue[15];
     setT2InvOn=T2InvOn;
     T2C1Off=EEPROMvalue[16];
     setT2C1Off=T2C1Off;
     T2C2Off=EEPROMvalue[17];
     setT2C2Off=T2C2Off;
     T2InvOff=EEPROMvalue[18];
     setT2InvOff=T2InvOff;
     triggerpowermode1=EEPROMvalue[19];
     settriggerpowermode1=triggerpowermode1;
     triggerpowermode2=EEPROMvalue[20];
     settriggerpowermode2=triggerpowermode2;
     showpower=EEPROMvalue[21];
     setshowpower=showpower;
     invertreadout=EEPROMvalue[22];
     setinvertreadout=invertreadout;
     fineadjustmentbyte=EEPROMvalue[23];
     setfineadjustmentbyte=fineadjustmentbyte;
     NormInvOn=EEPROMvalue[24];
     setNormInvOn=NormInvOn;
     NormInvOff=EEPROMvalue[25];
     setNormInvOff=NormInvOff;
     Dreset=EEPROMvalue[26];
     setDreset=Dreset;
     Dresethr=EEPROMvalue[27];
     setDresethr=Dresethr;
     Dresetmin=EEPROMvalue[28];
     setDresetmin=Dresetmin;
     invcurrsens=EEPROMvalue[29];
     setinvcurrsens=invcurrsens;  
     wifiavail=EEPROMvalue[30];
     setwifiavail=wifiavail;
  }
 
  // Initialize the rtc object
  clock.begin();


  // Set display type as 20 char, 4 rows
  lcd.begin(20,4);
  
  // LCD Print on first row
  lcd.setCursor(0,0);
  lcd.print("  System start up   ");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print("acpw@thehillside.net");
  lcd.setCursor(0,3);
  lcd.print("V1.00.37/11.10.2020");
  
  HT.begin();

  // 12-way Mode Switch
  pinMode(Mode1Pin,INPUT);  
  pinMode(Mode2Pin,INPUT);  
  pinMode(Mode3Pin,INPUT);
  pinMode(Mode4Pin,INPUT);
  pinMode(Mode5Pin,INPUT);  
  pinMode(Mode6Pin,INPUT);  
  pinMode(Mode7Pin,INPUT);
  pinMode(Mode8Pin,INPUT);
  pinMode(Mode9Pin,INPUT);  
  pinMode(Mode10Pin,INPUT);  
  pinMode(Mode11Pin,INPUT);
  pinMode(Mode12Pin,INPUT);
  // Relays module
  pinMode(relayPin1,OUTPUT);  // Charger 1
  pinMode(relayPin2,OUTPUT);  // Charger 2
  pinMode(relayPin3,OUTPUT);  // Inverter Grid tie
  pinMode(relayPin4,OUTPUT);  // Fan
  pinMode(relayPin5,OUTPUT);  // Inverter Capacitor Precharging
  pinMode(relayPin6,OUTPUT);  // spare

  // All relays unpowered
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin4,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin6,HIGH);
  
  // Set rotary encoder pins 
  pinMode (CLK,INPUT);
  pinMode (DT,INPUT);
  pinMode (SW,INPUT_PULLUP);
  previousStateCLK = digitalRead(CLK);

  switchpress=digitalRead(SW);
  if (switchpress==0){
    wifidis=1;                          // wifi and Blynk can be overriden when switch pushed at startup
   }

  if (wifiavail==0){
    wifidis=1; 
  }

  // ESP and Blynk 
  if (wifidis==0){
  EspSerial.begin(ESP8266_BAUD);
  delay(100);
  Blynk.begin(auth, wifi, ssid, pass);
  }
  
  // You can also specify server:
  //Blynk.begin(auth, wifi, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, wifi, ssid, pass, IPAddress(192,168,1,100), 8080);
  
  
  
  // Modbus setup
  // SERIAL_BEGIN(9600);
  MBserial.begin(MB_BAUDRATE, MB_BYTEFORMAT, REFRESH_INTERVAL);
  
  //LEDs
  pinMode(greenpin,OUTPUT);  // Power available
  pinMode(redpin,OUTPUT);  // Inverting
  pinMode(yellowpin,OUTPUT);  // Charging

  // ADS1115
   ads.setGain(GAIN_TWOTHIRDS);   // full range
   ads.begin();
 
  // SDcard
  pinMode(CS,OUTPUT);

   fineadjustment=(fineadjustmentbyte-20)/500000;   // with ADS1115;
   calibrationvalue=0.002392;   //calibrate your readout with a volt meter!
  
  delay(1000);
  
  lcd.clear();

  if (SD.begin(CS)) {
    lcd.setCursor(0,0);
    lcd.print("   SD Card ready    ");
    lcd.setCursor(0,1);
    lcd.print("Data will be logged ");
    lcd.setCursor(0,2);
    lcd.print("as long Card present");
    lcd.setCursor(0,3);
    lcd.print("in Modes 1 or 2 only");
    SDavail=1;
    }
    else {
    lcd.setCursor(0,0);
    lcd.print("   SD Card failed   ");
    lcd.setCursor(0,1);
    lcd.print("Enter Card and re-  ");
    lcd.setCursor(0,2);
    lcd.print("start unit, if      ");
    lcd.setCursor(0,3);
    lcd.print("logging is required ");
    SDavail=0;
    }

 // Wait 5 second
  delay(4000);
  
  lcd.clear();
  wdt_enable(WDTO_8S);
   
}

void loop(){

// Main WatchdogTimer reset
wdt_reset(); 
  
// getting the Modbus readout
if (MBserial.available()) {
    measuredpower=holdpower.toFloat();    // reconverts String to float
    trueread=1;
}

if (trueread==1){
if (invertreadout==1) {
  measuredpower=measuredpower*-1;
}
trueread=0;
}

if (ledcounter==10000) {  //mode readout 1 time a second
  mode=1;
if (digitalRead(Mode1Pin)==HIGH) { mode=1; }
if (digitalRead(Mode2Pin)==HIGH) { mode=2; }
if (digitalRead(Mode3Pin)==HIGH) { mode=3; }
if (digitalRead(Mode4Pin)==HIGH) { mode=4; }
if (digitalRead(Mode5Pin)==HIGH) { mode=5; }
if (digitalRead(Mode6Pin)==HIGH) { mode=6; }
if (digitalRead(Mode7Pin)==HIGH) { mode=7; }
if (digitalRead(Mode8Pin)==HIGH) { mode=8; }
if (digitalRead(Mode9Pin)==HIGH) { mode=9; }
if (digitalRead(Mode10Pin)==HIGH) { mode=10; }
if (digitalRead(Mode11Pin)==HIGH) { mode=11; }
if (digitalRead(Mode12Pin)==HIGH) { mode=12; }
}


loopcounter++;
ledcounter++;

 if (mode==1 and loopcounter>=65000){
  Mode1();
  loopcounter=0;
  wdt_reset();
  }

 if (mode==2 and loopcounter>=65000){
  Mode2();
  loopcounter=0;
  wdt_reset();
 }
 
 if (mode==3 and loopcounter>=65000){
  Mode3();
  loopcounter=0;
  wdt_reset();
  }
  
 if (mode==4 and loopcounter>=65000){
  Mode4();
  loopcounter=0;
  wdt_reset();
  }

 if (mode==5 and loopcounter>=100000){
  Mode5();
  loopcounter=0;
  wdt_reset();
 }

 if (mode==6 and loopcounter>=100000){
  Mode6();
  loopcounter=0;
  wdt_reset();
 }

  if (mode==7 and loopcounter>=100000){
  Mode7();
  loopcounter=0;
  wdt_reset();
  }
  
 if (mode==8 and loopcounter>=100000){
  Mode8();
  loopcounter=0;
  wdt_reset();
  }

 if (mode==9 and loopcounter>=5000){
  Mode9();
  loopcounter=0;
  wdt_reset();
 }

 if (mode==10 and loopcounter>=5000){
  Mode10();
  loopcounter=0;
  wdt_reset();
 }

 if (mode==11 and loopcounter>=5000){
  Mode11();
  loopcounter=0;
  wdt_reset();
 }
 
 if (mode==12 and loopcounter>=5000){
  Mode12();
  loopcounter=0;
  wdt_reset();
 }

 if (ledcounter>=40000) {  //approx 1 sec cycle
  LEDsignals();
  ledcounter=0;

  CheckFan();

  currentcounter++;
  if (currentcounter>=30){
    currentcounter=0;
    minCurrent=65535;
    maxCurrent=1;
  }

  if (currentcounter==5 or currentcounter==15 or currentcounter==25) {
    BlynkDataTransmit();
  }
  
 }

innercounter++;
 if (currentcounter==1 and innercounter>=100){
  Currentaquisition();
  innercounter=0;
 }

if (Dreset==1 and dt.hour==Dresethr and dt.minute==Dresetmin){
  if (dt.second>=0 and dt.second<=5){
      resetFunc(); 
  }                                    // resets system according setup
}


}


void Mode1(){

  // Normal Mode - avoid Import (Mode1),  Relays are ON when "L0W"
 
    switchpress=digitalRead(SW);

    
    // charger activation cycle
    averagepower=averagepower+measuredpower;
    averagepowerpool=averagepower/powercounter;
    powercounter++;
   
   if (chargerone==1 and chargertwo==0) {
    realpower=averagepowerpool+(battvoltage*ConeAmps);
    }
    if (chargerone==0 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*CtwoAmps);
    }
    if (chargerone==1 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*(ConeAmps+CtwoAmps));
    }
    if (chargerone==0 and chargertwo==0) {
    realpower=averagepowerpool;
    }
    
if (powercounter>=relaycycletime) {
// upper limit for charging
if (battSOC<=maximumSOC) {
  
if (typeofbatt==1 and corrindchg==7) {
  chargerlock=1;
}
if (typeofbatt==1 and inverter==1) {
  chargerlock=0;
}
  
   if (realpower<=((triggerpowermode1*10)+(50*ConeAmps))) {
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     chargerone=0;
     chargertwo=0;
   } 
   if (realpower>((triggerpowermode1*10)+(50*ConeAmps)) and realpower<((triggerpowermode1*10)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,LOW);
     digitalWrite(relayPin2,HIGH);
     chargerone=1;
     chargertwo=0;
   }
    if (realpower>=((triggerpowermode1*10)+(50*CtwoAmps)) and realpower<((triggerpowermode1*10)+(50*ConeAmps)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,HIGH);
     chargerone=0;
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
   } 
   if (realpower>=((triggerpowermode1*10)+(50*ConeAmps)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,LOW);
     chargerone=1;
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
     }
     else {
     digitalWrite(relayPin2,HIGH);
     chargertwo=0; 
     }
   } 
}
else{
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     chargerone=0;
     chargertwo=0;
  
}

  Datalogging();
   
    SOCfreeze=1;
    lastaveragepower=averagepowerpool;
    averagepower=0;
    powercounter=1;
    if (chargerone==0 and chargertwo==0 and inverter==0){
         if (oldidlestatus==1){
         idlestatus=1;
         }
         else {
         oldidlestatus=1;
         }
    }
     else {
     oldidlestatus=0; 
     idlestatus=0;    
     }
   }


if (powercounter==55) {
  SOCfreeze=0;
}

if (switchpress==1) {
  lcd.setCursor(0,0);
  lcd.print("Mode 1a: avoid Imp. ");
  lcd.setCursor(0,1);
  if (chargerone==1 and chargertwo==0) {
    lcd.print("C1:ON C2:OFF  ");
  }
  if (chargerone==0 and chargertwo==1) {
    lcd.print("C1:OFF C2:ON  ");
  }
  if (chargerone==1 and chargertwo==1) {
    lcd.print("C1:ON C2:ON   ");
  }
 if (chargerone==0 and chargertwo==0) {
    lcd.print("C1:OFF C2:OFF ");
  }   

 getVoltage();
 
 getSOC();

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

  // date
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
 // time
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);

  
  lcd.setCursor(14,1);
  if (inverter==1) {
    lcd.print("In:ON ");
  }
  if (inverter==0) {
    lcd.print("In:OFF");
  }
 
}
else {
    lcd.setCursor(0,0);
  lcd.print("Mode 1b: avoid Imp. ");
  lcd.setCursor(0,1);
  if (showpower==0) {
  lcd.print("P-averg:");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(averagepowerpool);
  }
  else {
  lcd.print("P-real: ");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(measuredpower);  
  }
  lcd.setCursor(19,1);
  if (averagepowerpool<lastaveragepower) {
    lcd.print("v");
  }
  else {
    lcd.print("^");
  }



  lcd.setCursor(0,2);
  lcd.print("Temp:");
  lcd.print(tempC);
  lcd.setCursor(12,2);
  lcd.print("RH:");
  lcd.print(humidity); 
 

  lcd.setCursor(0,3);  
  if (typeofbatt==0) { 
    lcd.print("LIPO     "); 
  } 
  if (typeofbatt==1) { 
    lcd.print("LIFEPO   ");
  } 
  if (typeofbatt==2) { 
    lcd.print("NMC      ");
  } 
 
    
  lcd.setCursor(9,3);
  if (invertreadout==1) { 
    lcd.print("invd  ");
  }
  if (invertreadout==0) { 
    lcd.print("norm  ");
  }

  lcd.setCursor(15,3);
    lcd.print(corrindrest);
    lcd.print("/");
    lcd.print(corrindchg);
    lcd.print("/");
    lcd.print(corrindinv);
   

  
}



   invertertime=(dt.hour);
   if (battSOC>minimumSOC+10){
    inverteravailable=1; 
   }

   if (battSOC<=minimumSOC){
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    inverteravailable=0;
   }
  
  if (inverteravailable==1){  
  if (NormInvOff<NormInvOn) {   
  if (invertertime>=NormInvOn or invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
        digitalWrite(relayPin3,LOW);
        inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }

 if (NormInvOff>NormInvOn) {   
  if (invertertime>=NormInvOn and invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
      if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
      digitalWrite(relayPin3,LOW);
      inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }
 
 } 

}

void Mode2(){

  // Normal Mode - allow Import (Mode2),  Relays are ON when "L0W"
  switchpress=digitalRead(SW);
    
      
    // charger activation cycle
    averagepower=averagepower+measuredpower;
    averagepowerpool=averagepower/powercounter;
    powercounter++;
   
   if (chargerone==1 and chargertwo==0) {
    realpower=averagepowerpool+(battvoltage*ConeAmps);
    }
    if (chargerone==0 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*CtwoAmps);
    }
    if (chargerone==1 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*(ConeAmps+CtwoAmps));
    }
    if (chargerone==0 and chargertwo==0) {
    realpower=averagepowerpool;
    }
    
if (powercounter>=relaycycletime) {
// upper limit for charging
if (battSOC<=maximumSOC) {
  
  if (typeofbatt==1 and corrindchg==7) {
  chargerlock=1;
  }
  if (typeofbatt==1 and inverter==1) {
  chargerlock=0;
  }

   if (realpower<=((triggerpowermode2*-10)+(50*ConeAmps))) {
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     chargerone=0;
     chargertwo=0;
   } 
   if (realpower>((triggerpowermode2*-10)+(50*ConeAmps)) and realpower<((triggerpowermode1*10)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,LOW);
     digitalWrite(relayPin2,HIGH);
     chargerone=1;
     chargertwo=0;
   }
    if (realpower>=((triggerpowermode2*-10)+(50*CtwoAmps)) and realpower<((triggerpowermode1*10)+(50*ConeAmps)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,LOW);
     chargerone=0;
     chargertwo=1;
   } 
   if (realpower>=((triggerpowermode2*-10)+(50*ConeAmps)+(50*CtwoAmps))) {
     digitalWrite(relayPin1,LOW);
     chargerone=1;
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
     }
     else {
     digitalWrite(relayPin2,HIGH);
     chargertwo=0; 
     }
   } 
}
else{
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     chargerone=0;
     chargertwo=0;
 }

  Datalogging();
   
    lastaveragepower=averagepowerpool;
    averagepower=0;
    powercounter=1;
    SOCfreeze=1;
    if (chargerone==0 and chargertwo==0 and inverter==0){
         if (oldidlestatus==1){
         idlestatus=1;
         }
         else {
         oldidlestatus=1;
         }
     }    
     else {
     oldidlestatus=0; 
     idlestatus=0;    
     }
 }
   

if (powercounter==55) {
  SOCfreeze=0;
}

if (switchpress==1) {
  lcd.setCursor(0,0);
  lcd.print("Mode 2a: allow Imp. ");
  lcd.setCursor(0,1);
  if (chargerone==1 and chargertwo==0) {
    lcd.print("C1:ON C2:OFF  ");
  }
  if (chargerone==0 and chargertwo==1) {
    lcd.print("C1:OFF C2:ON  ");
  }
  if (chargerone==1 and chargertwo==1) {
    lcd.print("C1:ON  C2:ON  ");
  }
 if (chargerone==0 and chargertwo==0) {
    lcd.print("C1:OFF C2:OFF ");
  }   

 getVoltage(); 

 getSOC();   

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

  // date
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
 // time
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);

   lcd.setCursor(14,1);
  if (inverter==1) {
    lcd.print("In:ON ");
  }
  if (inverter==0) {
    lcd.print("In:OFF");
  }

}
else {
    lcd.setCursor(0,0);
  lcd.print("Mode 2b: allow Imp. ");
  lcd.setCursor(0,1);
  if (showpower==0) {
  lcd.print("P-averg:");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(averagepowerpool);
  }
  else {
  lcd.print("P-real: ");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(measuredpower);  
  }
  lcd.setCursor(19,1);
  if (averagepowerpool<lastaveragepower) {
    lcd.print("v");
  }
  else {
    lcd.print("^");
  }



  lcd.setCursor(0,2);
  lcd.print("Temp:");
  lcd.print(tempC);
  lcd.setCursor(12,2);
  lcd.print("RH:");
  lcd.print(humidity); 



  lcd.setCursor(0,3);  
  if (typeofbatt==0) { 
    lcd.print("LIPO     "); 
  } 
  if (typeofbatt==1) { 
    lcd.print("LIFEPO   ");
  } 
  if (typeofbatt==2) { 
    lcd.print("NMC      ");
  } 
 
 
 
  lcd.setCursor(9,3);
  if (invertreadout==1) { 
    lcd.print("invd  ");
  }
  if (invertreadout==0) { 
    lcd.print("norm  ");
  }

  lcd.setCursor(15,3);
    lcd.print(corrindrest);
    lcd.print("/");
    lcd.print(corrindchg);
    lcd.print("/");
    lcd.print(corrindinv);

}
  

   invertertime=(dt.hour);
   if (battSOC>minimumSOC+10){
    inverteravailable=1; 
   }

   if (battSOC<=minimumSOC){
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    inverteravailable=0;
   }
  
  if (inverteravailable==1){  
  if (NormInvOff<NormInvOn) {   
  if (invertertime>=NormInvOn or invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
        digitalWrite(relayPin3,LOW);
        inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }

 if (NormInvOff>NormInvOn) {   
  if (invertertime>=NormInvOn and invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
      if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
      digitalWrite(relayPin3,LOW);
      inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }
 
 } 
 


}


void Mode3(){

  // Timer Based (Mode3)
   switchpress=digitalRead(SW);
  dt = clock.getDateTime();
  timermatch=0;
  timer2match=0;
  timer3match=0;
  actC1Timer=0;
  actC2Timer=0;
  actInvTimer=0;
 


   invertertime=(dt.hour);
    if (battSOC>minimumSOC+10){
       inverteravailable=1; 
     }

    if (battSOC<=minimumSOC){
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    inverteravailable=0;
   }

if (inverteravailable==1){
   
 if (T1InvOff<T1InvOn) {   
  if (invertertime>=T1InvOn or invertertime<T1InvOff) {              
     if (battSOC>minimumSOC) {
  timermatch=1;
  inverter=1;
  showInvOn=T1InvOn;
  showInvOff=T1InvOff;
  actInvTimer=1;
     }
  }
 else {
  inverter=0;
  }
 }

 if (T1InvOff>T1InvOn) {   
  if (invertertime>=T1InvOn and invertertime<T1InvOff) {              
     if (battSOC>minimumSOC) {
  timermatch=1;
  inverter=1;
  showInvOn=T1InvOn;
  showInvOff=T1InvOff;
  actInvTimer=1;
     }
  }
 else {
  inverter=0;
  }
 }

if (timermatch==0) {
   
if (T2InvOff<T2InvOn) {   
  if (invertertime>=T2InvOn or invertertime<T2InvOff) {              
     if (battSOC>minimumSOC) {
  inverter=1;
  showInvOn=T2InvOn;
  showInvOff=T2InvOff;
  actInvTimer=2;
     }
  }
 else {
  inverter=0;
  }
 }

 if (T2InvOff>T2InvOn) {   
  if (invertertime>=T2InvOn and invertertime<T2InvOff) {              
     if (battSOC>minimumSOC) {
  inverter=1;
  showInvOn=T2InvOn;
  showInvOff=T2InvOff;
  actInvTimer=2;
     }
  }
 else {
  inverter=0;
  }
 }
 }

}

 if (battSOC<=minimumSOC){
       inverteravailable=0; 
     }
 
  C1time=(dt.hour);
  
 if (T1C1Off<T1C1On) {   
  if (C1time>=T1C1On or C1time<T1C1Off) {
    if (inverter==0){               
  timer2match=1;
  chargerone=1;
  showC1On=T1C1On;
  showC1Off=T1C1Off;
  actC1Timer=1;
    }
  }
 else {
  chargerone=0;
  }
 }
 

if (T1C1Off>T1C1On) {   
  if (C1time>=T1C1On and C1time<T1C1Off) {
    if (inverter==0){               
  timer2match=1;
  chargerone=1;
  showC1On=T1C1On;
  showC1Off=T1C1Off;
  actC1Timer=1;
    }
  }
 else {
  chargerone=0;
  }
 }

if (timer2match==0) {
 if (T2C1Off<T2C1On) {   
  if (C1time>=T2C1On or C1time<T2C1Off) {
    if (inverter==0){               
  chargerone=1;
  showC1On=T2C1On;
  showC1Off=T2C1Off;
  actC1Timer=2;
    }
  }
 else {
  chargerone=0;
  }
 }
 

if (T2C1Off>T2C1On) {   
  if (C1time>=T2C1On and C1time<T2C1Off) {
    if (inverter==0){               
  chargerone=1;
  showC1On=T2C1On;
  showC1Off=T2C1Off;
  actC1Timer=2;
    }
  }
 else {
  chargerone=0;
  }
 }
}

   C2time=(dt.hour);
   
 if (T1C2Off<T1C2On) {   
  if (C2time>=T1C2On or C2time<T1C2Off) {
    if (inverter==0){               
  timer3match=1;
  chargertwo=1;
  showC2On=T1C2On;
  showC2Off=T1C2Off;
  actC2Timer=1;
    }
  }
 else {
  chargertwo=0;
  }
 }
 

if (T1C2Off>T1C2On) {   
  if (C2time>=T1C2On and C2time<T1C2Off) {
    if (inverter==0){               
  timer3match=1;
  chargertwo=1;
  showC2On=T1C2On;
  showC2Off=T1C2Off;
  actC2Timer=1;
    }
  }
 else {
  chargertwo=0;
  }
 }

if (timer3match==0) {
 if (T2C2Off<T2C2On) {   
  if (C2time>=T2C2On or C2time<T2C2Off) {
    if (inverter==0){               
  chargertwo=1;
  showC2On=T2C2On;
  showC2Off=T2C2Off;
  actC2Timer=2;
    }
  }
 else {
  chargertwo=0;
  }
 }
 

if (T2C2Off>T2C2On) {   
  if (C2time>=T2C2On and C2time<T2C2Off) {
    if (inverter==0){               
  chargertwo=1;
  showC2On=T2C2On;
  showC2Off=T2C2Off;
  actC2Timer=2;
    }
  }
 else {
  chargertwo=0;
  }
 }
}  

 getVoltage(); 
 SOCfreeze=0;
 getSOC(); 
 

     
 if (switchpress==1) { 
  lcd.setCursor(0,0);
  lcd.print("Mode 3a: Timer Based");
      
  if (inverter==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF Inv:ON");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
    digitalWrite(relayPin3,LOW);
  }
  
  
  if (battSOC>maximumSOC){
    chargerone=0;
    chargertwo=0;
  }
  if (typeofbatt==1 and corrindchg==7) {
  chargerlock=1;
  }
  if (typeofbatt==1 and inverter==1) {
  chargerlock=0;
  }
  
  if (inverter==0 and chargerone==1 and chargertwo==0){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:ON C2:OFF Inv:OFF");
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
  if (inverter==0 and chargerone==0 and chargertwo==0){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
 
 if (inverter==0 and chargerone==0 and chargertwo==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:ON Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,LOW);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }

   if (inverter==0 and chargerone==1 and chargertwo==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:ON C2:ON Inv:OFF ");
    digitalWrite(relayPin1,LOW);
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     lcd.setCursor(0,1);
     lcd.print("C1:ON C2:ON Inv:OFF ");
     }
     else {
     digitalWrite(relayPin2,HIGH);
     chargertwo=0; 
     lcd.setCursor(0,1);
     lcd.print("C1:ON C2:OFF Inv:OFF");
     }
     digitalWrite(relayPin5,HIGH);
     digitalWrite(relayPin3,HIGH);
     stopdelay=0;
  }
 
    if (chargerone==0 and chargertwo==0 and inverter==0){
         if (oldidlestatus==1){
         idlestatus=1;
         }
         else {
         oldidlestatus=1;
         }
    }
     else {
     oldidlestatus=0; 
     idlestatus=0;    
     }
    

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

// date
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
  // time
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);
}
 else {
   lcd.setCursor(0,0);
  lcd.print("Mode 3b: Act. Timer ");
  lcd.setCursor(0,1);
  lcd.print("C1: ");
  lcd.setCursor(4,1);
  lcd.print("         ");
  lcd.setCursor(4,1);
  lcd.print(showC1On);
  lcd.print("-");  
  lcd.print(showC1Off);
  lcd.setCursor(13,1);
  if (actC1Timer==1){
  lcd.print("Timer 1");
  }
  if (actC1Timer==2){
  lcd.print("Timer 2");
  }
  if (actC1Timer==0){
  lcd.print("Waiting");
  }
  lcd.setCursor(0,2);
  lcd.print("C2: ");
  lcd.setCursor(4,2);
  lcd.print("         ");
  lcd.setCursor(4,2);
  lcd.print(showC2On);
  lcd.print("-");  
  lcd.print(showC2Off);
  lcd.setCursor(13,2);
  if (actC2Timer==1){
  lcd.print("Timer 1");
  }
  if (actC2Timer==2){
  lcd.print("Timer 2");
  }
  if (actC2Timer==0){
  lcd.print("Waiting");
  }
  lcd.setCursor(0,3);
  lcd.print("Inv: ");
  lcd.setCursor(5,3);
  lcd.print("        ");
  lcd.setCursor(5,3);
  lcd.print(showInvOn);
  lcd.print("-");  
  lcd.print(showInvOff);
  lcd.setCursor(13,3);
  if (actInvTimer==1){
  lcd.print("Timer 1");
  }
  if (actInvTimer==2){
  lcd.print("Timer 2");
  }
  if (actInvTimer==0){
  lcd.print("Waiting");
  } 
}


}

void Mode4(){

  // Mixed M1/T1 (Mode4)
  
   switchpress=digitalRead(SW);
  dt = clock.getDateTime();

    averagepower=averagepower+measuredpower;
    averagepowerpool=averagepower/powercounter;
    powercounter++;

  
   invertertime=(dt.hour);
    if (battSOC>minimumSOC+10){
       inverteravailable=1; 
     }

    if (battSOC<=minimumSOC){
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    inverteravailable=0;
   }


if (powercounter>=relaycycletime) {
  
  timermatch=0;
  timer2match=0;
  timer3match=0;
  

if (inverteravailable==1){
   
 if (T1InvOff<T1InvOn) {   
  if (invertertime>=T1InvOn or invertertime<T1InvOff) {              
     if (battSOC>minimumSOC) {
  timermatch=1;
  inverter=1;
  showInvOn=T1InvOn;
  showInvOff=T1InvOff;
  actInvTimer=1;
     }
  }
  else{
  inverter=0;  
  }
 
 }

 if (T1InvOff>T1InvOn) {   
  if (invertertime>=T1InvOn and invertertime<T1InvOff) {              
     if (battSOC>minimumSOC) {
  timermatch=1;
  inverter=1;
  showInvOn=T1InvOn;
  showInvOff=T1InvOff;
  actInvTimer=1;
     }
  }
  else{
  inverter=0;  
  }
 
 }

}


  C1time=(dt.hour);
  
 if (T1C1Off<T1C1On) {   
  if (C1time>=T1C1On or C1time<T1C1Off) {
    if (inverter==0){               
  timer2match=1;
  chargerone=1;
  showC1On=T1C1On;
  showC1Off=T1C1Off;
  actC1Timer=1;
    }
  }
 
 }
 

if (T1C1Off>T1C1On) {   
  if (C1time>=T1C1On and C1time<T1C1Off) {
    if (inverter==0){               
  timer2match=1;
  chargerone=1;
  showC1On=T1C1On;
  showC1Off=T1C1Off;
  actC1Timer=1;
    }
  }
 
 }


   C2time=(dt.hour);
   
 if (T1C2Off<T1C2On) {   
  if (C2time>=T1C2On or C2time<T1C2Off) {
    if (inverter==0){               
  timer3match=1;
  chargertwo=1;
  showC2On=T1C2On;
  showC2Off=T1C2Off;
  actC2Timer=1;
    }
  }
 
 }
 

if (T1C2Off>T1C2On) {   
  if (C2time>=T1C2On and C2time<T1C2Off) {
    if (inverter==0){               
  timer3match=1;
  chargertwo=1;
  showC2On=T1C2On;
  showC2Off=T1C2Off;
  actC2Timer=1;
    }
  }
 
 }


if (timermatch==0 and timer2match==0 and timer3match==0){
  
   
   if (chargerone==1 and chargertwo==0) {
    realpower=averagepowerpool+(battvoltage*ConeAmps);
    }
    if (chargerone==0 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*CtwoAmps);
    }
    if (chargerone==1 and chargertwo==1) {
    realpower=averagepowerpool+(battvoltage*(ConeAmps+CtwoAmps));
    }
    if (chargerone==0 and chargertwo==0) {
    realpower=averagepowerpool;
    }
    


if (battSOC<=maximumSOC) {
 
if (typeofbatt==1 and corrindchg==7) {
  chargerlock=1;
}
if (typeofbatt==1 and inverter==1) {
  chargerlock=0;
}
  
   if (realpower<=((triggerpowermode1*10)+(50*ConeAmps))) {
     chargerone=0;
     chargertwo=0;
   } 
   if (realpower>((triggerpowermode1*10)+(50*ConeAmps)) and realpower<((triggerpowermode1*10)+(50*CtwoAmps))) {
     chargerone=1;
     chargertwo=0;
   }
    if (realpower>=((triggerpowermode1*10)+(50*CtwoAmps)) and realpower<((triggerpowermode1*10)+(50*ConeAmps)+(50*CtwoAmps))) {
     chargerone=0;
     chargertwo=1;
   } 
   if (realpower>=((triggerpowermode1*10)+(50*ConeAmps)+(50*CtwoAmps))) {
     chargerone=1;
     chargertwo=1;
     }
   } 
 else{
     chargerone=0;
     chargertwo=0;
  
 }
}
   
    SOCfreeze=1;
    lastaveragepower=averagepowerpool;
    averagepower=0;
    powercounter=1;
    if (chargerone==0 and chargertwo==0 and inverter==0){
         if (oldidlestatus==1){
         idlestatus=1;
         }
         else {
         oldidlestatus=1;
         }
    }
     else {
     oldidlestatus=0; 
     idlestatus=0;    
     }
  }


if (powercounter==55) {
  SOCfreeze=0;
}

 if (switchpress==1) { 
  lcd.setCursor(0,0);
  lcd.print("Mode 4a: Mixed T1/M1");
      
 if (timermatch==1){ 
  if (inverter==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF Inv:ON");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
    digitalWrite(relayPin3,LOW);
  }
 }
  
  if (inverter==0 and chargerone==1 and chargertwo==0){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:ON C2:OFF Inv:OFF");
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
  if (inverter==0 and chargerone==0 and chargertwo==0){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
 
 if (inverter==0 and chargerone==0 and chargertwo==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:ON Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,LOW);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }

   if (inverter==0 and chargerone==1 and chargertwo==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:ON C2:ON Inv:OFF ");
    digitalWrite(relayPin1,LOW);
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     }
     else {
     digitalWrite(relayPin2,HIGH); 
     }
     digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }

      

 getVoltage(); 
 
 getSOC();    

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");


  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
 
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);
}
 else {
    lcd.setCursor(0,0);
  lcd.print("Mode 4b: Mixed T1/M1");
  lcd.setCursor(0,1);
  if (showpower==0) {
  lcd.print("P-averg:");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(averagepowerpool);
  }
  else {
  lcd.print("P-real: ");
  lcd.setCursor(8,1);
  lcd.print("            ");
  lcd.setCursor(8,1);
  lcd.print(measuredpower);  
  }
  lcd.setCursor(19,1);
  if (averagepowerpool<lastaveragepower) {
    lcd.print("v");
  }
  else {
    lcd.print("^");
  }



  lcd.setCursor(0,2);
  lcd.print("Temp:");
  lcd.print(tempC);
  lcd.setCursor(12,2);
  lcd.print("RH:");
  lcd.print(humidity); 



  lcd.setCursor(0,3);  
  if (typeofbatt==0) { 
    lcd.print("LIPO     "); 
  } 
  if (typeofbatt==1) { 
    lcd.print("LIFEPO   ");
  } 
  if (typeofbatt==2) { 
    lcd.print("NMC      ");
  } 
 
  lcd.setCursor(9,3);
  if (invertreadout==1) { 
    lcd.print("invd  ");
  }
  if (invertreadout==0) { 
    lcd.print("norm  ");
  }

  lcd.setCursor(15,3);
    lcd.print(corrindrest);
    lcd.print("/");
    lcd.print(corrindchg);
    lcd.print("/");
    lcd.print(corrindinv);
 }

    


  if (timermatch==0 and timer2match==0 and timer3match==0){
  if (inverteravailable==1){  
  if (NormInvOff<NormInvOn) {   
  if (invertertime>=NormInvOn or invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
        digitalWrite(relayPin3,LOW);
        inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }

 if (NormInvOff>NormInvOn) {   
  if (invertertime>=NormInvOn and invertertime<NormInvOff) {              
     if (battSOC>minimumSOC) {
      if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
      digitalWrite(relayPin3,LOW);
      inverter=1;
     }
  }
 else {
  digitalWrite(relayPin5,HIGH);
  digitalWrite(relayPin3,HIGH);
  inverter=0;
  stopdelay=0;
  }
 }
  }

 if (inverter==1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF Inv:ON");
 }
 else{
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("C1:OFF C2:OFF In:OFF");
 }
 } 

}




void Mode5(){

   // Charger 1 - manuell ON (Mode5), Relays are ON when "L0W"
   
  lcd.setCursor(0,0);
  lcd.print("Mode 5: Charger 1 ON");
    lcd.setCursor(0,1);
    
  if (battSOC<=99){                             // only this mode allows 100% charge
    lcd.print("C1:ON C2:OFF Inv:OFF ");
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=1;
    chargertwo=0;
    inverter=0;
    stopdelay=0;
  }
  else {
    lcd.print("C1:OFF C2:OFF Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=0;
    chargertwo=0;
    inverter=0;
    stopdelay=0;
  }
    

 getVoltage();  

 getSOC();    

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

  
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
  
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);


}


void Mode6(){

  // Charger 2 - manuell ON (Mode6), Relays are ON when "L0W"
   
  lcd.setCursor(0,0);
  lcd.print("Mode 6: Charger 2 ON");
    lcd.setCursor(0,1);
    if (battSOC<=maximumSOC){  
    lcd.print("C1:ON C2:OFF Inv:OFF ");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,LOW);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=0;
    chargertwo=1;
    inverter=0;
    stopdelay=0;
  }
  else {
    lcd.print("C1:OFF C2:OFF Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=0;
    chargertwo=0;
    inverter=0;
    stopdelay=0;
  }

 getVoltage(); 

 getSOC();    

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

 
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
 
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);


}


void Mode7(){

  // Charger 1+2 - manuell ON (Mode7), Relays are ON when "L0W"

if (typeofbatt==1 and corrindchg==7) {
  chargerlock=1;
}
  
  lcd.setCursor(0,0);
  lcd.print("Mode 7: Charger12 ON");
   if (battSOC<=maximumSOC){    
    lcd.setCursor(0,1);
    lcd.print("C1:ON C2:ON Inv:OFF ");
    digitalWrite(relayPin1,LOW);
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
     }
     else {
     digitalWrite(relayPin2,HIGH);
     chargertwo=0; 
     }
     digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=1;
    inverter=0;
    stopdelay=0;
   }
   else {
    lcd.print("C1:OFF C2:OFF Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=0;
    chargertwo=0;
    inverter=0;
    stopdelay=0;
  }

 getVoltage(); 
 
 getSOC();  

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

 // date
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
  // time
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);


}


void Mode8(){

  // Discharge Battery (Mode8), Relays are ON when "L0W"

     if (battSOC>minimumSOC+3){
       inverteravailable=1; 
     }

if (typeofbatt==1 and inverter==1) {
  chargerlock=0;
}

    if (battSOC<=0){                           // discharge mode can go always to zero!!!
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    inverteravailable=0;
   }

  
  if (inverteravailable==1){  
    if (battSOC>0) {
         if (stopdelay==0){    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
    digitalWrite(relayPin3,LOW);
    inverter=1;
    }
    else  {
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    }
  }

  lcd.setCursor(0,0);
  lcd.print("Mode 8: Discharge   ");
    lcd.setCursor(0,1);
    if (inverter==1){
    lcd.print("C1:OFF C2:OFF Inv:ON");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    chargerone=0;
    chargertwo=0;
    }
    else {
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    chargerone=0;
    chargertwo=0;
    }

 getVoltage();  
 
 getSOC();   

  
    lcd.setCursor(0,2);
    lcd.print("BatV:");
    lcd.setCursor(8,2);
    lcd.print("     ");
    lcd.setCursor(5,2);
    lcd.print(battvoltage);
    lcd.setCursor(12,2);
    lcd.print("SOC:");
    lcd.setCursor(17,2);
    lcd.print("  ");
    lcd.setCursor(16,2);
    lcd.print(battSOC);
    lcd.setCursor(19,2);
    lcd.print("%");

 
  dt = clock.getDateTime();
  lcd.setCursor(9,3);
  lcd.print("   ");
  lcd.setCursor(0,3);
  if (dt.day<10) { 
    lcd.print("0");
    }
  lcd.print(dt.day);
  lcd.print(".");
  if (dt.month<10) { 
    lcd.print("0");
    }
  lcd.print(dt.month);
  lcd.print(".");
  lcd.print(dt.year);
 
  lcd.setCursor(12,3);
  if (dt.hour<10) { 
    lcd.print("0");
    }
  lcd.print(dt.hour);
  lcd.print(":");
  if (dt.minute<10) { 
    lcd.print("0");
    }
  lcd.print(dt.minute);
  lcd.print(":");
  if (dt.second<10) {
    lcd.print("0"); 
    }
  lcd.print(dt.second);


}


void Mode9(){

  // Timer Program (Mode9)
  
if (setpos<=7) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9a: Set Timer 1");
  if (setpos==0) {
  counter=setT1C1On;  
  lcd.setCursor(0,1);
  lcd.print("Press Knob to start ");
  lcd.setCursor(0,2);
  lcd.print("Dischg prio over chg");
  lcd.setCursor(0,3);
  lcd.print("T1 priority over T2 ");
  }
  else {
  lcd.setCursor(0,1);
  lcd.print("C1 ON:");
  lcd.setCursor(6,1);
  lcd.print("    ");
  lcd.setCursor(7,1);
  lcd.print(setT1C1On);
  lcd.setCursor(9,1);
  if (setpos==2){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(11,1);
  lcd.print("OFF:");
  lcd.setCursor(15,1);
  lcd.print("     ");
  lcd.setCursor(16,1);
  lcd.print(setT1C1Off);
  lcd.setCursor(18,1);
  if (setpos==3){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
   lcd.setCursor(0,2);
  lcd.print("C2 ON:");
  lcd.setCursor(6,2);
  lcd.print("     ");
  lcd.setCursor(7,2);
  lcd.print(setT1C2On);
  lcd.setCursor(9,2);
  if (setpos==4){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(11,2);
  lcd.print("OFF:");
  lcd.setCursor(15,2);
  lcd.print("     ");
  lcd.setCursor(16,2);
  lcd.print(setT1C2Off);
  lcd.setCursor(18,2);
  if (setpos==5){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
   lcd.setCursor(0,3);
  lcd.print("Inv ON:");
  lcd.setCursor(7,3);
  lcd.print("    ");
  lcd.setCursor(8,3);
  lcd.print(setT1InvOn);
  lcd.setCursor(10,3);
  if (setpos==6){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(12,3);
  lcd.print("OFF:");
  lcd.setCursor(16,3);
  lcd.print("   ");
  lcd.setCursor(17,3);
  lcd.print(setT1InvOff);
  lcd.setCursor(19,3);
  if (setpos==7){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  }
  }
  
  if (setpos>7 and setpos<=13) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9b: Set Timer 2");
  lcd.setCursor(0,1);
  lcd.print("C1 ON:");
  lcd.setCursor(6,1);
  lcd.print("     ");
  lcd.setCursor(7,1);
  lcd.print(setT2C1On);
  lcd.setCursor(9,1);
  if (setpos==8){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(11,1);
  lcd.print("OFF:");
  lcd.setCursor(15,1);
  lcd.print("     ");
  lcd.setCursor(16,1);
  lcd.print(setT2C1Off);
  lcd.setCursor(18,1);
  if (setpos==9){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
   lcd.setCursor(0,2);
  lcd.print("C2 ON:");
  lcd.setCursor(6,2);
  lcd.print("     ");
  lcd.setCursor(7,2);
  lcd.print(setT2C2On);
  lcd.setCursor(9,2);
  if (setpos==10){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(11,2);
  lcd.print("OFF:");
  lcd.setCursor(15,2);
  lcd.print("     ");
  lcd.setCursor(16,2);
  lcd.print(setT2C2Off);
  lcd.setCursor(18,2);
  if (setpos==11){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
   lcd.setCursor(0,3);
  lcd.print("Inv ON:");
  lcd.setCursor(7,3);
  lcd.print("    ");
  lcd.setCursor(8,3);
  lcd.print(setT2InvOn);
  lcd.setCursor(10,3);
  if (setpos==12){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(12,3);
  lcd.print("OFF:");
  lcd.setCursor(16,3);
  lcd.print("   ");
  lcd.setCursor(17,3);
  lcd.print(setT2InvOff);
  lcd.setCursor(19,3);
  if (setpos==13){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }         
  }

  if (setpos>13) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9c: Other Modes");
  lcd.setCursor(0,1);
  lcd.print("Inv ON:");
  lcd.setCursor(7,1);
  lcd.print("     ");
  lcd.setCursor(8,1);
  lcd.print(setNormInvOn);
  lcd.setCursor(10,1);
  if (setpos==14){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(12,1);
  lcd.print("OFF:");
  lcd.setCursor(16,1);
  lcd.print("    ");
  lcd.setCursor(16,1);
  lcd.print(setNormInvOff);
  lcd.setCursor(19,1);
  if (setpos==15){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
   lcd.setCursor(0,2);
  lcd.print("Daily reset:        ");
  lcd.setCursor(14,2);
  if (setDreset==0){
   lcd.print("NO "); 
  }
  else{
   lcd.print("YES"); 
  }
  lcd.setCursor(19,2);
  if (setpos==16){
  lcd.print("<");  
  }
  else {
  lcd.print("");
  }
  lcd.setCursor(0,3);
  lcd.print("At: ");
  lcd.setCursor(4,3);
  lcd.print("    ");
  lcd.setCursor(4,3);
  lcd.print(setDresethr);
  lcd.setCursor(6,3);
  if (setpos==17){
  lcd.print("<");  
  }
  else {
  lcd.print("h");
  }
  lcd.setCursor(7,3);
  lcd.print("             ");
  lcd.setCursor(8,3);
  lcd.print(setDresetmin);
  lcd.setCursor(10,3);
  if (setpos==18){
  lcd.print("<  ");  
  }
  else {
  lcd.print("min");
  }
  
  }

  switchpress=digitalRead(SW);
  delay(50);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    buttonrelease=1;
    }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==2){
 // set Timer 1, Charger 1 On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
  
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter ++;
       }
     if (counter>23) { counter=0;
     }
     setT1C1On=counter;
   } }

if (setpos==3){
 // set Timer 1, Charger 1 Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>23) { counter=0;
     }
     setT1C1Off=counter;
   }  }  

   if (setpos==4){
 // set Timer 1, Charger 2 On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       

     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>23) { counter=0;
     }
     setT1C2On=counter;
   } }

     if (setpos==5){
 // set set Timer 1, Charger 2 Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT1C2Off=counter;
   } }

     if (setpos==6){
 // set set Timer 1, Inverter On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       

     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT1InvOn=counter;
   } }

    if (setpos==7){
 // set Timer 1, Inverter Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       

     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT1InvOff=counter;
   } }

   if (setpos==8){
 // set Timer 2, Charger 1 On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>23) { counter=0;
     }
     setT2C1On=counter;
   } }

if (setpos==9){
 // set Timer 2, Charger 1 Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter ++;
       }
     if (counter>23) { counter=0;
     }
     setT2C1Off=counter;
   }  }  

   if (setpos==10){
 // set Timer 2, Charger 2 On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter ++;
       }
     
     if (counter>23) { counter=0;
     }
     setT2C2On=counter;
   } }

     if (setpos==11){
 // set Timer 2, Charger 2 Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT2C2Off=counter;
   } }

     if (setpos==12){
 // set Timer 2, Inverter On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT2InvOn=counter;
   } }

    if (setpos==13){
 // set Timer 2, Inverter Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setT2InvOff=counter;
   } }

      if (setpos==14){
 // other modes, Inverter On
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setNormInvOn=counter;
   } }

      if (setpos==15){
 // set Timer 2, Inverter Off
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setNormInvOff=counter;
   } }

       if (setpos==16){
 // set daily reset
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>1) { counter=0;
     }
     setDreset=counter;
   } }
 
       if (setpos==17){
 // set daily reset hr
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>23) { counter=0;
     }
     setDresethr=counter;
   } }

     if (setpos==18){
 // set daily reset min
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
 
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter++;
       }
     
     if (counter>59) { counter=0;
     }
     setDresetmin=counter;
   } }

 
  if (setpos==19){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 9: Set Timers ");
   lcd.setCursor(7,2);
   lcd.print("DONE");
   // The following lines set parameters

   T1C1On=setT1C1On;
   T1C1Off=setT1C1Off;
   T1C2On=setT1C2On;
   T1C2Off=setT1C2Off;
   T1InvOn=setT1InvOn;
   T1InvOff=setT1InvOff;
   T2C1On=setT2C1On;
   T2C1Off=setT2C1Off;
   T2C2On=setT2C2On;
   T2C2Off=setT2C2Off;
   T2InvOn=setT2InvOn;
   T2InvOff=setT2InvOff;
   NormInvOn=setNormInvOn;
   NormInvOff=setNormInvOff;
   Dreset=setDreset;
   Dresethr=setDresethr;
   Dresetmin=setDresetmin;
   //write limits to EEPROM
   EEPROM.write(7,T1C1On);
   EEPROM.write(8,T1C2On);
   EEPROM.write(9,T1InvOn);
   EEPROM.write(10,T1C1Off);
   EEPROM.write(11,T1C2Off);
   EEPROM.write(12,T1InvOff);
   EEPROM.write(13,T2C1On);
   EEPROM.write(14,T2C2On);
   EEPROM.write(15,T2InvOn);
   EEPROM.write(16,T2C1Off);
   EEPROM.write(17,T2C2Off);
   EEPROM.write(18,T2InvOff);
   EEPROM.write(24,NormInvOn);
   EEPROM.write(25,NormInvOff);
   EEPROM.write(26,Dreset);
   EEPROM.write(27,Dresethr);
   EEPROM.write(28,Dresetmin);
   
   delay(4000);
   setpos=0;
   }

   previousStateCLK = currentStateCLK;  

}

void Mode10(){
 
  //Set Limits (Mode10)

 if (setpos<=3) {
  counter=setminimumSOC;
  lcd.setCursor(0,0);
  lcd.print("Mode 10a: Set Limits");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("SOC Min:");
  lcd.setCursor(8,2);
  lcd.print("  ");
  lcd.setCursor(8,2);
  lcd.print(setminimumSOC);
  lcd.setCursor(10,2);
  if (setpos==1){
  lcd.print("<");  
  }
  else {
  lcd.print("%");
  }
  lcd.setCursor(12,2);
  lcd.print("Max:");
  lcd.setCursor(16,2);
  lcd.print("   ");
  lcd.setCursor(16,2);
  lcd.print(setmaximumSOC);
  lcd.setCursor(19,2);
  if (setpos==2){
  lcd.print("<");  
  }
  else {
  lcd.print("%");
  }
  lcd.setCursor(0,3);
  lcd.print("Fan Act Temp:");
  lcd.setCursor(13,3);
  lcd.print("       ");
  lcd.setCursor(13,3);
  lcd.print(round(setfanActtemp));  
  if (setpos==3){
  lcd.setCursor(15,3);  
  lcd.print("<");  
  }
  else {
  lcd.print("C");  
  }
 }
  if (setpos>3) {
  lcd.setCursor(0,0);
  lcd.print("Mode 10b: Set Limits");
  lcd.setCursor(0,1);
  lcd.print("Mode1 deltaP: +");
  lcd.setCursor(15,1);
  lcd.print("    ");
  lcd.setCursor(15,1);
  lcd.print(settriggerpowermode1*10);
  lcd.setCursor(19,1);
  if (setpos==4){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  lcd.setCursor(0,2);
  lcd.print("Mode2 deltaP: -");
  lcd.setCursor(15,2);
  lcd.print("    ");
  lcd.setCursor(15,2);
  lcd.print(settriggerpowermode2*10);
  lcd.setCursor(19,2);
  if (setpos==5){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  lcd.setCursor(0,3);
  lcd.print("Vread cal: ");
  lcd.setCursor(11,3);
  lcd.print("    ");
  lcd.setCursor(11,3);
  lcd.print(round(setfineadjustmentbyte-20));
  lcd.setCursor(15,3);
  if (setpos==6){
  lcd.print("<    ");  
  }
  else {
  lcd.print("units");
  }
  }

  switchpress=digitalRead(SW);
  delay(50);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set minimum SOC
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       

     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>49) { counter=0;
     }
     setminimumSOC=counter;
   } }

if (setpos==2){
 // set month
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter ++;
       }
     if (counter<50 or counter>100) { counter=50;
     }
     setmaximumSOC=counter;
   }  }  

   if (setpos==3){
 // set year
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter<20 or counter>40) { counter=20;
     }
     setfanActtemp=counter;
   } }

     if (setpos==4){
 // set trigger mode 1
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter=counter+10;
       } 
       else {
       counter=counter+10;
       }
     
     if (counter<10 or counter>90) { counter=10;
     }
     settriggerpowermode1=counter;
   } }

     if (setpos==5){
 // set trigger mode 1
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter=counter+10;
       } 
       else {
       // Encoder is rotating clockwise
       counter=counter+10;
       }
     
     if (counter<10 or counter>50) { counter=10;
     }
     settriggerpowermode2=counter;
   } }

    if (setpos==6){
 // set voltage readout fine tuning
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
      if (digitalRead(DT) != currentStateCLK) { 
       counter=counter+1;
       } 
       else {
       counter=counter+1;
       }
     
     if (counter<0 or counter>40) { counter=0;
     }
     setfineadjustmentbyte=counter;
   } }
   
  if (setpos==7){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 10: Set Limits ");
   lcd.setCursor(7,2);
   lcd.print("DONE");
   // The following lines set parameters

   minimumSOC=setminimumSOC;
   maximumSOC=setmaximumSOC;
   fanActtemp=setfanActtemp;
   triggerpowermode1=settriggerpowermode1;
   triggerpowermode2=settriggerpowermode2;
   fineadjustmentbyte=setfineadjustmentbyte;
   //write limits to EEPROM
   EEPROM.write(1,minimumSOC);
   EEPROM.write(2,maximumSOC);
   EEPROM.write(3,fanActtemp);
   EEPROM.write(19,triggerpowermode1);
   EEPROM.write(20,triggerpowermode2);
   EEPROM.write(23,fineadjustmentbyte);
   
   delay(4000);

   fineadjustment=(fineadjustmentbyte-20)/500000;
   
   setpos=0;
   }
   previousStateCLK = currentStateCLK;  
}

void Mode11(){

  // Set parameters (Mode11)
  if (setpos<=2) {
  counter=setConeAmps;
  lcd.setCursor(0,0);
  lcd.print("Mode 11a: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("Chg 1 Amps:");
  lcd.setCursor(11,2);
  lcd.print("         ");
  lcd.setCursor(13,2);
  lcd.print(setConeAmps);
  lcd.setCursor(17,2);
  if (setpos==1){
  lcd.print("<");  
  }
  else {
  lcd.print("A");
  }
  lcd.setCursor(0,3);
  lcd.print("Chg 2 Amps:");
  lcd.setCursor(11,3);
  lcd.print("         ");
  lcd.setCursor(13,3);
  lcd.print(setCtwoAmps);
  lcd.setCursor(17,3);
  if (setpos==2){
  lcd.print("<");  
  }
  else {
  lcd.print("A");
  }
  }
  if (setpos>2 and setpos<=5) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11b: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Batt Type:");
  lcd.setCursor(10,1);
  lcd.print("         ");
  lcd.setCursor(11,1);
  if (settypeofbatt==0) {
  lcd.print("LiPo");
  }
  if (settypeofbatt==1) {
  lcd.print("LiFePo");
  }
  if (settypeofbatt==2) {
  lcd.print("NMC");
  }
  lcd.setCursor(19,1);
  if (setpos==3){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  lcd.setCursor(0,2);
  lcd.print("Show Power:");
  lcd.setCursor(12,2);
  lcd.print("        ");
  lcd.setCursor(12,2);
  if (setshowpower==0) {
  lcd.print("Average ");
  }
  if (setshowpower==1) {
  lcd.print("Real    ");
  }
  lcd.setCursor(19,2);
  if (setpos==4){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  lcd.setCursor(0,3);
  lcd.print("Inverse Readout:");
  lcd.setCursor(16,3);
  lcd.print("    ");
  lcd.setCursor(16,3);
  if (setinvertreadout==1) {
  lcd.print("YES ");
  }
  if (setinvertreadout==0) {
  lcd.print("NO  ");
  }
  
  lcd.setCursor(19,3);
  if (setpos==5){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  }
  if (setpos>5) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11c: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Inv Curr Sensor:    ");
  lcd.setCursor(16,1);
  lcd.print(setinvcurrsens);
  lcd.setCursor(19,1);
  if (setpos==6){
  lcd.print("<");  
  }
  else {
  lcd.print("A");
  }
  lcd.setCursor(0,2);
  lcd.print("Use Wifi:           ");
  lcd.setCursor(10,1);
  if (setwifiavail==1) {
  lcd.print("YES ");
  }
  if (setwifiavail==0) {
  lcd.print("NO  ");
  }
  lcd.setCursor(15,1);
  if (setpos==7){
  lcd.print("<");  
  }
  else {
  lcd.print(" ");
  }
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }
    
    
  switchpress=digitalRead(SW);
  delay(50);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set charger 1 amps
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>45) { counter=0;
     }
     setConeAmps=counter;
   } }

if (setpos==2){
 // set charger 2 amps
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>45) { counter=0;
     }
     setCtwoAmps=counter;
   }  }  

   if (setpos==3){
 // set batterytype
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>2) { counter=0;
     }
     settypeofbatt=counter;
   } }

        if (setpos==4){
 // if readout of power shall be inverted
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>1) { counter=0;
     }
     setshowpower=counter;
   } }
   

     if (setpos==5){
 // if readout of power shall be inverted
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>1) { counter=0;
     }
     setinvertreadout=counter;
   } }

   
     if (setpos==6){
 // if readout of power shall be inverted
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>30) { counter=0;
     }
     setinvcurrsens=counter;
   } }

     if (setpos==7){
 // if readout of power shall be inverted
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter>1) { counter=0;
     }
     setwifiavail=counter;
   } }

  if (setpos==8){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 11: Set Param. ");
   lcd.setCursor(7,2);
   lcd.print("DONE");
   // The following lines set parameters

   ConeAmps=setConeAmps;
   CtwoAmps=setCtwoAmps;
   typeofbatt=settypeofbatt;
   invertreadout=setinvertreadout;
   showpower=setshowpower;
   invcurrsens=setinvcurrsens;
   wifiavail=setwifiavail;
   //write limits to EEPROM
   EEPROM.write(4,typeofbatt);
   EEPROM.write(5,ConeAmps);
   EEPROM.write(6,CtwoAmps);
   EEPROM.write(21,showpower);
   EEPROM.write(22,invertreadout);
   EEPROM.write(29,invcurrsens);
   EEPROM.write(30,wifiavail);
   
   delay(4000);
   setpos=0;
   }
   previousStateCLK = currentStateCLK;  

}

void Mode12(){
  
  // Set Clock Mode (Mode12)

  
  lcd.setCursor(0,0);
  lcd.print("Mode 12: Set Clock   ");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("Date: ");
  lcd.setCursor(6,2);
  lcd.print("  ");
  lcd.setCursor(6,2);
  lcd.print(setday);
  lcd.setCursor(8,2);
  if (setpos==1){
  lcd.print("<");  
  }
  else {
  lcd.print(".");
  }
  lcd.setCursor(9,2);
  lcd.print("  ");
  lcd.setCursor(9,2);
  lcd.print(setmonth);
  lcd.setCursor(11,2);
  if (setpos==2){
  lcd.print("<");  
  }
  else {
  lcd.print(".");
  }
  lcd.setCursor(12,2);
  lcd.print("        ");
  lcd.setCursor(12,2);
  lcd.print(setyear);
  if (setpos==3){
  lcd.setCursor(16,2);  
  lcd.print("<");  
  }
  else {
  lcd.print(" ");  
  }
  lcd.setCursor(0,3);
  lcd.print("Time: ");
  lcd.setCursor(6,3);
  lcd.print("  ");
  lcd.setCursor(6,3);
  lcd.print(sethour);
  lcd.setCursor(8,3);
  if (setpos==4){
  lcd.print("<");  
  }
  else {
  lcd.print(".");
  }
  lcd.setCursor(9,3);
  lcd.print("  ");
  lcd.setCursor(9,3);
  lcd.print(setminute);
  lcd.setCursor(11,3);
  if (setpos==5){
  lcd.print("<");  
  }
  else {
  lcd.print(".");
  }
  lcd.setCursor(12,3);
  lcd.print("00      ");

  switchpress=digitalRead(SW);
  delay(50);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set day
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
      if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
        counter ++;
       }
     if (counter>31) { counter=1;
     }
     setday=counter;
   } }

if (setpos==2){
 // set month
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>12) { counter=1;
     }
     setmonth=counter;
   }  }  

   if (setpos==3){
 // set year
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     
     if (counter<2020 or counter>2031) { counter=2020;
     }
     setyear=counter;
   } }

   if (setpos==4){
   // set hour
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>23) { counter=0;
     }
     sethour=counter;
   } }

   if (setpos==5){
 // set minute
   currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != previousStateCLK){ 
       
     if (digitalRead(DT) != currentStateCLK) { 
       counter ++;
       } 
       else {
       counter ++;
       }
     if (counter>59) { counter=0;
     }
     setminute=counter;
   } }

   if (setpos==6){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 12: Set Time   ");
   lcd.setCursor(7,2);
   lcd.print("DONE");
  clock.setDateTime(setyear,setmonth,setday,sethour,setminute, 0);     // Set the time to  (24hr format)
    
   delay(4000);
   setpos=0;
   }
 // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK;  

}


void LEDsignals () {

  chargercounter++;
  invertercounter++;
  greencounter++;
  
  if (mode<=8) {
  digitalWrite(greenpin,HIGH);               // system showing ON only when in a running mode
  }
  else {
  digitalWrite(greenpin,LOW);
  }

  
  
  if (chargerone==1 and chargertwo==1) {
   digitalWrite(yellowpin,HIGH); 
   chargercounter=0;
  }

  if (chargerone==1 and chargertwo==0) {     //1 sec blink when only C1
   if (chargercounter==1) {
   digitalWrite(yellowpin,HIGH); 
   }
   else {
   digitalWrite(yellowpin,LOW);
   chargercounter=0; 
   }
  }
  
if (chargerone==0 and chargertwo==1) {     //2 sec blink when only C2   
   if (chargercounter==1 or chargercounter==2) {
   digitalWrite(yellowpin,HIGH); 
   }
   else {
   digitalWrite(yellowpin,LOW);
   chargercounter=0; 
   }
  }

  if (chargerone==0 and chargertwo==0) {
   digitalWrite(yellowpin,LOW); 
   chargercounter=0;
  }

  if (inverter==1) {     //1 sec blink when discharging
   if (invertercounter==1) {
   digitalWrite(redpin,HIGH); 
   }
   else {
   digitalWrite(redpin,LOW);
   invertercounter=0; 
   }
  }

   if (inverter==0) {
   digitalWrite(redpin,LOW); 
   invertercounter=0;
  }
}

void Datalogging(){

if (chargerone==1){
chargeronelog=holdvolt*ConeAmps;
}
else{
chargeronelog=0;  
}
if (chargertwo==1){
chargertwolog=holdvolt*CtwoAmps;
}
else{
chargertwolog=0;  
}

bothchargerslog=chargeronelog+chargertwolog;

if (inverter==1){
inverterlog=((invcurrsens*diffCurrent*230)/65536);      // calculate inverter watts
}
else{
inverterlog=0;  
}

  // SDcard write
if (SDavail==1){
  ACPWdata=SD.open("ACPWdata.txt",FILE_WRITE);
if (ACPWdata){
  ACPWdata.print(dt.hour);
  ACPWdata.print(":");
  ACPWdata.print(dt.minute);
  ACPWdata.print(":");
  ACPWdata.print(dt.second);
  ACPWdata.print(",");
  ACPWdata.print(dt.day);
  ACPWdata.print(",");
  ACPWdata.print(holdvolt);
  ACPWdata.print(",");
  ACPWdata.print(battSOC);
  ACPWdata.print(",");
  ACPWdata.print(realpower);
  ACPWdata.print(",");
  ACPWdata.print(averagepowerpool);
  ACPWdata.print(",");
  ACPWdata.print(chargeronelog);
  ACPWdata.print(",");
  ACPWdata.print(chargertwolog);
  ACPWdata.print(",");
  ACPWdata.print(bothchargerslog);
  ACPWdata.print(",");
  ACPWdata.print(inverterlog);
  ACPWdata.print(",");
  ACPWdata.print(tempC);
  ACPWdata.print(",");
  ACPWdata.print(millis()/10000);
  ACPWdata.println("");
  ACPWdata.close();
}
}

}


void BlynkDataTransmit(){

 if (wifidis==0){

 if (Blynk.connected()){
 
 Blynk.run();
   
 Blynk.virtualWrite(V5, millis() / 10000);
 Blynk.virtualWrite(V4, mode);
 Blynk.virtualWrite(V6, holdvolt);
 Blynk.virtualWrite(V7, battSOC);
 Blynk.virtualWrite(V8, round(averagepowerpool));
 Blynk.virtualWrite(V9, round(measuredpower));
 Blynk.virtualWrite(V10, tempC);
 if (chargerone==1){
 Blynk.virtualWrite(V1,HIGH);
 }
 else {
 Blynk.virtualWrite(V1,LOW); 
 }
 if (chargertwo==1){
 Blynk.virtualWrite(V2,HIGH);
 }
 else {
 Blynk.virtualWrite(V2,LOW); 
 }
 if (inverter==1){
 Blynk.virtualWrite(V3,HIGH);
 }
 else {
 Blynk.virtualWrite(V3,LOW); 
 }

 }
 else {
 Blynk.connect();
 }
 
 }
}

void Currentaquisition (){

if (inverter==1){

readCurrent=ads.readADC_SingleEnded(1);  

if (readCurrent<minCurrent){
  minCurrent=readCurrent;
}
if (readCurrent>maxCurrent){
  maxCurrent=readCurrent;
}

diffCurrent=maxCurrent-minCurrent;
diffCurrent=diffCurrent*2.9;

if (diffCurrent<1){
  diffCurrent=1;
}

 
}

}


void CheckFan(){

   humidity=HT.readHumidity();
   tempC=HT.readTemperature();
   
 if (tempC>=fanActtemp){
  fancounter=100;
 }   

  if (fancounter>0) {
  digitalWrite(relayPin4,LOW);
  fancounter--;
  }
  else {
  digitalWrite(relayPin4,HIGH);
 }



}

void getVoltage(){

 voltread=ads.readADC_SingleEnded(0);  
 // voltread=analogRead(voltPin);
 holdvolt=(voltread*(calibrationvalue+fineadjustment));   //calibrate your readout with a volt meter!
 averagevolt=averagevolt+holdvolt;           //smoothing out voltage readings
 if (voltcounter>=5) {
   battvoltage=averagevolt/5;
   averagevolt=0;
   voltcounter=0;  
 }
 voltcounter++;
  
}



void getSOC() {
  
SOCvoltage=battvoltage;
corrindchg=0;
corrindinv=0;
corrindrest=0;

if (typeofbatt==1){

   
  if (battvoltage>=54.9){
  if (chargerone==1) {         //correcting by charger voltage 
  SOCvoltage=SOCvoltage-(ConeAmps*0.060);
  corrindchg=7;
  }
  if (chargertwo==1) {         //correcting by charger voltage
  SOCvoltage=SOCvoltage-(CtwoAmps*0.060);
  corrindchg=7;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.058*0.19);  
  }
  }
  
  if (battvoltage>=54.70 and battvoltage<54.9){
  if (chargerone==1) {         
  SOCvoltage=SOCvoltage-(ConeAmps*0.048);
  corrindchg=6;
  }
  if (chargertwo==1) {         
  SOCvoltage=SOCvoltage-(CtwoAmps*0.048);
  corrindchg=6;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.048*0.20);  
  }
  }

   if (battvoltage>=54.4 and battvoltage<54.70){
  if (chargerone==1) {        
  SOCvoltage=SOCvoltage-(ConeAmps*0.037);
  corrindchg=5;
  }
  if (chargertwo==1) {       
  SOCvoltage=SOCvoltage-(CtwoAmps*0.037);
  corrindchg=5;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.039*0.21);  
  }
  }
  
  if (battvoltage>=54.05 and battvoltage<54.4){
  if (chargerone==1) {       
  SOCvoltage=SOCvoltage-(ConeAmps*0.030);
  corrindchg=4;
  }
  if (chargertwo==1) {      
  SOCvoltage=SOCvoltage-(CtwoAmps*0.030);
  corrindchg=4;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.030*0.21);  
  }
  }

  if (battvoltage>=53.6 and battvoltage<54.05){
  if (chargerone==1) {     
  SOCvoltage=SOCvoltage-(ConeAmps*0.024);
  corrindchg=3;
  }
  if (chargertwo==1) {     
  SOCvoltage=SOCvoltage-(CtwoAmps*0.024);
  corrindchg=3;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.024*0.22);  
  }
  }
  
  if (battvoltage>=53.1 and battvoltage<53.6){
  if (chargerone==1) {    
  SOCvoltage=SOCvoltage-(ConeAmps*0.021);
  corrindchg=2;
  }
  if (chargertwo==1) {     
  SOCvoltage=SOCvoltage-(CtwoAmps*0.021);
  corrindchg=2;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.021*0.23);  
  }
  }
  
  if (battvoltage<53.1) {
  if (chargerone==1) {      
  SOCvoltage=SOCvoltage-(ConeAmps*0.017);
  corrindchg=1;
  }
  if (chargertwo==1) {     
  SOCvoltage=SOCvoltage-(CtwoAmps*0.017);
  corrindchg=1;
  }
  if (chargerone==1 and chargertwo==1){
   SOCvoltage=SOCvoltage+((ConeAmps+CtwoAmps) *0.017*0.24);  
  }
  }
  
}
else {
  if (chargerone==1) {      
  SOCvoltage=SOCvoltage-(ConeAmps*0.02);
  corrindchg=2;
  }
  if (chargertwo==1) {      
  SOCvoltage=SOCvoltage-(CtwoAmps*0.02);
  corrindchg=2;
  }
}

if (inverter==1) {         //correcting by inverter voltage by Current module
 
  SOCvoltage=SOCvoltage+0.28;    // basic corr for going into discharge
  SOCvoltage=SOCvoltage+(diffCurrent*0.0000101*(invcurrsens/5));
  tempdiffCurrent=diffCurrent*(invcurrsens/8.7);
  if (tempdiffCurrent>=60000){
    corrindinv=7;
    }
  if (tempdiffCurrent>=50000 and tempdiffCurrent<60000){
    corrindinv=6;
    } 
   if (tempdiffCurrent>=40000 and tempdiffCurrent<50000){
    corrindinv=5;
    } 
   if (tempdiffCurrent>=30000 and tempdiffCurrent<40000){
    corrindinv=4;
    } 
   if (tempdiffCurrent>=20000 and tempdiffCurrent<30000){
    corrindinv=3;
    } 
   if (tempdiffCurrent>=10000 and tempdiffCurrent<20000){
    corrindinv=2;
    }  
   if (tempdiffCurrent<10000){
    corrindinv=1;
    }   
}

if (SOCfreeze==0) {                    // after load change do not check SOC for around 2 min
  
  
battSOC=100;

if (typeofbatt==0) {                   //LIPO
if (SOCvoltage<=57.7) {
battSOC=95;
}
if (SOCvoltage<=56.5) {
battSOC=90;
}
if (SOCvoltage<=55.6) {
battSOC=85;
}
if (SOCvoltage<=54.7) {
battSOC=80;
}
if (SOCvoltage<=53.8) {
battSOC=75;
}
if (SOCvoltage<=52.9) {
battSOC=70;
}  
if (SOCvoltage<=52.5) {
battSOC=65;
}
if (SOCvoltage<=52.0) {
battSOC=60;
}
if (SOCvoltage<=51.7) {
battSOC=55;
}
if (SOCvoltage<=51.4) {
battSOC=50;
}
if (SOCvoltage<=51.0) {
battSOC=45;
}
if (SOCvoltage<=50.7) {
battSOC=40;
}  
if (SOCvoltage<=50.5) {
battSOC=35;
}
if (SOCvoltage<=50.2) {
battSOC=30;
}
if (SOCvoltage<=49.9) {
battSOC=25;
}
if (SOCvoltage<=49.5) {
battSOC=20;
}
if (SOCvoltage<=48.8) {
battSOC=15;
}
if (SOCvoltage<=48.2) {
battSOC=10;
}  
if (SOCvoltage<=46.7) {
battSOC=5;
}  
if (SOCvoltage<=44.0) {
battSOC=0;
}  
}

if (typeofbatt==1) {                     // LIFEPO

if (idlestatus==1) {                           // correcting voltage sag when batt idling for longer
  SOCvoltage=SOCvoltage+0.15;
  corrindrest=1;
}

if (SOCvoltage<=54.05) {
battSOC=99;
}  
if (SOCvoltage<=53.93) {
battSOC=98;
}  
if (SOCvoltage<=53.83) {
battSOC=97;
}  
if (SOCvoltage<=53.78) {
battSOC=96;
}  
if (SOCvoltage<=53.74) {
battSOC=95;
}
if (SOCvoltage<=53.70) {
battSOC=94;
}  
if (SOCvoltage<=53.67) {
battSOC=93;
}
if (SOCvoltage<=53.66) {
battSOC=92;
} 
if (SOCvoltage<=53.65) {
battSOC=91;
}   
if (SOCvoltage<=53.64) {
battSOC=90;
}
if (SOCvoltage<=53.63) {
battSOC=88;
}  
if (SOCvoltage<=53.62) {
battSOC=85;
}
if (SOCvoltage<=53.60) {
battSOC=83;
}  
if (SOCvoltage<=53.57) {
battSOC=80;
}
if (SOCvoltage<=53.52) {
battSOC=78;
}  
if (SOCvoltage<=53.45) {
battSOC=75;
}
if (SOCvoltage<=53.38) {
battSOC=73;
}  
if (SOCvoltage<=53.27) {
battSOC=70;
}
if (SOCvoltage<=53.23) {
battSOC=68;
}  
if (SOCvoltage<=53.19) {
battSOC=67;
}    
if (SOCvoltage<=53.15) {
battSOC=65;
}
if (SOCvoltage<=53.14) {
battSOC=63;
}  
if (SOCvoltage<=53.13) {
battSOC=62;
}  
if (SOCvoltage<=53.12) {
battSOC=60;
}
if (SOCvoltage<=53.10) {
battSOC=58;
}  
if (SOCvoltage<=53.09) {
battSOC=57;
}  
if (SOCvoltage<=53.08) {
battSOC=55;
}
if (SOCvoltage<=53.04) {
battSOC=53;
}  
if (SOCvoltage<=53.01) {
battSOC=52;
}  
if (SOCvoltage<=52.98) {
battSOC=50;
}
if (SOCvoltage<=52.97) {
battSOC=48;
}  
if (SOCvoltage<=52.96) {
battSOC=47;
}  
if (SOCvoltage<=52.94) {
battSOC=45;
}
if (SOCvoltage<=52.93) {
battSOC=43;
}  
if (SOCvoltage<=52.92) {
battSOC=42;
}  
if (SOCvoltage<=52.90) {
battSOC=40;
}  
if (SOCvoltage<=52.88) {
battSOC=35;
}
if (SOCvoltage<=52.83) {
battSOC=33;
}  
if (SOCvoltage<=52.80) {
battSOC=32;
}  
if (SOCvoltage<=52.76) {
battSOC=30;
}
if (SOCvoltage<=52.69) {
battSOC=28;
}  
if (SOCvoltage<=52.65) {
battSOC=27;
}  
if (SOCvoltage<=52.57) {
battSOC=25;
}
if (SOCvoltage<=52.48) {
battSOC=23;
}  
if (SOCvoltage<=52.40) {
battSOC=22;
}  
if (SOCvoltage<=52.26) {
battSOC=20;
}
if (SOCvoltage<=52.18) {
battSOC=19;
}  
if (SOCvoltage<=52.09) {
battSOC=18;
}  
if (SOCvoltage<=52.00) {
battSOC=17;
}  
if (SOCvoltage<=51.93) {
battSOC=16;
}  
if (SOCvoltage<=51.84) {
battSOC=15;
}
if (SOCvoltage<=51.80) {
battSOC=14;
}  
if (SOCvoltage<=51.75) {
battSOC=13;
}  
if (SOCvoltage<=51.69) {
battSOC=12;
}  
if (SOCvoltage<=51.65) {
battSOC=11;
}  
if (SOCvoltage<=51.60) {
battSOC=10;
}  
if (SOCvoltage<=51.51) {
battSOC=9;
}  
if (SOCvoltage<=51.40) {
battSOC=8;
}  
if (SOCvoltage<=51.30) {
battSOC=7;
}  
if (SOCvoltage<=51.15) {
battSOC=6;
} 
if (SOCvoltage<=50.93) {
battSOC=5;
}  
if (SOCvoltage<=50.63) {
battSOC=4;
}   
if (SOCvoltage<=50.23) {
battSOC=3;
}  
if (SOCvoltage<=49.00) {
battSOC=2;
}  
if (SOCvoltage<=45.00) {
battSOC=0;
}  

// corr for when high SOC when charging
if (corrindchg==4 and battSOC<55){
  battSOC=55;
}
if (corrindchg==5 and battSOC<73){
  battSOC=73;
}
if (corrindchg==6 and battSOC<80){
  battSOC=80;
}
if (corrindchg==7 and battSOC<85){
  battSOC=85;
}

}

if (typeofbatt==2) {                       // NMC
if (SOCvoltage<=57.1) {
battSOC=98;
}  
if (SOCvoltage<=56.2) {
battSOC=95;
}
if (SOCvoltage<=55.9) {
battSOC=93;
}  
if (SOCvoltage<=55.4) {
battSOC=90;
}
if (SOCvoltage<=55) {
battSOC=87;
}  
if (SOCvoltage<=54.7) {
battSOC=85;
}
if (SOCvoltage<=54.4) {
battSOC=83;
}  
if (SOCvoltage<=53.9) {
battSOC=80;
}
if (SOCvoltage<=53.4) {
battSOC=75;
}
if (SOCvoltage<=52.8) {
battSOC=70;
}  
if (SOCvoltage<=52.5) {
battSOC=65;
}
if (SOCvoltage<=51.8) {
battSOC=60;
}
if (SOCvoltage<=51.5) {
battSOC=55;
}
if (SOCvoltage<=51.2) {
battSOC=50;
}
if (SOCvoltage<=51) {
battSOC=45;
}
if (SOCvoltage<=50.7) {
battSOC=40;
}  
if (SOCvoltage<=50.4) {
battSOC=35;
}
if (SOCvoltage<=50) {
battSOC=30;
}
if (SOCvoltage<=49.7) {
battSOC=25;
}
if (SOCvoltage<=49.3) {
battSOC=20;
}
if (SOCvoltage<=48.8) {
battSOC=15;
}
if (SOCvoltage<=48.2) {
battSOC=10;
}  
if (SOCvoltage<=46.1) {
battSOC=5;
}  
if (SOCvoltage<=45) {
battSOC=3;
}  
if (SOCvoltage<=44.0) {
battSOC=0;
}  
}

}

}


void resetFunc(){
asm volatile ("  jmp 0");
}


uint16_t calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t temp, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    temp = temp ^ array[i];
    for (uint8_t j = 8; j; j--) {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  return temp;
}

// Constructor
modbusSensor::modbusSensor(modbusMaster * mbm, uint8_t id, uint16_t adr, uint8_t hold) {
  _frame[0] = id;
  _frame[1] = READ_INPUT_REGISTERS;
  _frame[2] = adr >> 8;
  _frame[3] = adr & 0x00FF;
  _frame[4] = 0x00;
  _frame[5] = 0x02;
  uint16_t crc = calculateCRC(_frame, 6);
  _frame[6] = crc & 0x00FF;
  _frame[7] = crc >> 8;
  _status = MB_TIMEOUT;
  _hold = hold;
  _value.f = 0.0;
  (*mbm).connect(this);
}

// read value in defined units
float modbusSensor::read() {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO: return 0.0;
      case CHANGE_TO_ONE: return 1.0;
      case HOLD_VALUE: return _value.f;
    }
  return _value.f;
}

// read value as a integer multiplied by factor
uint16_t modbusSensor::read(uint16_t factor) {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO: return (uint16_t) 0;
      case CHANGE_TO_ONE: return (uint16_t) factor;
      case HOLD_VALUE: return (uint16_t)(_value.f * factor);
    }
  return (uint16_t)(_value.f * factor);
}
// get status of the value
inline uint8_t modbusSensor::getStatus() {
  return _status;
}

// write sensor value
inline void modbusSensor::write(float value) {
  _value.f = value;
}

//  put new status
inline uint8_t modbusSensor::putStatus(uint8_t status) {
  _status = status;
  return _status;
}

// get pointer to _poll frame
inline uint8_t *modbusSensor::getFramePtr() {
  return _frame;
}

//---------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------//

//constructor
modbusMaster::modbusMaster(HardwareSerial * hwSerial, uint8_t TxEnPin) {
  _state = STOP;
  _TxEnablePin = TxEnPin;
  pinMode(_TxEnablePin, OUTPUT);
  _MBSerial = hwSerial;
  _totalSensors = 0;
  for (uint8_t i = 0; i < MAX_SENSORS; i++)
    _mbSensorsPtr[i] = 0;
}


// Connect a modbusSensor to the modbusMaster array of queries
void modbusMaster::connect(modbusSensor * mbSensor) {
  if (_totalSensors < MAX_SENSORS) {
    _mbSensorsPtr[_totalSensors] = mbSensor;
    _totalSensors++;
  }
  return;
}

//------------------------------------------------------------------------------
// Disconnect a modbusSensor to the modbusMaster array of queries
void modbusMaster::disconnect(modbusSensor * mbSensor) {
  uint8_t i, j;
  for (i = 0;   i < _totalSensors; i++) {
    if (_mbSensorsPtr[i] == mbSensor)  {
      for (j = i; j < _totalSensors - 1; j++) {
        _mbSensorsPtr[j] = _mbSensorsPtr[j + 1];
      }
      _totalSensors--;
      _mbSensorsPtr[_totalSensors] = 0;
    }
  }
}

//------------------------------------------------------------------------------
// begin communication using ModBus protocol over RS485
void modbusMaster::begin(uint16_t baudrate, uint8_t byteFormat, uint16_t pollInterval) {
  _pollInterval = pollInterval - 1;
  if (baudrate > 19200)
    _T2_5 = 1250;
  //_T3_5 = 1750; _T1_5 = 750;
  else
    _T2_5 = 27500000 / baudrate; // 2400 bauds --> 11458 us; 9600 bauds --> 2864 us

  (*_MBSerial).begin(baudrate, byteFormat);
  _state = SEND;
  digitalWrite(_TxEnablePin, LOW);
}

//------------------------------------------------------------------------------
// end communication over serial port
inline void modbusMaster::end() {
  _state = STOP;
  (*_MBSerial).end();
}

// Finite State Machine core,
boolean modbusMaster::available() {
  static uint8_t  indexSensor = 0;                // index of arrray of sensors
  static uint8_t  frameSize;                      // size of the answer frame
  static uint32_t tMicros;                        // time to check between characters in a frame
  static uint32_t nowMillis = millis();
  static uint32_t lastPollMillis = nowMillis;     // time to check poll interval
  static uint32_t sendMillis = nowMillis;         // time to check timeout interval
  static uint32_t receiveMillis = nowMillis;      // time to check waiting interval

  switch (_state) {
    //-----------------------------------------------------------------------------
    case SEND:

      if (indexSensor < _totalSensors) {
        _mbSensorPtr = _mbSensorsPtr[indexSensor];
        _framePtr = (*_mbSensorPtr).getFramePtr();
        digitalWrite(_TxEnablePin, HIGH);
        sendFrame();

        _state = SENDING;
        return false;
      }
      else {
        indexSensor = 0;
        _state = WAITING_NEXT_POLL;
        return true;
      }

    //-----------------------------------------------------------------------------
    case SENDING:

      if ((*_MBSerial).availableForWrite() == SERIAL_TX_BUFFER_SIZE - 1) { //TX buffer empty
        delayMicroseconds(_T2_5); // time to be sure last byte sended
        while ((*_MBSerial).available()) (*_MBSerial).read(); // clean RX buffer
        digitalWrite(_TxEnablePin, LOW);
        sendMillis = millis(); //starts  slave's timeOut
        _state = RECEIVING;
        frameSize = 0;
      }
      return false;

    //-----------------------------------------------------------------------------
    case RECEIVING:

      if (!(*_MBSerial).available()) {
        if (millis() - sendMillis > TIMEOUT) {
          (*_mbSensorPtr).putStatus(MB_TIMEOUT);
          indexSensor++;
          _state = SEND;
        }
        return false;
      }

      if ((*_MBSerial).available() > frameSize) {
        frameSize++;
        tMicros = micros();
      }
      else {
        if (micros() - tMicros > _T2_5) {
          readBuffer(frameSize);
          MODBUS_SERIAL_PRINTLN((*_mbSensorPtr).getStatus(), HEX);
          indexSensor++;
          receiveMillis = millis(); //starts waiting interval to next request
          _state = IDLE;
        }
      }
      return false;

    //-----------------------------------------------------------------------------
    case IDLE:
      if (millis() - receiveMillis > WAITING_INTERVAL)
        _state = SEND;
      return false;

    //-----------------------------------------------------------------------------
    case WAITING_NEXT_POLL:
      nowMillis = millis();
      if ((nowMillis - lastPollMillis) > _pollInterval) {
        lastPollMillis = nowMillis;
        _state = SEND;
      }
      return false;

    //-----------------------------------------------------------------------------
    case STOP:   // do nothing

      return false;

  }
}

//-----------------------------------------------------------------------------
inline void modbusMaster::readBuffer(uint8_t frameSize) {
  uint8_t index = 0;
    //  boolean ovfFlag = false;
 MODBUS_SERIAL_PRINT(millis());
  MODBUS_SERIAL_PRINT("  SLAVE:");
  for (index = 0; index < frameSize; index++) {
    _buffer[index] = (*_MBSerial).read();
#ifdef MODBUS_SERIAL_OUTPUT
   if (_buffer[index] < 0x10)
   Serial.print(F(" 0"));
    else
    Serial.print(F(" "));
    Serial.print(_buffer[index], HEX);
#endif
  }
 MODBUS_SERIAL_PRINT(" ");
 MODBUS_SERIAL_PRINTLN(millis());

  // The minimum buffer size from a slave can be an exception response of 5 bytes.
  // If the buffer was partially filled set a frame_error.
  if (frameSize < 5) {
    (*_mbSensorPtr).putStatus(MB_SLAVE_FAIL);
    return;
  }

  if (_buffer[0] != _framePtr[0]) {
    (*_mbSensorPtr).putStatus(MB_INVALID_ID);
    return;
  }

  uint16_t crc = calculateCRC(_buffer, index - 2);
  if (_buffer[frameSize - 1] != crc >> 8 && _buffer[frameSize - 2] != crc & 0x00FF) {
    (*_mbSensorPtr).putStatus(MB_INVALID_CRC);
    return;
  }
  if (_buffer[1] & 0x80 == 0x80) {
    (*_mbSensorPtr).putStatus(_buffer[2]); // see exception codes in define area
    return;
  }

  if (_buffer[1] != _framePtr[1]) {
    (*_mbSensorPtr).putStatus(MB_INVALID_FC);
    return;
  }

  switch (_buffer[1]) {
    case READ_INPUT_REGISTERS:
      if (_buffer[2] == 4) {
        dataFloat temp;
        temp.array[3] = _buffer[3];
        temp.array[2] = _buffer[4];
        temp.array[1] = _buffer[5];
        temp.array[0] = _buffer[6];
       MODBUS_SERIAL_PRINTLN(temp.f);
        holdpower=String(temp.f);  // saves value as string to enable negative values
        (*_mbSensorPtr).write(temp.f);
        (*_mbSensorPtr).putStatus(MB_VALID_DATA);
        return;
      }
      else {
        (*_mbSensorPtr).putStatus(MB_ILLEGAL_DATA);
        return;
      }
    default:
      (*_mbSensorPtr).putStatus(MB_INVALID_FC);
      return;
  }
}

//-----------------------------------------------------------------------------
//
inline void modbusMaster::sendFrame() {

  (*_MBSerial).write(_framePtr, 8);


}
