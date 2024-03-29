
// This is final-software for my AC-powerwall controller with Version 3 board
// Version 1.04.02, 10.02.2021
// Author and system designer: Roland Wukovits
// e-mail: acpw@thehillside.net
// The code, or parts of it, can be used for private DIY projects only, as
// parts of the code (especially libraries and standard routines) 
// are originated from other authors
// Code for the whole controller, is intellectual property of the 
// system designer. Please contact me for special approval if you are
// planning to use the code in a commecial application!
// If you need customized features, you can contact me for assistance,
// as well I can source components and build a controller board (anywhwere)
// (or even a whole powerwall (Thailand)) for you 
// If I was able to help you out with valueable information to 
// realize your project, please consider an adequate donation by using
// PayPal: sales@thehillside.net
// Each donation, even small ones, are very much appreciated. Thanks!


// I2C
#include <Wire.h>
// NewLiquidCrystal Library for I2C
#include <LiquidCrystal_I2C.h>
// DS3231 RTC
#include <DS3231.h>
// DHT11 Libraries
#include <Adafruit_Sensor.h>
#include <DHT.h>
// Modbus for SDM-Meter Library
#include "ModbusSensor.h"
// EEPROM
#include <EEPROM.h>
// ADS1115 Library
#include <Adafruit_ADS1015.h>
// SD module
#include <SD.h>
#include <SPI.h>
// ESP01 and Blynk for Mega
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
// OLED display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// INA226 shunt sensor
#include <INA226_WE.h>
// 16Ch Multiplexer
#include <CD74HC4067.h>

#define BLYNK_PRINT Serial
char auth[] = "Your Auth";
char ssid[] = "yourSSID";
char pass[] = "yourPW";

WidgetLED led1(V15);
WidgetLED led2(V1);
WidgetLED led3(V2);
WidgetLED led4(V3);
WidgetLED led5(V0);
WidgetLED led6(V23);
// WidgetLED led7(V24);

// Hardware Serial on Mega
#define EspSerial Serial2

// or Software Serial on Uno, Nano...
//#include <SoftwareSerial.h>
//SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);

//INA226
#define I2C_ADDRESS 0x40
//INA226_WE ina226(I2C_ADDRESS);
INA226_WE ina226 = INA226_WE(I2C_ADDRESS); 

//OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
int MB_BAUDRATE;         // baud 2400 for SDM120, 9600 for X835
#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       41
#define ID_1  1                       // id 001  modbus id of the energy monitor
#define REFRESH_INTERVAL  5000        // refresh time, 5 SECONDS
//#define POW_ADR 0x0010    // measured power X835/phase3  (0x000C for SDM120 or X835/phase1, 0x000E for X835/phase2

modbusMaster MBserial(MB_SERIAL_PORT, TxEnablePin);  // instance to collect data using Modbus protocol over RS485
modbusSensor pwr1(&MBserial, ID_1, 0x000C, CHANGE_TO_ZERO);
modbusSensor blank1(&MBserial, ID_1, 0x0000, CHANGE_TO_ZERO);    //dummy read
modbusSensor pwr2(&MBserial, ID_1, 0x000E, CHANGE_TO_ZERO);
modbusSensor blank2(&MBserial, ID_1, 0x0006, CHANGE_TO_ZERO);    //dummy read
modbusSensor pwr3(&MBserial, ID_1, 0x0010, CHANGE_TO_ZERO);

byte typeofmeter=0;            //0=SDM120, 1=SDM630, 2=X835, 9=None
byte settypeofmeter;
float measuredpower;
float measuredpower1;
float measuredpower2;
float measuredpower3;
float oldmeasuredpower1;
float oldmeasuredpower2;
float oldmeasuredpower3;
float measuredpowerS;
float measuredpowerT;
float blankpower;
float averagepower=0;
float averagepowerpool=0;
float averagepower1=0;
float averagepowerpool1=0;
float averagepower2=0;
float averagepowerpool2=0;
float averagepower3=0;
float averagepowerpool3=0;
float averagepowerS=0;
float averagepowerpoolS=0;
float averagepowerpoolT=0;
float lastaveragepower=0;
byte activephase=1;         // calculations based on this phase measurement
byte setactivephase;
int powercounter=1;
int triggerpowermode1=40;  //  minimum power available for mode 1 / 10 (avoid import)
int triggerpowermode2=20;  //  minimum power available for mode 2 / 10 (allow import)
int settriggerpowermode1;
int settriggerpowermode2;
float realpower=0;
float realpowerpri=0;
float realpowerthree=0;
byte chargerone=0;
byte chargertwo=0;
byte chargersnbr;
byte chargerthree=0;
byte inverter=0;
byte invertreadout=0;
byte setinvertreadout;
byte trueread;
byte dyntrigger=0;
byte setdyntrigger;
float minimumpower;
float maximumpower;
float diffpower;
byte dynlevel=0;
 
// Rotary Encoder Inputs
 #define CLK 2
 #define DT 3
 #define SW 45
 int lastCount=50;
 byte encoderblock;
 volatile int virtualPosition=50;
 volatile bool pinAStateHigh;
 volatile bool pinBStateHigh;
 bool oldPinAStateHigh;
 bool oldPinBStateHigh;
 unsigned long lastencodermillis;
// Init the DS3231 using the hardware interface
DS3231 clock;
RTCDateTime dt;
// DHT11 sensor data
#define sensepin 4
DHT HT(sensepin,DHT11);
float humidity;
float tempC;
float fancounter;
// Relays module
#define relayPin1 34               // Charger 1
#define relayPin2 35               // Charger 2
#define relayPin3 36               // Inverter DC
#define relayPin4 37               // Fans
#define relayPin5 38               // Inverter Capacitor Precharging
#define relayPin6 39               // optional Charger 3
byte relayAct1=0;
byte relayAct2=0;
byte relayAct3=0;
byte relayAct4=0;
long relaycycletime=360000;    // set for the charger activation cycle time in milliseconds
int prechargedelay=3500;                // set for inverter capacitor precharging in milliseconds
int prechgreleasedelay=150;             // set for precharging relay release in milliseconds
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
byte mode=1; 
byte oldmode=1;  
unsigned long modestartmillis; 
unsigned long modeinterval=1500;                // checks for mode change every 1,5 sec  
unsigned long modestartdelaymillis;      
// Rotary switch
int counter=1; 
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
float oldbattvoltage;
float fineadjustment=0;
float fineadjustmentbyte=30;          
float setfineadjustmentbyte;          
byte battSOC;
byte OldbattSOC;
float battCapFact;
int SOCfreeze;
int averageSOCpool;
byte averageSOC;
byte minimumSOC=10;
byte setminimumSOC;
byte maximumSOC=90;
byte setmaximumSOC;
byte fanAct=0;
float setfanActtemp;                 
float fanActtemp=30;                 
unsigned long loopstartmillis;
unsigned long shortloopinterval=1400;         // start certain modes every ... milliseconds
unsigned long longloopinterval=2400;         // start certain modes every ... milliseconds
float holdvolt;
// Moving Average Voltage
const int VnumReadings=8;          // how many values are part of MA
float Vreadings[VnumReadings];      
int VreadIndex=0;            
float Vtotal;   
float SOCvoltage;
byte showpower=1;
byte setshowpower;
byte inverteravailable;
byte invcurrsens;
float inverterlimitW=1500;                // set max output limit from your SUN2000GTIL or other inverter
byte setinvcurrsens;
byte chargerlock;
byte Maxlock;
byte MinLock;
byte pagelock;
unsigned long cyclestartmillis;
// EEPROM
byte EEPROMaddress = 0;
byte EEPROMvalue[50];            // Array accomodating 45 stored values, increase if needed
//LED announciators
unsigned long ledinterval=1000;  // LED interval in milliseconds
unsigned long ledstartmillis;
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
byte CthreeAmps=0;
byte settypeofbatt;
byte setConeAmps;
byte setCtwoAmps;
byte setCthreeAmps;
byte Cthree=0;
byte setCthree;
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
byte invertertimeHr;
byte invertertimeMin;
int invertertimeinminutes;
int invertertimeONinminutes;
int invertertimeOFFinminutes;
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
byte NormInvHrOn=18;
byte NormInvHrOff=7;
byte NormInvMinOn=0;
byte NormInvMinOff=0;
byte setNormInvHrOn;
byte setNormInvHrOff;
byte setNormInvMinOn;
byte setNormInvMinOff;
byte setDreset;
byte setDresethr;
byte setDresetmin;
byte Dreset=0;
byte Dresethr=7;
byte Dresetmin=5;
unsigned long startmillis;
unsigned long diffmillis;
byte longinvcorr;
byte invcorract;
unsigned long idlestartmillis;
unsigned long idlediffmillis;
unsigned long minimumstartmillis;
byte minimumreach;
byte idlestatus;
byte idleactive;
byte Limiter;
byte setLimiter;
byte LimiterOps=1;
unsigned long Limiterstartmillis;
byte Limitertimeout;
byte Limitercounter;
byte Limiterfirst;
int Limiterbreakout=-500;           // set value of max. Import above which Limiter shall stop charging (M3)
// ADS1115
Adafruit_ADS1115 ads;
float ADSGain;
// SD module
File ACPWdata;
#define CS 53
byte SDavail;
float chargeronelog;
float chargertwolog;
float chargerthreelog;
float averageinvwatts;
float averageinvwattspool;
int invwattcounter;
float inverterwatts;
float oldinverterwatts;
float diffinverterwatts;
float diffbattvoltage;
float bothchargerslog;
// Current module
unsigned long currentstartmillis;
unsigned long currentinterval=10000;   // interval of current aquiration when inverting
int aquisitioncounter;
long maxCurrent=1;
long minCurrent=65535;
long diffCurrent;
long tempdiffCurrent;
long readCurrent;
// Blynk/Wifi
byte wifidis=0;
byte wifiavail=0;
byte setwifiavail;
byte bly;
byte linkState;
byte connectCounter;
byte OLEDstop;
unsigned long sendstartmillis;
unsigned long sendinterval=3500;         // interval of sending Blynk data
byte sendpart=0;
byte transmitcounter=0;
// Ext WDT
#define ExtWDTpin 8
byte ExtWDT=0;
byte setExtWDT;
unsigned long ExtWDTstartmillis;
unsigned long ExtWDTinterval=3000;         // interval of ext. WDT reset puls
int disconnects;
// Temp probes
#define Probe1 A7
#define Probe2 A9
#define Probe3 A11
int proberead1;
int proberead2;
int proberead3;
float probetemp1;
float probetemp2;
float probetemp3;
float sendprobetemp1;
float sendprobetemp2;
float sendprobetemp3;
float highestprobe;
byte subpage;

// Moving Average
const int numReadings=7;          // how many values are part of MA
int T1readings[numReadings];
int T2readings[numReadings];    
int T3readings[numReadings];          
int readIndex=0;              
int totalT1;                  // the running total for MA calc
int totalT2;
int totalT3;
float totalT1a;                  // the running total for MA calc
float totalT2a;
float totalT3a;
// Moving Average current
const int AnumReadings=7;          // how many values are part of MA
float Areadings[AnumReadings];      
int AreadIndex=0;              
float total;                 
byte homing=1;           
byte Dhoming;

const int numReadingsW=5;
float totalW;
int WreadIndex=0;
float Wreadings[numReadingsW];
float inverterwattsB;

float shuntVolt;
int shuntMaxmV=75;                       // enter your Shunt mV rating
int shuntMaxAmps=100;                    // enter your Shunt current rating in A
float shunttemp;
float current;
float holdcurrent;
float lastcurrent; 
float lastAhigh;
float lastAlow;
unsigned long Aloopstartmillis;
unsigned long Aloopinterval=650;         // interval of loop in mSec  
float watts;

#define Battswitch 15                   // enter your Switch Digital Pin
byte Battmode;

float Cwatts;
int wattscounter=0;
int dwattcounter=0;
float averagewatts=0;
float daveragewatts;
float chgwatthours=0;
float watthoursTot;
float daverageAmps;
float ActAh;
float AhoursTot;
float ActWh;
float oldActWh;
unsigned long wattsstartmillis;
unsigned long wattsinterval=180000;     //set 600000 to log Wh once every 10 min, 300000 for every 5 min, 60000 for every min 
unsigned long dwattsstartmillis;
unsigned int extractor;
byte cycleslobyte;
byte cycleshibyte;
byte battcapacityLobyte;
byte battcapacityHibyte;
byte capsave=0;
int cycles=250;                      //enter numbers of cycles, you want to flash on EEPROM first time
byte flashcyclesLobyte;
byte flashcyclesHibyte;
long battcapacity=19000;      //enter your battery capacity in Wh (max. 65300), you want to flash on EEPROM first time 
long Cbattcapacity;
byte flashbattcapacityLobyte;
byte flashbattcapacityHibyte;
int cwattscounter;
float CCwatts;
float Caveragewatts;
float CCwatthours;
float CDwatts;
float Daveragewatts;
float CDwatthours;
byte firstcharge;
byte firstdischarge;
byte coulstart=0;
byte SOCCoul;
byte SOCDisplay;
float CoulombPointer;
unsigned long coulombstartmillis;
float invwattsbyshunt;  
byte StartSOC;
byte EndSOC;
byte diffSOC;
byte Meteraccur=3;
float SOCdiff;
byte blockdischargetest;
byte coulcorrbyte;
byte setcoulcorrbyte;
byte coulcorrstart;
byte coulcorrend;
byte coulcorrarm=0;
float coulcorr;   
float coulcorrDW;  
float coulcorrCW; 
unsigned long couldelaymillis;                 
byte couldelay;
byte calistop;
byte Dtest;
int checkshunt=18800;   // temporary estimated battery capacity used for sensor check triggers
#define Vsense A15
int Vvalue;
byte Vpresent;
byte Vcounter;
// MUX
float CHvoltage[16];
float stringvoltage[16];
byte vc=0;
byte vi;
float lowestSV=0;
float highestSV=0;
float lowestSnbr=1;
float highestSnbr=1;
float delta;
byte vt;
CD74HC4067 my_mux(42, 43, 44, 46);  // s0 s1 s2 s3   create a new CD74HC4067 object with its four control pins
const int SnumReadings=5;          // how many values are part of MA
float S1readings[SnumReadings];
float S2readings[SnumReadings];
float S3readings[SnumReadings];
float S4readings[SnumReadings];
float S5readings[SnumReadings];
float S6readings[SnumReadings];
float S7readings[SnumReadings];
float S8readings[SnumReadings];
float S9readings[SnumReadings];
float S10readings[SnumReadings];
float S11readings[SnumReadings];
float S12readings[SnumReadings];
float S13readings[SnumReadings];
float S14readings[SnumReadings];
float S15readings[SnumReadings];
float S16readings[SnumReadings];
int SreadIndex=0;              
float totalSV[16];
byte Scounter=0;

void setup(){
  
  Serial.begin(9600);
  //check for EEPROM data and read
  for (EEPROMaddress=0;EEPROMaddress<50;EEPROMaddress++) {
    EEPROMvalue[EEPROMaddress]=EEPROM.read(EEPROMaddress);
    delay(50);
  }
  //firstwrite EEPROM
  if (EEPROMvalue[0]==255 or EEPROMvalue[0]==0) {

   extractor=cycles;
    while (extractor>256){
      flashcyclesHibyte++;
      extractor=extractor-256;
    }
    flashcyclesLobyte=extractor;

    if(battcapacity<300){              //flashes a minimum amount of capacity is case you forgot or declared wrong
      battcapacity=300;
    }

    extractor=battcapacity;
    while (extractor>256){
      flashbattcapacityHibyte++;
      extractor=extractor-256;
    }
    flashbattcapacityLobyte=extractor;
 
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
     EEPROM.write(24,NormInvHrOn);
     EEPROM.write(25,NormInvHrOff);
     EEPROM.write(26,Dreset);                      
     EEPROM.write(27,Dresethr);                  
     EEPROM.write(28,Dresetmin);
     EEPROM.write(29,invcurrsens);
     EEPROM.write(30,wifiavail);
     EEPROM.write(31,dyntrigger);
     EEPROM.write(32,activephase);               
     EEPROM.write(33,typeofmeter);
     EEPROM.write(34,NormInvMinOn);
     EEPROM.write(35,NormInvMinOff);
     EEPROM.write(36,coulcorrbyte);              
     EEPROM.write(37,Cthree);                  
     EEPROM.write(38,CthreeAmps);
     EEPROM.write(39,Limiter);
     EEPROM.write(40,ExtWDT);           
     EEPROM.write(41,flashcyclesLobyte);              
     EEPROM.write(42,flashcyclesHibyte); 
     EEPROM.write(43,flashbattcapacityLobyte); 
     EEPROM.write(44,flashbattcapacityHibyte);  
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
     NormInvHrOn=EEPROMvalue[24];
     setNormInvHrOn=NormInvHrOn;
     NormInvHrOff=EEPROMvalue[25];
     setNormInvHrOff=NormInvHrOff;
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
     dyntrigger=EEPROMvalue[31];
     setdyntrigger=dyntrigger;
     activephase=EEPROMvalue[32];
     setactivephase=activephase;
     typeofmeter=EEPROMvalue[33];
     settypeofmeter=typeofmeter;
     NormInvMinOn=EEPROMvalue[34];
     setNormInvMinOn=NormInvMinOn;
     NormInvMinOff=EEPROMvalue[35];
     setNormInvMinOff=NormInvMinOff;
     coulcorrbyte=EEPROMvalue[36];
     setcoulcorrbyte=coulcorrbyte;
     Cthree=EEPROMvalue[37];
     setCthree=Cthree;
     CthreeAmps=EEPROMvalue[38];
     setCthreeAmps=CthreeAmps; 
     Limiter=EEPROMvalue[39];
     setLimiter=Limiter; 
     ExtWDT=EEPROMvalue[40];
     setExtWDT=ExtWDT; 
     cycleslobyte= EEPROMvalue[41];
     cycleshibyte= EEPROMvalue[42];
     battcapacityLobyte= EEPROMvalue[43];
     battcapacityHibyte= EEPROMvalue[44];
     
    cycles=cycleslobyte+(cycleshibyte*256);
    battcapacity=battcapacityLobyte+(battcapacityHibyte*256);
    Cbattcapacity=battcapacity;  
    if (coulcorrbyte>=160 or coulcorrbyte<=70){
       coulcorrarm=1;
       coulcorr=1.00;
    }
    else{
       coulcorrarm=0;
       coulcorr=coulcorrbyte/100.00;
    }
    
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
  lcd.print("V1.04.02/10.02.2021");

  //OLED  SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Altn Address 0x3D for 128x64
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();

  //DHT11
  HT.begin();

  //Ext WDT
  pinMode(ExtWDTpin,OUTPUT);

  //Temp probes
  pinMode(Probe1,INPUT);
  pinMode(Probe2,INPUT);
  pinMode(Probe3,INPUT);

  //12V Sense
  pinMode(Vsense,INPUT);

  //INA219
  //ina219.init();
  //ina219.setADCMode(SAMPLE_MODE_32);  // choose how many samples to take each measurement (2,4,8,16,32,64,128)
  //ina219.setMeasureMode(TRIGGERED);   // Triggered measurements
  //ina219.setPGain(PG_160);             // choose gain according your shunt voltage (40,80,160,320) mV max
  //ina219.setCorrectionFactor(0.99); // insert your correction factor if necessary

  //INA226
  ina226.init();
  ina226.setAverage(AVERAGE_64); // choose how many samples to take each measurement (1,4,16,64,128,256,512,1024)
  ina226.setMeasureMode(TRIGGERED);   // Triggered measurements
  ina226.setCurrentRange(MA_400);  //choose gain according your shunt voltage (400,800) mA max
  ina226.setCorrectionFactor(1.67);
  
  my_mux.channel(vc);
  
  pinMode(Battswitch,INPUT);

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
  pinMode(relayPin6,OUTPUT);  // optional Charger 3

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
  pinMode (SW,INPUT);
  // pinMode (SW,INPUT_PULLUP);        if encoder doesnt have pullup resistor
  pinAStateHigh = digitalRead(CLK);
  pinBStateHigh = digitalRead(DT);
  oldPinAStateHigh = pinAStateHigh;
  oldPinBStateHigh = pinBStateHigh; 
  
  delay(1500);

  switchpress=digitalRead(SW);
  if (switchpress==0){
    wifidis=1;                          // wifi and Blynk can be overriden when switch pushed at startup
   }

  if (wifiavail==0){
    wifidis=1; 
  }

// Ext WatchdogTimer reset
if (ExtWDT==1){
  simpleWDTreset();
}

  // ESP and Blynk 
  if (wifidis==0){
  EspSerial.begin(ESP8266_BAUD);
  delay(100);
  lcd.setCursor(0,1);
  lcd.print(" connecting WiFi... ");
  wifi.setDHCP(1, 1, 1); //Enable dhcp in station mode and save in flash of esp8266
  Blynk.config(wifi, auth, "blynk-cloud.com", 8080); 
    if (ExtWDT==1){
      simpleWDTreset();
    }
  if (Blynk.connectWiFi(ssid, pass)) {
      if (ExtWDT==1){
        simpleWDTreset();
      }
     lcd.setCursor(0,1);
     lcd.print(" connecting Blynk.. ");
     Blynk.connect();
      if (ExtWDT==1){
        simpleWDTreset();
      }
     if (Blynk.connected()){
     lcd.setCursor(0,1);
     lcd.print("    Blynk... OK     ");
     linkState=1;
     Blynk.run();
     }
     else{
     lcd.setCursor(0,1);
     lcd.print("   Blynk failed!    ");
     connectCounter=1;
     }
     
     }
     else {
     linkState = 0;
     connectCounter=1;
     }
  }

  // Ext WatchdogTimer reset
  if (ExtWDT==1){
    simpleWDTreset();
  }
  
  // Modbus setup
  // SERIAL_BEGIN(9600);
 if (typeofmeter!=9){ 
  if (typeofmeter==0){
    MB_BAUDRATE=2400;
    }
    else{
    MB_BAUDRATE=9600;  
    }
   
  MBserial.begin(MB_BAUDRATE, MB_BYTEFORMAT, REFRESH_INTERVAL);
 }
  
  //LEDs
  pinMode(greenpin,OUTPUT);  // Power available
  pinMode(redpin,OUTPUT);  // Inverting
  pinMode(yellowpin,OUTPUT);  // Charging

  // ADS1115
  // ads.setGain(GAIN_TWOTHIRDS);   // +/- 6.144V  1 bit =  0.1875mV (default)
  // ADSGain=0.0001875;
   ads.setGain(GAIN_ONE);       // +/- 4.096V  1 bit =  0.125mV
   ADSGain=0.000125;
   ads.begin();
 
  // SDcard
  pinMode(CS,OUTPUT);

   fineadjustment=(fineadjustmentbyte-30)/700000;   // with ADS1115;
   calibrationvalue=0.001945;   //calibrate your readout with a volt meter!
  
  delay(3000);
  
  lcd.clear();

  if (SD.begin(CS)) {
    lcd.setCursor(0,0);
    lcd.print("   SD Card ready    ");
    lcd.setCursor(0,1);
    lcd.print("Data will be logged ");
    lcd.setCursor(0,2);
    lcd.print("as long Card present");
    lcd.setCursor(0,3);
    lcd.print("in Modes 1 & 2 only ");
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

  if (ExtWDT==1){
      simpleWDTreset();
  }


 // Wait 5 second
  delay(4000);
  
  lcd.clear();
  
  if (ExtWDT==1){
  simpleWDTreset();
  }


  cyclestartmillis=millis(); 
  ledstartmillis=millis();
  sendstartmillis=millis();
  modestartmillis=millis();
  loopstartmillis=millis();
  ExtWDTstartmillis=millis();
  Aloopstartmillis=millis();
  wattsstartmillis=millis();
  coulombstartmillis=millis();
  dwattsstartmillis=millis();

  battCapFact=(0.940+(battcapacity/(ConeAmps+CtwoAmps+CthreeAmps))*0.0001);
}

void loop(){

// Main WatchdogTimer reset
if (ExtWDT==1){
  resetExtWDT();
}

// getting the Modbus readout
if (mode<9){

if (typeofmeter!=9){           // no meter installed
  
if (typeofmeter==0){

if (MBserial.available()) {

    oldmeasuredpower1=measuredpower1;
    
    measuredpower1=pwr1.read();
    measuredpower1=round(measuredpower1);

      if (measuredpower1==0){
        measuredpower1=oldmeasuredpower1;
      }
  
    measuredpower=measuredpower1;
  
    trueread=1;
}
}

if (typeofmeter==1 or typeofmeter==2){

if (MBserial.available()) {

    oldmeasuredpower1=measuredpower1;
    oldmeasuredpower2=measuredpower2;
    oldmeasuredpower3=measuredpower3; 
     
    measuredpower1=pwr1.read();
    measuredpower1=round(measuredpower1);
    blankpower=blank1.read();
    measuredpower2=pwr2.read();
    measuredpower2=round(measuredpower2);
    blankpower=blank2.read();
    measuredpower3=pwr3.read();
    measuredpower3=round(measuredpower3);

      if (measuredpower1==0){
        measuredpower1=oldmeasuredpower1;
      }
      if (measuredpower2==0){
        measuredpower2=oldmeasuredpower2;
      }
      if (measuredpower3==0){
        measuredpower3=oldmeasuredpower3;
      }

    if (activephase==1){
    measuredpower=measuredpower1;
    }
    if (activephase==2){
    measuredpower=measuredpower2;
    }
    if (activephase==3){
    measuredpower=measuredpower3;
    }
  
    trueread=1;
}
}

}
else{
    measuredpower1=0;
    measuredpower2=0;
    measuredpower3=0; 
    measuredpower=0;
}
  

if (trueread==1){
if (invertreadout==1) {
  measuredpower=measuredpower*-1;
  if (Cthree==0){
  measuredpowerS=0;
  }
  if (Cthree==1){
  measuredpowerS=measuredpower1*-1;
  }
  if (Cthree==2){
  measuredpowerS=measuredpower2*-1;
  }
  if (Cthree==3){
  measuredpowerS=measuredpower3*-1;
  }
}
trueread=0;
}

}

if (millis()>=(modestartmillis+modeinterval)) {
  modestartmillis=millis();
  
mode=13;
if (digitalRead(Mode1Pin)==HIGH) { mode=1; }
if (digitalRead(Mode2Pin)==HIGH) { mode=2; }
if (digitalRead(Mode3Pin)==HIGH) { mode=3; }
if (digitalRead(Mode4Pin)==HIGH) { mode=4; }
if (digitalRead(Mode5Pin)==HIGH) { mode=5; }
if (digitalRead(Mode6Pin)==HIGH) { mode=6; }
if (digitalRead(Mode7Pin)==HIGH) { mode=oldmode; }     //not used
if (digitalRead(Mode8Pin)==HIGH) { mode=8; }
if (digitalRead(Mode9Pin)==HIGH) { mode=9; }
if (digitalRead(Mode10Pin)==HIGH) { mode=10; }
if (digitalRead(Mode11Pin)==HIGH) { mode=11; }
if (digitalRead(Mode12Pin)==HIGH) { mode=12; }

if(mode!=oldmode){
  OLEDstop=0;
  setpos=0;
  Stopdevices();
  modestartdelaymillis=millis();
}

if (mode==13){
  mode=oldmode;
}

oldmode=mode;
pagelock=0;

if (OLEDstop==0){
  displaymode();
  if (ExtWDT==1){
    simpleWDTreset();
  }
  OLEDstop=1;
}

}

 if (mode==1 and millis()>=(loopstartmillis+shortloopinterval)){
  loopstartmillis=millis();
  Mode1();
  }

 if (mode==2 and millis()>=(loopstartmillis+shortloopinterval)){
  loopstartmillis=millis();  
  Mode2();
 }
 
 if (mode==3 and millis()>=(loopstartmillis+longloopinterval)){
  loopstartmillis=millis();  
  Mode3();
  }
  
 if (mode==4 and millis()>=(loopstartmillis+shortloopinterval)){
  loopstartmillis=millis();  
  Mode4();
  }

 
if (millis()>=modestartdelaymillis+4000){
 if (mode==5 and millis()>=(loopstartmillis+longloopinterval)){
  loopstartmillis=millis();  
  Mode5();
 }

 if (mode==6 and millis()>=(loopstartmillis+longloopinterval)){
  loopstartmillis=millis();  
  Mode6();
 }

/*
  if (mode==7 and millis()>=(loopstartmillis+longloopinterval)){
  loopstartmillis=millis();  
  Mode7();
  }
  */
  
 if (mode==8 and millis()>=(loopstartmillis+longloopinterval)){
  loopstartmillis=millis();  
  Mode8();
  }
 }
 
 if (mode==9){
  Mode9();
 }

 if (mode==10){
  Mode10();
 }

 if (mode==11){
  Mode11();
 }
 
 if (mode==12){
  Mode12();
 }
  

// current loop
 if (millis()>=(Aloopstartmillis+Aloopinterval)){
  Aloopstartmillis=millis();
  getCurrent();
 }


if (millis()>=(ledstartmillis+ledinterval)) {
  LEDsignals();
  ledstartmillis=millis();

  CheckFan(); 
  
}

 if (millis()>=(sendstartmillis+sendinterval)) {
  sendstartmillis=millis();

 if (mode<9){

  if (wifidis==0){
    
  if (connectCounter==0){
   if (!Blynk.connected()) {                               //If Blynk is not connected
    linkState = 0;
    connectCounter=1;
    if (ExtWDT==1){
      simpleWDTreset();
    }
    delay(5000);    
    startBlynk();
   }
   else{
    if (ExtWDT==1){
      simpleWDTreset();
    }
   Blynk.run();
   } 
  }
  }
   
  if (linkState==1){  
    if (transmitcounter==0){  
     BlynkDataTransmit();
     transmitcounter=1;
    if (ExtWDT==1){
      simpleWDTreset();
    }   
    }
    else{
     transmitcounter=0;
    }
  }
  
 }
 
 }


 if (millis()>=(currentstartmillis+currentinterval)) {
   currentstartmillis=millis();

  if (mode<9){
    
   Currentaquisition();

   Vcounter++;
   
   if (Vcounter>=2){
    get12Vsense();
    Vcounter=0;
   }
  
  }

 }

if (Dreset==1 and dt.hour==Dresethr and dt.minute==Dresetmin){
  if (dt.second>0 and dt.second<=5){
      resetFunc(); 
  }                                    // resets system according setup
}

if (millis()>4000000000){             // resets system after about 47 days runtime
      resetFunc(); 
}   


}


void Mode1(){

  // Normal Mode - Excess Metered (Mode1),  Relays are ON when "L0W"
 
    switchpress=digitalRead(SW);
    Battmode=digitalRead(Battswitch);
    
    // charger activation cycle

 Powercalculations();
    
    if (chargerone==1 and chargertwo==0) {
    realpowerpri=averagepowerpool+(battvoltage*ConeAmps);
    }
    if (chargerone==0 and chargertwo==1) {
    realpowerpri=averagepowerpool+(battvoltage*CtwoAmps);
    }
    if (chargerone==1 and chargertwo==1) {
    realpowerpri=averagepowerpool+(battvoltage*(ConeAmps+CtwoAmps));
    }
    if (chargerone==0 and chargertwo==0) {
    realpowerpri=averagepowerpool;
    }

if (Maxlock==1 or SOCCoul>maximumSOC) {
   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin6,HIGH);
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   chargerlock=1;
   Maxlock=1;
}

if (chargersnbr<=1 and battSOC>=96 and inverter==0){
  SOCCoul=battSOC;
  CoulombPointer=(Cbattcapacity/100)*SOCCoul;
}


  if (battSOC<=maximumSOC-8){
    Maxlock=0;
  }

    
if (millis()>=(cyclestartmillis+relaycycletime)) {

  if (coulstart==0){
    SOCCoul=battSOC;
    coulstart=1;
    CoulombPointer=(Cbattcapacity/100)*SOCCoul;
    coulcorrstart=battSOC;
  }
 
// upper limit for charging
if (SOCCoul<=maximumSOC and Maxlock==0 and MinLock==0) {
  



if (inverter==1) {
  chargerlock=0;
}
else{
   if (SOCCoul>=maximumSOC-4 and chargersnbr>=2){
     battSOC=battSOC-5;
     SOCfreeze=65;
     chargerlock=1;
   }
   if (SOCCoul>=maximumSOC-4 and chargersnbr<=1){
     chargerlock=1;
   }
   if (chargersnbr==3 and battvoltage>=55.0){
     battSOC=battSOC-5;
     SOCfreeze=65;
     chargerlock=1;
   }
   if (chargersnbr==2 and battvoltage>=54.45){
     battSOC=battSOC-5;
     SOCfreeze=65;
     chargerlock=1;
   }
}

  
   if (realpowerpri<=(((dynlevel+triggerpowermode1)*10)+(battvoltage*ConeAmps))) {
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     chargerone=0;
     chargertwo=0;
   } 
   if (realpowerpri>(((dynlevel+triggerpowermode1)*10)+(battvoltage*ConeAmps)) and realpowerpri<(((dynlevel+triggerpowermode1)*10)+(battvoltage*CtwoAmps))) {
     digitalWrite(relayPin1,LOW);
     digitalWrite(relayPin2,HIGH);
     chargerone=1;
     chargertwo=0;
   }
    if (realpowerpri>=(((dynlevel+triggerpowermode1)*10)+(battvoltage*CtwoAmps)) and realpowerpri<(((dynlevel+triggerpowermode1)*10)+(battvoltage*ConeAmps)+(battvoltage*CtwoAmps))) {
     digitalWrite(relayPin1,HIGH);
     chargerone=0;
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
   } 
   if (realpowerpri>=(((dynlevel+triggerpowermode1)*10)+(battvoltage*ConeAmps)+(battvoltage*CtwoAmps))) {
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
     if (chargerlock==0) {
     digitalWrite(relayPin1,LOW);
     chargerone=1;
     }
     else {
     digitalWrite(relayPin1,HIGH);
     chargerone=0; 
     }
   } 

   Activatechargerthree();
   
}
else{
     digitalWrite(relayPin1,HIGH);
     digitalWrite(relayPin2,HIGH);
     digitalWrite(relayPin6,HIGH);
     chargerone=0;
     chargertwo=0;
     chargerthree=0;
     chargerlock=1;
  
}

  Datalogging();
   
  Newpowercycle();
    
 }

 Checkidlestatus();
  
 getVoltage();
 
 getSOC();

 getTempprobes();

 Scounter++;
 if (Scounter>=2){
    getStringvoltage();
    Scounter=0;
 }   

dt = clock.getDateTime();

if (Battmode==0) {

if (switchpress==1) {
  subpage=0;
  lcd.setCursor(0,0);
  if (inverter==1) {
    lcd.print("Inverter: ON        ");
  }
  else{
      
  if (chargerone==1 and chargertwo==0) {
    lcd.print("C1:ON  C2:OFF       ");
  }
  if (chargerone==0 and chargertwo==1) {
    lcd.print("C1:OFF C2:ON        ");
  }
  if (chargerone==1 and chargertwo==1) {
    lcd.print("C1:ON  C2:ON        ");
  }
 if (chargerone==0 and chargertwo==0) {
    lcd.print("C1:OFF C2:OFF       ");
  } 
 if (Cthree>0 and chargerthree==0){
    lcd.setCursor(14,0);
    lcd.print("C3:OFF");    
 }
 if (Cthree>0 and chargerthree==1){
    lcd.setCursor(14,0);
    lcd.print("C3:ON");    
 }

 }
  
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(SOCDisplay);
    lcd.setCursor(19,1);
    lcd.print("%");

if (inverter==1){
    lcd.setCursor(0,2);
    lcd.print("Inverter W:         ");
    lcd.setCursor(11,2);
    lcd.print(inverterwatts);
    }
  else{
      if (showpower==0) {
      lcd.setCursor(0,2);
      lcd.print("P-a:");
      lcd.setCursor(4,2);
      lcd.print("                ");
      lcd.setCursor(4,2);
      lcd.print(averagepowerpool,0);
      lcd.print("/");
      lcd.print(averagepowerpoolS,0);
      }
      else {
      lcd.setCursor(0,2);
      lcd.print("P-r:");
      lcd.setCursor(4,2);
      lcd.print("                ");
      lcd.setCursor(4,2);
      lcd.print(measuredpower,0);
      lcd.print("/");
      lcd.print(measuredpowerS,0);
      }
   lcd.setCursor(17,2);
   if (dynlevel==10){
    lcd.print("d"); 
   }
   if (dynlevel==20){
    lcd.print("D"); 
   }
   lcd.setCursor(19,2);
   if (averagepowerpool<lastaveragepower) {
    lcd.print("v");
   }
   else {
    lcd.print("^");
   }
 }

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
  if (subpage<=1) {
  
  lcd.setCursor(0,0);
  lcd.print("Chg Lock:      ");  
  lcd.setCursor(9,0);
  if (chargerlock==1) {
    lcd.print("ON");
  }
  if (chargerlock==0) {
    lcd.print("OFF");
  }
  lcd.setCursor(14,0);
  lcd.print("      ");
  lcd.setCursor(14,0);
  lcd.print(millis()/10000);   

  
  lcd.setCursor(0,1);
  lcd.print("                    ");  
  lcd.setCursor(0,1);   
  lcd.print(measuredpower1,0);
  lcd.print("/");  
  lcd.print(measuredpower2,0);
  lcd.print("/");  
  lcd.print(measuredpower3,0);

  
  lcd.setCursor(0,2);
  lcd.print("TempC:      ");
  lcd.setCursor(6,2);  
  lcd.print(tempC,1);
  lcd.setCursor(12,2);
  lcd.print("RH%:    ");
  lcd.setCursor(16,2);
  lcd.print(humidity,0); 
 

  lcd.setCursor(0,3);  
  if (typeofbatt==0) { 
    lcd.print("LPO "); 
  } 
  if (typeofbatt==1) { 
    lcd.print("LFP ");
  } 
  if (typeofbatt==2) { 
    lcd.print("NMC ");
  } 

 lcd.setCursor(4,3);
  if (linkState==1) { 
    lcd.print("@ ");
  }
  if (linkState==0) { 
    lcd.print("  ");
  }

 lcd.setCursor(6,3);
  if (typeofmeter==0) { 
    lcd.print("SDM120 ");
  }
   if (typeofmeter==1) { 
    lcd.print("SDM630 ");
  }
    if (typeofmeter==2) { 
    lcd.print("X835   ");
  }
     
  lcd.setCursor(13,3);
  if (invertreadout==1) { 
    lcd.print("i ");
  }
  if (invertreadout==0) { 
    lcd.print("n ");
  }

  lcd.setCursor(15,3);
    lcd.print(corrindrest);
    lcd.print("/");
    lcd.print(corrindinv);

  }

  if (subpage>=3){

  lcd.setCursor(0,0);
  lcd.print("BMS Temp    :      C");  
  lcd.setCursor(14,0);
  lcd.print(probetemp1,1);
  lcd.setCursor(0,1);
  lcd.print("Shunt Temp  :      C");  
  lcd.setCursor(14,1);
  lcd.print(probetemp2,1);
  lcd.setCursor(0,2);
  lcd.print("Chgr 2 Temp :      C");  
  lcd.setCursor(14,2);
  lcd.print(probetemp3,1);
  lcd.setCursor(0,3);
  lcd.print("WDT:                ");  
  lcd.setCursor(5,3);
  if (ExtWDT==1) { 
  lcd.print("external");
  }
  if (ExtWDT==0) { 
  lcd.print("off");
  }
  lcd.setCursor(16,3);
  lcd.print(disconnects);

   
  }

  subpage++;
  if (subpage==5){
    subpage=0;
  }  
  
 }

}
else {
  
  if (switchpress==1) {
  subpage=0;  
  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,0); 
  lcd.print(battvoltage,2); 
  lcd.print("V"); 
  lcd.setCursor(8,0);
    if (linkState==1) { 
    lcd.print("@ ");
  }
  if (linkState==0) { 
    lcd.print("  ");
  }
  lcd.setCursor(11,0); 
  lcd.print("Accur:"); 
  lcd.setCursor(18,0);
    if (Meteraccur==1) { 
    lcd.print("H");
    }
    if (Meteraccur==2) { 
    lcd.print("M");
    }
    if (Meteraccur==3) { 
    lcd.print("L");
    }
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print(current,1); 
  lcd.print("A"); 
   if(homing==1){ 
    lcd.setCursor(8,1);
    lcd.print("><"); 
   } 
  lcd.setCursor(13,1);
  lcd.print(watts,0); 
  lcd.print("W"); 
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print(battSOC); 
  lcd.print(" %byV"); 
  lcd.setCursor(10,2);
  lcd.print(SOCCoul); 
  lcd.print(" %byC"); 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print(CCwatthours,0);
  lcd.print("WhC");
  lcd.setCursor(10,3);
  lcd.print(CDwatthours,0);
  lcd.print("WhD");

  
}
else {

  if (subpage<=1) {
      
  lcd.setCursor(0,0);
  lcd.print("Cy:                 ");  
  lcd.setCursor(3,0);
  lcd.print(cycles);
  lcd.setCursor(9,0);
  lcd.print(chgwatthours,0);
  lcd.print("WhC"); 
  if (capsave==1){ 
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print("Start:      ");  
  lcd.setCursor(7,1);
  lcd.print(StartSOC);
  lcd.print("%");
  lcd.setCursor(12,1);
  lcd.print("End:    ");  
  lcd.setCursor(17,1);
  lcd.print(EndSOC);
  lcd.print("%");
  lcd.setCursor(0,2);
  lcd.print("BattCap:        Wh  ");  
  lcd.setCursor(9,2);
  lcd.print(watthoursTot,0);
  lcd.setCursor(0,3);
  lcd.print("BattCap:        Ah  ");  
  lcd.setCursor(9,3);
  lcd.print(AhoursTot,0);
  }
  if (capsave==0){
  lcd.setCursor(0,1);
  lcd.print("C-meter drift:      "); 
  lcd.setCursor(15,1);
  lcd.print(coulcorr);     
  lcd.setCursor(0,2);
  lcd.print("BattCap:        Wh  ");  
  lcd.setCursor(9,2);
  lcd.print(battcapacity);
  lcd.setCursor(0,3);
  if (Dtest==0){  
  lcd.print("BattCap: -----  Ah  ");  
  }
  else{
  lcd.print("CapTest in progress ");      
  }
  }

  }

  if (subpage>=2 and subpage<=5){

  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,0);  
  lcd.print(stringvoltage[0],2);
  lcd.print("/");  
  lcd.print(stringvoltage[1],2);
  lcd.print("/");  
  lcd.print(stringvoltage[2],2);
  lcd.print("/");  
  lcd.print(stringvoltage[3],2);
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,1);    
  lcd.print(stringvoltage[4],2);
  lcd.print("/");  
  lcd.print(stringvoltage[5],2);
  lcd.print("/");  
  lcd.print(stringvoltage[6],2);
  lcd.print("/");  
  lcd.print(stringvoltage[7],2);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);    
  lcd.print(stringvoltage[8],2);
  lcd.print("/");  
  lcd.print(stringvoltage[9],2);
  lcd.print("/");  
  lcd.print(stringvoltage[10],2);
  lcd.print("/");  
  lcd.print(stringvoltage[11],2);
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);    
  lcd.print(stringvoltage[12],2);
  lcd.print("/");  
  lcd.print(stringvoltage[13],2);
  lcd.print("/");  
  lcd.print(stringvoltage[14],2);
  lcd.print("/");  
  lcd.print(stringvoltage[15],2);
      
  }

  if (subpage>=6){

  lcd.setCursor(0,0);
  lcd.print("Delta:              ");
  lcd.setCursor(7,0);  
  lcd.print(delta,2);
  lcd.setCursor(0,1);
  lcd.print("high:      Cell:    "); 
  lcd.setCursor(6,1);  
  lcd.print(highestSV,2);
  lcd.setCursor(17,1);   
  lcd.print(highestSnbr,0);
  lcd.setCursor(0,2); 
  lcd.print("low:       Cell:    "); 
  lcd.setCursor(6,2);  
  lcd.print(lowestSV,2);
  lcd.setCursor(17,2);   
  lcd.print(lowestSnbr,0);
  lcd.setCursor(0,3);
  lcd.print("                    ");
       
  }
  
  subpage++;
  if (subpage==9){
    subpage=0;
  }  
  
}
      
}



   if (battSOC>minimumSOC+10){
    inverteravailable=1; 
   }

Checkminimumreached();

Startinverting();

InverterSOCcorrection();

}




void Mode2(){

  // Timer Based (Mode2)
  switchpress=digitalRead(SW);
  dt = clock.getDateTime();
 
  timermatch=0;
  timer2match=0;
  timer3match=0;
  actC1Timer=0;
  actC2Timer=0;
  actInvTimer=0;

 Powercalculations();

 if (millis()>=(cyclestartmillis+relaycycletime)) {

  if (Limiter==1 and typeofmeter!=9){
  LimiterOps=1;
  Limiterfirst=0;
  }
  
  Datalogging();
   
  Newpowercycle();
 
 }


   invertertime=(dt.hour);
    if (battSOC>minimumSOC+10){
       inverteravailable=1; 
     }

 Checkminimumreached();
 
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

if (LimiterOps==1){
  
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

}

Limitercheck();

 getVoltage(); 

 getSOC(); 
 
    
 if (switchpress==1) { 
  lcd.setCursor(0,0);
  lcd.print("                    ");
      
  if (inverter==1){

    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:OFF Inv:ON");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
         if (stopdelay==0){
            startmillis=millis();
            invcorract=1;    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
    digitalWrite(relayPin3,LOW);
    delay(prechgreleasedelay);
    digitalWrite(relayPin5,HIGH);

  }
  
  
  if (battSOC>=99 or Maxlock==1 or battSOC>maximumSOC){
    chargerone=0;
    chargertwo=0;
    chargerlock=1;
    Maxlock=1;
  }
  if (battSOC<=maximumSOC-8){
    Maxlock=0;
  }

  if (battSOC>maximumSOC){
  chargerlock=1;
  }
  
  
  if (inverter==0 and chargerone==1 and chargertwo==0){
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("C1:ON C2:OFF Inv:OFF");
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
  if (inverter==0 and chargerone==0 and chargertwo==0){
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }
 
 if (inverter==0 and chargerone==0 and chargertwo==1){
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:ON Inv:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,LOW);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    stopdelay=0;
  }

   if (inverter==0 and chargerone==1 and chargertwo==1){
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,0);
    lcd.print("C1:ON C2:ON Inv:OFF ");
    digitalWrite(relayPin1,LOW);
     if (chargerlock==0) {
     digitalWrite(relayPin2,LOW);
     lcd.setCursor(0,0);
     lcd.print("C1:ON C2:ON Inv:OFF ");
     }
     else {
     digitalWrite(relayPin2,HIGH);
     chargertwo=0; 
     lcd.setCursor(0,0);
     lcd.print("C1:ON C2:OFF Inv:OFF");
     }
     digitalWrite(relayPin5,HIGH);
     digitalWrite(relayPin3,HIGH);
     stopdelay=0;
  }

 CheckChargerthree();

 Checkidlestatus();
 
   
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

  lcd.setCursor(0,2);
  lcd.print("TempC:      ");
  lcd.setCursor(6,2);  
  lcd.print(tempC,1);
  lcd.setCursor(12,2);
  lcd.print("P:      ");
  lcd.setCursor(14,2);
  lcd.print(measuredpower,0); 
  lcd.setCursor(19,2);
    if (Limiter==1) { 
    lcd.print("L");
    }
 
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

 if (subpage<=1) {
   
   lcd.setCursor(0,0);
  lcd.print("Active Timers:      ");
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

  if (subpage>=2){

  lcd.setCursor(0,0);
  lcd.print("Temp-probe 1:      C");  
  lcd.setCursor(14,0);
  lcd.print(probetemp1,1);
  lcd.setCursor(0,1);
  lcd.print("Temp-probe 2:      C");  
  lcd.setCursor(14,1);
  lcd.print(probetemp2,1);
  lcd.setCursor(0,2);
  lcd.print("Temp-probe 3:      C");  
  lcd.setCursor(14,2);
  lcd.print(probetemp3,1);
  lcd.setCursor(0,3);
  lcd.print("WDT:                ");  
  lcd.setCursor(5,3);
  if (ExtWDT==1) { 
  lcd.print("external");
  }
  if (ExtWDT==0) { 
  lcd.print("off");
  }
  lcd.setCursor(16,3);
  lcd.print(disconnects);

  }

  subpage++;
  if (subpage==4){
    subpage=0;
  }  
  
}

InverterSOCcorrection();

}



void Mode3(){

   // Charger 1 - manuell ON (Mode3), Relays are ON when "L0W"

  switchpress=digitalRead(SW);
 
  if (battSOC>=99 or Maxlock==1) {
   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin6,HIGH);
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   Maxlock=1;
 }

  if (battSOC<=93){
    Maxlock=0;
  }
    
  if (battSOC<99 and Maxlock==0){      // only this mode allows 100% charge
    lcd.setCursor(0,0);
    lcd.print("C1:ON C2:OFF C3:OFF ");
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin6,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=1;
    chargertwo=0;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:OFF C3:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,HIGH);
    chargerone=0;
    chargertwo=0;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
  }
    
 getVoltage();  

 getSOC();    

 if (switchpress==1){
   
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

   lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print(current,1); 
  lcd.print("A"); 
   if(homing==1){ 
    lcd.setCursor(8,2);
    lcd.print("><"); 
   } 
  lcd.setCursor(13,2);
  lcd.print(watts,0); 
  lcd.print("W"); 

 
  
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
 else {
  
    lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,0); 
  lcd.print(battvoltage,2); 
  lcd.print("V"); 
  lcd.setCursor(8,0);
    if (linkState==1) { 
    lcd.print("@ ");
  }
  if (linkState==0) { 
    lcd.print("  ");
  }
  lcd.setCursor(11,0); 
  lcd.print("Accur:"); 
  lcd.setCursor(18,0);
    if (Meteraccur==1) { 
    lcd.print("H");
    }
    if (Meteraccur==2) { 
    lcd.print("M");
    }
    if (Meteraccur==3) { 
    lcd.print("L");
    }

  lcd.setCursor(0,1);
  lcd.print("TempC:      ");
  lcd.setCursor(6,1);  
  lcd.print(tempC,1);
  lcd.setCursor(12,1);
  lcd.print("RH%:    ");
  lcd.setCursor(16,1);
  lcd.print(humidity,0); 

  
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print(battSOC); 
  lcd.print(" %byV"); 
  lcd.setCursor(10,2);
  lcd.print(SOCCoul); 
  lcd.print(" %byC"); 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print(CCwatthours,0);
  lcd.print("WhC");
  lcd.setCursor(10,3);
  lcd.print(CDwatthours,0);
  lcd.print("WhD");

  }

}


void Mode4(){

  // Charger 2 - manuell ON (Mode4), Relays are ON when "L0W"

  lcd.setCursor(0,0);

if (battSOC>=99 or Maxlock==1 or battSOC>maximumSOC) {
   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin6,HIGH);
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   Maxlock=1;
}

  if (battSOC<=maximumSOC-8){
    Maxlock=0;
  }
    
    if (battSOC<=maximumSOC and Maxlock==0){  
    lcd.print("C1:OFF C2:ON C3:OFF ");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,LOW);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,HIGH);
    chargerone=0;
    chargertwo=1;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
  }
  else {
    lcd.print("C1:OFF C2:OFF C3:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,HIGH);
    chargerone=0;
    chargertwo=0;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
    Maxlock=1;
  }

 getVoltage(); 

 getSOC();    

  
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

   lcd.setCursor(0,2);
  lcd.print("TempC:      ");
  lcd.setCursor(6,2);  
  lcd.print(tempC,1);
  lcd.setCursor(12,2);
  lcd.print("RH%:    ");
  lcd.setCursor(16,2);
  lcd.print(humidity,0); 

 
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

void Mode5(){

   // Charger 3 - manuell ON (Mode5), Relays are ON when "L0W"
 
  if (battSOC>=99 or Maxlock==1 or battSOC>maximumSOC) {
   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin6,HIGH);
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   Maxlock=1;
}

  if (battSOC<=maximumSOC-8){
    Maxlock=0;
  }
    
    if (battSOC<=maximumSOC and Maxlock==0){  
    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:OFF C3:ON ");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,LOW);
    chargerone=0;
    chargertwo=0;
    chargerthree=1;
    inverter=0;
    stopdelay=0;
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,HIGH);   
    chargerone=0;
    chargertwo=0;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
  }
    
 getVoltage();  

 getSOC();    

  
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

  lcd.setCursor(0,2);
  lcd.print("TempC:      ");
  lcd.setCursor(6,2);  
  lcd.print(tempC,1);
  lcd.setCursor(12,2);
  lcd.print("RH%:    ");
  lcd.setCursor(16,2);
  lcd.print(humidity,0); 
 
  
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

  // All Chargers - manuell ON (Mode7), Relays are ON when "L0W"

if (battSOC>=99 or Maxlock==1 or battSOC>maximumSOC) {
   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin6,HIGH);
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   chargerlock=1;
   Maxlock=1;
}

  if (battSOC<=maximumSOC-8){
    Maxlock=0;
  }
  
   if (battSOC<=maximumSOC and Maxlock==0){    
    lcd.setCursor(0,0);
    digitalWrite(relayPin1,LOW);
    digitalWrite(relayPin6,LOW);    
     if (chargerlock==0) {
     lcd.print("C1:ON C2:ON C3:ON  "); 
     digitalWrite(relayPin2,LOW);
     chargertwo=1;
     chargerthree=1;
     }
     else {
     lcd.print("C1:ON C2:OFF C3:ON ");         
     digitalWrite(relayPin2,HIGH);
     chargertwo=0;
     chargerthree=1; 
     }
     digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    chargerone=1;
    inverter=0;
    stopdelay=0;
   }
   else {
    lcd.setCursor(0,0); 
    lcd.print("C1:OFF C2:OFF In:OFF");
    digitalWrite(relayPin1,HIGH);
    digitalWrite(relayPin2,HIGH);
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    digitalWrite(relayPin6,HIGH);
    chargerone=0;
    chargertwo=0;
    chargerthree=0;
    inverter=0;
    stopdelay=0;
    chargerlock=1;
  }

 getVoltage(); 
 
 getSOC();  

  
    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

  lcd.setCursor(0,2);
  lcd.print("TempC:      ");
  lcd.setCursor(6,2);  
  lcd.print(tempC,1);
  lcd.setCursor(12,2);
  lcd.print("RH%:    ");
  lcd.setCursor(16,2);
  lcd.print(humidity,0); 

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
  
  Battmode=digitalRead(Battswitch);

  if (battSOC<97 and Maxlock==1){
       Maxlock=0; 
     }

     if (battSOC>2){
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
            startmillis=millis();
            invcorract=1;    
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
    digitalWrite(relayPin3,LOW);
    delay(prechgreleasedelay);
    digitalWrite(relayPin5,HIGH);
    inverter=1;
    }
    else  {
    digitalWrite(relayPin5,HIGH);
    digitalWrite(relayPin3,HIGH);
    inverter=0;
    stopdelay=0;
    }
  }

 getVoltage();  
 
 getSOC();   


if (Battmode==0){
    lcd.setCursor(0,0);
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


    lcd.setCursor(0,1);
    lcd.print("BatV:");
    lcd.setCursor(8,1);
    lcd.print("     ");
    lcd.setCursor(5,1);
    lcd.print(battvoltage);
    lcd.setCursor(12,1);
    lcd.print("SOC:");
    lcd.setCursor(17,1);
    lcd.print("  ");
    lcd.setCursor(16,1);
    lcd.print(battSOC);
    lcd.setCursor(19,1);
    lcd.print("%");

    lcd.setCursor(0,2);
    lcd.print("Inverter W:         ");
    lcd.setCursor(12,2);
    lcd.print(inverterwatts);

 
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
  else {
  
  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,0); 
  lcd.print(battvoltage,2); 
  lcd.print("V"); 
  lcd.setCursor(8,0);
    if (linkState==1) { 
    lcd.print("@ ");
  }
  if (linkState==0) { 
    lcd.print("  ");
  }
  lcd.setCursor(11,0); 
  lcd.print("Accur:"); 
  lcd.setCursor(18,0);
    if (Meteraccur==1) { 
    lcd.print("H");
    }
    if (Meteraccur==2) { 
    lcd.print("M");
    }
    if (Meteraccur==3) { 
    lcd.print("L");
    }
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print(current,1); 
  lcd.print("A"); 
   if(homing==1){ 
    lcd.setCursor(8,1);
    lcd.print("><"); 
   } 
  lcd.setCursor(13,1);
  lcd.print(watts,0); 
  lcd.print("W"); 
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print(battSOC); 
  lcd.print(" %byV"); 
  lcd.setCursor(10,2);
  lcd.print(SOCCoul); 
  lcd.print(" %byC"); 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print(CCwatthours,0);
  lcd.print("WhC");
  lcd.setCursor(10,3);
  lcd.print(CDwatthours,0);
  lcd.print("WhD");

}

InverterSOCcorrection();

}


void Mode9(){

  // Timer Program (Mode9)

if (pagelock==0){
  if (setpos==0) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9a: Set Timer 1");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to start ");
  lcd.setCursor(0,2);
  lcd.print("Dischg prio over chg");
  lcd.setCursor(0,3);
  lcd.print("T1 priority over T2 ");
  }

  if (setpos>=1 and setpos<=6) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9a: Set Timer 1");  
  lcd.setCursor(0,1);
  lcd.print("C1 ON:   h  ");
  lcd.setCursor(7,1);
  if (setT1C1On<10){ lcd.print("0");} 
  lcd.print(setT1C1On);
  lcd.setCursor(11,1);
  lcd.print("OFF:   h ");
  lcd.setCursor(16,1);
  if (setT1C1Off<10){ lcd.print("0");} 
  lcd.print(setT1C1Off);
  lcd.setCursor(18,1);
  lcd.setCursor(0,2);
  lcd.print("C2 ON:   h  ");
  lcd.setCursor(7,2);
  if (setT1C2On<10){ lcd.print("0");} 
  lcd.print(setT1C2On);
  lcd.setCursor(11,2);
  lcd.print("OFF:   h ");
  lcd.setCursor(16,2);
  if (setT1C2Off<10){ lcd.print("0");} 
  lcd.print(setT1C2Off);
  lcd.setCursor(18,2);
  lcd.setCursor(0,3);
  lcd.print("Inv ON:   h ");
  lcd.setCursor(8,3);
  if (setT1InvOn<10){ lcd.print("0");} 
  lcd.print(setT1InvOn);
  lcd.setCursor(10,3);
  lcd.setCursor(12,3);
  lcd.print("OFF:   h");
  lcd.setCursor(17,3);
  if (setT1InvOff<10){ lcd.print("0");} 
  lcd.print(setT1InvOff);
  lcd.setCursor(19,3);
  }
  
  if (setpos>=7 and setpos<=12) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9b: Set Timer 2");  
  lcd.setCursor(0,1);
  lcd.print("C1 ON:   h");
  lcd.setCursor(7,1);
  if (setT2C1On<10){ lcd.print("0");} 
  lcd.print(setT2C1On);
  lcd.setCursor(9,1);
  lcd.setCursor(11,1);
  lcd.print("OFF:   h ");
  lcd.setCursor(16,1);
  if (setT2C1Off<10){ lcd.print("0");} 
  lcd.print(setT2C1Off);
  lcd.setCursor(18,1);
  lcd.setCursor(0,2);
  lcd.print("C2 ON:   h");
  lcd.setCursor(7,2);
  if (setT2C2On<10){ lcd.print("0");} 
  lcd.print(setT2C2On);
  lcd.setCursor(9,2);
  lcd.setCursor(11,2);
  lcd.print("OFF:   h ");
  lcd.setCursor(16,2);
  if (setT2C2Off<10){ lcd.print("0");} 
  lcd.print(setT2C2Off);
  lcd.setCursor(18,2);
  lcd.setCursor(0,3);
  lcd.print("Inv ON:   h");
  lcd.setCursor(8,3);
  if (setT2InvOn<10){ lcd.print("0");} 
  lcd.print(setT2InvOn);
  lcd.setCursor(10,3);
  lcd.setCursor(12,3);
  lcd.print("OFF:   h");
  lcd.setCursor(17,3);
  if (setT2InvOff<10){ lcd.print("0");} 
  lcd.print(setT2InvOff);
  lcd.setCursor(19,3);
  }

  if (setpos>12 and setpos<=16) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9c: Other Modes");
  lcd.setCursor(0,1);
  lcd.print("Inv ON:   h ");
  lcd.setCursor(8,1);
  if (setNormInvHrOn<10){ lcd.print("0");} 
  lcd.print(setNormInvHrOn);
  lcd.setCursor(12,1);
  lcd.print("   min  ");
  lcd.setCursor(12,1);
  if (setNormInvMinOn<10){ lcd.print("0");} 
  lcd.print(setNormInvMinOn);
  lcd.setCursor(0,2);
  lcd.print("Inv OFF:   h ");
  lcd.setCursor(8,2);
  if (setNormInvHrOff<10){ lcd.print("0");} 
  lcd.print(setNormInvHrOff);
  lcd.setCursor(13,2);
  lcd.print("   min ");
  lcd.setCursor(13,2);
  if (setNormInvMinOff<10){ lcd.print("0");} 
  lcd.print(setNormInvMinOff);
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }

  if (setpos>16) {
  lcd.setCursor(0,0);
  lcd.print("Mode 9d: Other Modes");
  lcd.setCursor(0,1);
  lcd.print("Daily reset:        ");
  lcd.setCursor(14,1);
  if (setDreset==0){
   lcd.print("NO "); 
  }
  else{
   lcd.print("YES"); 
  }
  lcd.setCursor(0,2);
  lcd.print("At:    h  ");
  lcd.setCursor(4,2);
  if (setDresethr<10){ lcd.print("0");}
  lcd.print(setDresethr);
  lcd.setCursor(10,2);
  lcd.print("   min    ");
  lcd.setCursor(10,2);
  if (setDresetmin<10){ lcd.print("0");}
  lcd.print(setDresetmin);
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }

  pagelock=1;
 }

  if (setpos==1){
  lcd.setCursor(7,1);
  if (setT1C1On<10){ lcd.print("0");} 
  lcd.print(setT1C1On);
  lcd.setCursor(9,1);
  lcd.print("<");  
  }

  if (setpos==2){
  lcd.setCursor(16,1);
  if (setT1C1Off<10){ lcd.print("0");} 
  lcd.print(setT1C1Off);
  lcd.setCursor(18,1);
  lcd.print("<");  
  }

  if (setpos==3){
  lcd.setCursor(7,2);
  if (setT1C2On<10){ lcd.print("0");} 
  lcd.print(setT1C2On);
  lcd.setCursor(9,2);
  lcd.print("<");  
  }
  
  if (setpos==4){
  lcd.setCursor(16,2);
  if (setT1C2On<10){ lcd.print("0");} 
  lcd.print(setT1C2Off);
  lcd.setCursor(18,2);
  lcd.print("<");  
  }

  if (setpos==5){
  lcd.setCursor(8,3);
  if (setT1InvOn<10){ lcd.print("0");} 
  lcd.print(setT1InvOn);
  lcd.setCursor(10,3);
  lcd.print("<");  
  }

  if (setpos==6){
  lcd.setCursor(17,3);
  if (setT1InvOff<10){ lcd.print("0");} 
  lcd.print(setT1InvOff);
  lcd.setCursor(19,3);
  lcd.print("<");  
  }

  if (setpos==7){
  lcd.setCursor(7,1);
  if (setT2C1On<10){ lcd.print("0");} 
  lcd.print(setT2C1On);
  lcd.setCursor(9,1);
  lcd.print("<");  
  }

  if (setpos==8){
  lcd.setCursor(16,1);
  if (setT2C1Off<10){ lcd.print("0");} 
  lcd.print(setT2C1Off);
  lcd.setCursor(18,1);
  lcd.print("<");  
  }

  if (setpos==9){
  lcd.setCursor(7,2);
  if (setT2C2On<10){ lcd.print("0");} 
  lcd.print(setT2C2On);
  lcd.setCursor(9,2);
  lcd.print("<");  
  }
  
  if (setpos==10){
  lcd.setCursor(16,2);
  if (setT2C2On<10){ lcd.print("0");} 
  lcd.print(setT2C2Off);
  lcd.setCursor(18,2);
  lcd.print("<");  
  }

  if (setpos==11){
  lcd.setCursor(8,3);
  if (setT2InvOn<10){ lcd.print("0");} 
  lcd.print(setT2InvOn);
  lcd.setCursor(10,3);
  lcd.print("<");  
  }

  if (setpos==12){
  lcd.setCursor(17,3);
  if (setT2InvOff<10){ lcd.print("0");} 
  lcd.print(setT2InvOff);
  lcd.setCursor(19,3);
  lcd.print("<");  
  }

  if (setpos==13){
  lcd.setCursor(8,1);
  if (setNormInvHrOn<10){ lcd.print("0");} 
  lcd.print(setNormInvHrOn);
  lcd.setCursor(10,1);
  lcd.print("<");  
  }

  if (setpos==14){
  lcd.setCursor(12,1);
  if (setNormInvMinOn<10){ lcd.print("0");} 
  lcd.print(setNormInvMinOn);
  lcd.setCursor(15,1);
  lcd.print(" <   ");  
  }

  if (setpos==15){
  lcd.setCursor(8,2);
  if (setNormInvHrOff<10){ lcd.print("0");} 
  lcd.print(setNormInvHrOff);
  lcd.setCursor(10,2);
  lcd.print("<");  
  }

  if (setpos==16){
  lcd.setCursor(13,2);
  if (setNormInvMinOff<10){ lcd.print("0");} 
  lcd.print(setNormInvMinOff);
  lcd.setCursor(16,2);
  lcd.print(" <  ");  
  }

  if (setpos==17){
  lcd.setCursor(14,1);
  if (setDreset==0){
   lcd.print("NO "); 
  }
  else{
   lcd.print("YES"); 
  }
  lcd.setCursor(19,1);
  lcd.print("<");  
  }

  if (setpos==18){
  lcd.setCursor(4,2);
  if (setDresethr<10){ lcd.print("0");}
  lcd.print(setDresethr);
  lcd.setCursor(7,2);
  lcd.print("<");  
  }

  if (setpos==19){
  lcd.setCursor(10,2);
  if (setDresetmin<10){ lcd.print("0");}
  lcd.print(setDresetmin);
  lcd.setCursor(13,2);
  lcd.print("<  ");  
  }


  switchpress=digitalRead(SW);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    pagelock=0;
    buttonrelease=1;
    }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set Timer 1, Charger 1 On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT1C1On=counter;
   } 

if (setpos==2){
 // set Timer 1, Charger 1 Off
   
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }
    
     setT1C1Off=counter;
   }  

   if (setpos==3){
 // set Timer 1, Charger 2 On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT1C2On=counter;
   }

     if (setpos==4){
 // set set Timer 1, Charger 2 Off
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT1C2Off=counter;
   }

     if (setpos==5){
 // set set Timer 1, Inverter On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT1InvOn=counter;
   }

    if (setpos==6){
 // set Timer 1, Inverter Off
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT1InvOff=counter;
   }

   if (setpos==7){
 // set Timer 2, Charger 1 On
  
   readEncoder();
   
     if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT2C1On=counter;
   }

if (setpos==8){
 // set Timer 2, Charger 1 Off
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT2C1Off=counter;
   }

   if (setpos==9){
 // set Timer 2, Charger 2 On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT2C2On=counter;
   }

     if (setpos==10){
 // set Timer 2, Charger 2 Off
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT2C2Off=counter;
   }

     if (setpos==11){
 // set Timer 2, Inverter On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

    setT2InvOn=counter;
   }

    if (setpos==12){
 // set Timer 2, Inverter Off
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setT2InvOff=counter;
   }

      if (setpos==13){
 // other modes, Inverter On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setNormInvHrOn=counter;
   }

      if (setpos==14){
 // set Timer 2, Inverter Off
  
   readEncoder();
   
    if (counter>59) { 
    counter=59;
    }
    if (counter<0) { 
    counter=0;
    }

    setNormInvMinOn=counter;
   }

      if (setpos==15){
 // other modes, Inverter On
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     setNormInvHrOff=counter;
   }

      if (setpos==16){
 // set Timer 2, Inverter Off
  
   readEncoder();
   
    if (counter>59) { 
    counter=59;
    }
    if (counter<0) { 
    counter=0;
    }

    setNormInvMinOff=counter;
   } 
       if (setpos==17){
 // set daily reset
  
   readEncoder();
   
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setDreset=counter;
   }
 
       if (setpos==18){
 // set daily reset hr
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

    setDresethr=counter;
   }

     if (setpos==19){
 // set daily reset min
   
   readEncoder();
   
     if (counter>59) { 
    counter=59;
    }
    if (counter<0) { 
    counter=0;
    }

     setDresetmin=counter;
   }

 
  if (setpos==20){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 9: Set Timers ");
   
   // The following lines save times
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
   NormInvHrOn=setNormInvHrOn;
   NormInvMinOn=setNormInvMinOn;
   NormInvHrOff=setNormInvHrOff;
   NormInvMinOff=setNormInvMinOff;
   Dreset=setDreset;
   Dresethr=setDresethr;
   Dresetmin=setDresetmin;
   //write times to EEPROM
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
   EEPROM.write(24,NormInvHrOn);
   EEPROM.write(25,NormInvHrOff);
   EEPROM.write(26,Dreset);
   EEPROM.write(27,Dresethr);
   EEPROM.write(28,Dresetmin);
   EEPROM.write(34,NormInvMinOn);
   EEPROM.write(35,NormInvMinOff);
   
   lcd.setCursor(7,2);
   lcd.print("DONE");
   
   delay(3000);
   setpos=0;
   }

}

void Mode10(){
 
  //Set Limits (Mode10)

 if (pagelock==0){
  if (setpos<=3) {
  lcd.setCursor(0,0);
  lcd.print("Mode 10a: Set Limits");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("SOC Min:  % ");
  lcd.setCursor(8,2);
  if(setminimumSOC<10){ lcd.print("0");}
  lcd.print(setminimumSOC);
  lcd.setCursor(12,2);
  lcd.print("Max:");
  lcd.setCursor(16,2);
  lcd.print("   %");
  lcd.setCursor(16,2);
  lcd.print(setmaximumSOC);
  lcd.setCursor(0,3);
  lcd.print("Fan Act Temp:   C   ");
  lcd.setCursor(14,3);
  lcd.print(round(setfanActtemp));  
  }

  if (setpos>3) {
  lcd.setCursor(0,0);
  lcd.print("Mode 10b: Set Limits");
  lcd.setCursor(0,1);
  lcd.print("Mode1 deltaP: +     ");
  lcd.setCursor(15,1);
  lcd.print(settriggerpowermode1*100);
  lcd.setCursor(0,2);
  lcd.print("Mode2 deltaP: -     ");
  lcd.setCursor(15,2);
  lcd.print(settriggerpowermode2*100);
  lcd.setCursor(0,3);
  lcd.print("Vread cal: ");
  lcd.setCursor(11,3);
  lcd.print("    ");
  lcd.setCursor(11,3);
  lcd.print(round(setfineadjustmentbyte-30));
  lcd.setCursor(15,3);
  lcd.print("units");
  }

  pagelock=1;
  }

  if (setpos==1){
  lcd.setCursor(8,2);
  if(setminimumSOC<10){ lcd.print("0");}
  lcd.print(setminimumSOC);
  lcd.setCursor(10,2);
  lcd.print("<");  
  }

  if (setpos==2){
  lcd.setCursor(16,2);
  lcd.print(setmaximumSOC);
  lcd.setCursor(19,2);
  lcd.print("<");  
  }

  if (setpos==3){
  lcd.setCursor(14,3);
  lcd.print(setfanActtemp,0);  
  lcd.setCursor(16,3);  
  lcd.print("<");  
  }

  if (setpos==4){
  lcd.setCursor(15,1);
  lcd.print(settriggerpowermode1*100);
  lcd.setCursor(19,1);
  lcd.print("<");  
  }

  if (setpos==5){
  lcd.setCursor(15,2);
  lcd.print(settriggerpowermode2*100);
  lcd.setCursor(19,2);
  lcd.print("<");  
  }

  if (setpos==6){
  lcd.setCursor(11,3);
  lcd.print(setfineadjustmentbyte-30,0);
  if(setfineadjustmentbyte>20 and setfineadjustmentbyte<29){ lcd.print(" ");}
  if(setfineadjustmentbyte>=30 and setfineadjustmentbyte<40){ lcd.print("  ");}
  lcd.setCursor(15,3);
  lcd.print("<    ");  
  }

  
  switchpress=digitalRead(SW);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    pagelock=0;
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set minimum SOC
  
   readEncoder();
   
    if (counter>29) { 
    counter=29;
    }
    if (counter<3) { 
    counter=3;
    }
    
     setminimumSOC=counter;
   }

if (setpos==2){
 // set month
  
   readEncoder();
   
    if (counter>95) { 
    counter=95;
    }
    if (counter<79) { 
    counter=79;
    }

     setmaximumSOC=counter;
   } 

   if (setpos==3){
 // set year
  
   readEncoder();
   
    if (counter>40) { 
    counter=40;
    }
    if (counter<20) { 
    counter=20;
    }

     setfanActtemp=counter;
   }

     if (setpos==4){
 // set trigger mode 1
  
   readEncoder();

    if (counter>9) { 
    counter=9;
    }
    if (counter<1) { 
    counter=1;
    }

     settriggerpowermode1=counter;
   }

     if (setpos==5){
 // set trigger mode 1
  
   readEncoder();

    if (counter>6) { 
    counter=6;
    }
    if (counter<1) { 
    counter=1;
    }

     settriggerpowermode2=counter;
   }

    if (setpos==6){
 // set voltage readout fine tuning
  
   readEncoder();
    
    if (counter>60) { 
    counter=60;
    }
    if (counter<0) { 
    counter=0;
    }

     setfineadjustmentbyte=counter;
   }
   
  if (setpos==7){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 10: Set Limits ");
   
   // The following lines save limits
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
   
   lcd.setCursor(0,2);
   lcd.print(" DONE...resetting.. ");
   
   delay(3000);
   resetFunc(); 
   
   setpos=0;
   }

}

void Mode11(){

  // Set parameters (Mode11)

  if (pagelock==0){
  if(setpos<=2){
  lcd.setCursor(0,0);
  lcd.print("Mode 11a: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("Chg 1 Amps:      A  ");
  lcd.setCursor(13,2);
  lcd.print(setConeAmps);
  lcd.setCursor(0,3);
  lcd.print("Chg 2 Amps:      A  ");
  lcd.setCursor(13,3);
  lcd.print(setCtwoAmps);
  }
  
  if (setpos>2 and setpos<=5) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11b: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Batt Type:          ");
  lcd.setCursor(11,1);
  if (settypeofbatt==0) {
  lcd.print("LiPo  ");
  }
  if (settypeofbatt==1) {
  lcd.print("LiFePo");
  }
  if (settypeofbatt==2) {
  lcd.print("NMC   ");
  }
  lcd.setCursor(0,2);
  lcd.print("Show Power:         ");
  lcd.setCursor(12,2);
  if (setshowpower==0) {
  lcd.print("Average ");
  }
  if (setshowpower==1) {
  lcd.print("Real    ");
  }
  lcd.setCursor(0,3);
  lcd.print("Invert Readout:     ");
  lcd.setCursor(16,3);
  if (setinvertreadout==1) {
  lcd.print("YES ");
  }
  if (setinvertreadout==0) {
  lcd.print("NO  ");
  }
  }
  
  if (setpos>5 and setpos<=8) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11c: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Inv Curr Sensor:    A");
  lcd.setCursor(16,1);
  lcd.print(setinvcurrsens);
  lcd.setCursor(0,2);
  lcd.print("Use Wifi:           ");
  lcd.setCursor(10,2);
  if (setwifiavail==1) {
  lcd.print("YES ");
  }
  if (setwifiavail==0) {
  lcd.print("NO  ");
  }
  lcd.setCursor(0,3);
  lcd.print("Dyn. Trigger:       ");
  lcd.setCursor(14,3);
  if (setdyntrigger==1) {
  lcd.print("YES ");
  }
  if (setdyntrigger==0) {
  lcd.print("NO  ");
  }
  }
  
  if (setpos>8 and setpos<=11) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11d: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Meter Type:         ");
  lcd.setCursor(12,1);
  if (settypeofmeter==0) {
  lcd.print("SDM120");
  }
  if (settypeofmeter==1) {
  lcd.print("SDM630");
  }
  if (settypeofmeter==2) {
  lcd.print("X835  ");
  }
  lcd.setCursor(0,2);
  lcd.print("Act Chg Phase:      ");
  lcd.setCursor(14,2);
  if (setactivephase==1) {
  lcd.print("Ph 1 ");
  }
  if (setactivephase==2) {
  lcd.print("Ph 2 ");
  }
  if (setactivephase==3) {
  lcd.print("Ph 3 ");
  }
  lcd.setCursor(0,3);
  lcd.print("Drift Fact:         ");
  lcd.setCursor(12,3);
  lcd.print(setcoulcorrbyte);
  }

   if (setpos>11 and setpos<=14) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11e: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Use 3rd Chgr:       ");
  lcd.setCursor(14,1);
  if (setCthree==0) {
  lcd.print("NO  ");
  }
  if (setCthree==1) {
  lcd.print("Ph 1");
  }
  if (setCthree==2) {
  lcd.print("Ph 2");
  }
  if (setCthree==3) {
  lcd.print("Ph 3");
  }
  lcd.setCursor(0,2);
  lcd.print("Chg 3 Amps:      A  ");
  lcd.setCursor(13,2);
  lcd.print(setCthreeAmps);
  lcd.setCursor(0,3);
  lcd.print("Export Limiter:     ");
  lcd.setCursor(15,3);
  if (setCthree==0) {
  lcd.print("NO ");
  }
  if (setCthree==1) {
  lcd.print("YES");
  }
  }

  if (setpos>14) {
  lcd.setCursor(0,0);
  lcd.print("Mode 11f: Set Param.");
  lcd.setCursor(0,1);
  lcd.print("Use Ext. WDT:       ");
  lcd.setCursor(15,1);
  if (setExtWDT==0) {
  lcd.print("NO  ");
  }
  if (setExtWDT==1) {
  lcd.print("YES ");
  }
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }
  
  pagelock=1;
  }
  

  if (setpos==1){
  lcd.setCursor(13,2);
  if (setConeAmps<10){ lcd.print("0");}
  lcd.print(setConeAmps);
  lcd.setCursor(17,2);
  lcd.print("<");  
  }
  
  if (setpos==2){
  lcd.setCursor(13,3);
  if (setCtwoAmps<10){ lcd.print("0");}  
  lcd.print(setCtwoAmps);
  lcd.setCursor(17,3);
  lcd.print("<");  
  }
  
  if (setpos==3){
  lcd.setCursor(11,1);
  if (settypeofbatt==0) {
  lcd.print("LiPo  ");
  }
  if (settypeofbatt==1) {
  lcd.print("LiFePo");
  }
  if (settypeofbatt==2) {
  lcd.print("NMC   ");
  }
  lcd.setCursor(19,1);
  lcd.print("<");  
  }

  if (setpos==4){
  lcd.setCursor(12,2);
  if (setshowpower==0) {
  lcd.print("Average ");
  }
  if (setshowpower==1) {
  lcd.print("Real    ");
  }
  lcd.setCursor(19,2);
  lcd.print("<");  
  }

  if (setpos==5){
  lcd.setCursor(16,3);
  if (setinvertreadout==1) {
  lcd.print("YES ");
  }
  if (setinvertreadout==0) {
  lcd.print("NO  ");
  }
  lcd.setCursor(19,3);
  lcd.print("<");  
  }

  if (setpos==6){
  lcd.setCursor(16,1);
  lcd.print(setinvcurrsens);
  lcd.setCursor(19,1);
  lcd.print("<");  
  }

  if (setpos==7){ 
  lcd.setCursor(10,2);
  if (setwifiavail==1) {
  lcd.print("YES ");
  }
  if (setwifiavail==0) {
  lcd.print("NO  ");
  }
  lcd.setCursor(15,2);
  lcd.print("<");  
  }

  if (setpos==8){
  lcd.setCursor(14,3);
  if (setdyntrigger==1) {
  lcd.print("YES ");
  }
  if (setdyntrigger==0) {
  lcd.print("NO  ");
  }
  lcd.setCursor(19,3);
  lcd.print("<");  
  }

  if (setpos==9){
  lcd.setCursor(12,1);
  
  if (settypeofmeter==4){ settypeofmeter=9; }
  
  if (settypeofmeter==0) {
  lcd.print("SDM120");
  }
  if (settypeofmeter==1) {
  lcd.print("SDM630");
  }
  if (settypeofmeter==2) {
  lcd.print("X835  ");
  }
  if (settypeofmeter==9) {
  lcd.print("NONE  ");
  }
  lcd.setCursor(19,1);
  lcd.print("<");  
  }

  if (setpos==10){
  lcd.setCursor(14,2);
  if (setactivephase==1) {
  lcd.print("Ph 1 ");
  }
  if (setactivephase==2) {
  lcd.print("Ph 2 ");
  }
  if (setactivephase==3) {
  lcd.print("Ph 3 ");
  }
  lcd.setCursor(19,2);
  lcd.print("<");  
  }

  if (setpos==11){
  lcd.setCursor(12,3);
  if (setcoulcorrbyte==161 or setcoulcorrbyte==69){
    lcd.print("Rst "); 
  }
  else{    
    lcd.print(setcoulcorrbyte);
  }
  lcd.setCursor(18,3);
  lcd.print("< ");  
  }
  
  if (setpos==12){
  lcd.setCursor(14,1);
  if (setCthree==0) {
  lcd.print("NO  ");
  }
  if (setCthree==1) {
  lcd.print("Ph1 ");
  }
  if (setCthree==2) {
  lcd.print("Ph2 ");
  }
  if (setCthree==3) {
  lcd.print("Ph3 ");
  }
  lcd.setCursor(19,1);
  lcd.print("<");  
  }
  if (setpos==13){
  lcd.setCursor(13,2);
  if (setCthreeAmps<10){ lcd.print("0");}
  lcd.print(setCthreeAmps);
  lcd.setCursor(17,2);
  lcd.print("<");  
  }

  if (setpos==14){
  lcd.setCursor(15,3);
  if (setLimiter==1) {
  lcd.print("YES");
  }
  if (setLimiter==0) {
  lcd.print("NO ");
  }
  lcd.setCursor(19,3);
  lcd.print("<");  
  }

  if (setpos==15){
  lcd.setCursor(15,1);
  if (setExtWDT==1) {
  lcd.print("YES");
  }
  if (setExtWDT==0) {
  lcd.print("NO ");
  }
  lcd.setCursor(19,1);
  lcd.print("<");  
  }


  switchpress=digitalRead(SW);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    pagelock=0;
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set charger 1 amps
  
   readEncoder();
   
    if (counter>45) { 
    counter=45;
    }
    if (counter<0) { 
    counter=1;
    }

     setConeAmps=counter;
   } 

if (setpos==2){
 // set charger 2 amps
  
   readEncoder();
   
    if (counter>45) { 
    counter=45;
    }
    if (counter<0) { 
    counter=1;
    }

     setCtwoAmps=counter;
   }   

   if (setpos==3){
 // set batterytype
  
   readEncoder();
     
    if (counter>2) { 
    counter=0;
    }
    if (counter<0) { 
    counter=2;
    }

     settypeofbatt=counter;
   } 

        if (setpos==4){
 // if readout of power shall be inverted
  
   readEncoder();
     
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setshowpower=counter;
   } 
   

     if (setpos==5){
 // if readout of power shall be inverted
  
   readEncoder();
   
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setinvertreadout=counter;
   } 

   
     if (setpos==6){
 // A meter module max Amps
  
   readEncoder();
    
    if (counter>30) { 
    counter=30;
    }
    if (counter<0) { 
    counter=0;
    }

     setinvcurrsens=counter;
   } 

     if (setpos==7){
 // if wifi shall be used
  
   readEncoder();
    
    if (counter>1) {  
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setwifiavail=counter;
   } 

        if (setpos==8){
 // if dyn trigger shall be inverted
  
   readEncoder();
    
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setdyntrigger=counter;
   } 

     if (setpos==9){
 // active phase shall be used
  
   readEncoder();
   
    if (counter>2) { 
    counter=2;
    }
    if (counter<0) { 
    counter=0;
    }

     settypeofmeter=counter;
   } 

     if (setpos==10){
 // active phase shall be used
  
   readEncoder();
   
    if (counter>3) { 
    counter=3;
    }
    if (counter<1) { 
    counter=1;
    }

     setactivephase=counter;
   } 

        if (setpos==11){
 // active phase shall be used
  
   readEncoder();
   
    if (counter>161) { 
    counter=161;
    }
    if (counter<69) { 
    counter=69;
    }

     setcoulcorrbyte=counter;
   } 
   
        if (setpos==12){
 // active phase shall be used
  
   readEncoder();
   
    if (counter>3) { 
    counter=3;
    }
    if (counter<0) { 
    counter=0;
    }

     setCthree=counter;
   } 

        if (setpos==13){
  
   readEncoder();
   
    if (counter>45) { 
    counter=45;
    }
    if (counter<1) { 
    counter=1;
    }

     setCthreeAmps=counter;
   } 

 if (setpos==14){
  
   readEncoder();
   
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setLimiter=counter;
   } 

 if (setpos==15){
  
   readEncoder();
   
    if (counter>1) { 
    counter=0;
    }
    if (counter<0) { 
    counter=1;
    }

     setExtWDT=counter;
   } 


  if (setpos==16){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 11: Set Param. ");

   CheckChargerthree();
   
   // The following lines save parameters
   ConeAmps=setConeAmps;
   CtwoAmps=setCtwoAmps;
   typeofbatt=settypeofbatt;
   invertreadout=setinvertreadout;
   showpower=setshowpower;
   invcurrsens=setinvcurrsens;
   wifiavail=setwifiavail;
   dyntrigger=setdyntrigger;
   typeofmeter=settypeofmeter;
   activephase=setactivephase;
   coulcorrbyte=setcoulcorrbyte;
   Cthree=setCthree;
   CthreeAmps=setCthreeAmps;
   Limiter=setLimiter;
   ExtWDT=setExtWDT;
   //write limits to EEPROM
   EEPROM.write(4,typeofbatt);
   EEPROM.write(5,ConeAmps);
   EEPROM.write(6,CtwoAmps);
   EEPROM.write(21,showpower);
   EEPROM.write(22,invertreadout);
   EEPROM.write(29,invcurrsens);
   EEPROM.write(30,wifiavail);
   EEPROM.write(31,dyntrigger);
   EEPROM.write(32,activephase);
   EEPROM.write(33,typeofmeter);
   EEPROM.write(36,coulcorrbyte);
   EEPROM.write(37,Cthree);
   EEPROM.write(38,CthreeAmps);
   EEPROM.write(39,Limiter);  
   EEPROM.write(40,ExtWDT);    
  
   
   lcd.setCursor(0,2);
   lcd.print(" DONE...resetting.. ");
   
   delay(3000);
   resetFunc(); 
   setpos=0;
   }

}

void Mode12(){
  
  // Set Clock Mode (Mode12)

 if (pagelock==0){
  lcd.setCursor(0,0);
  lcd.print("Mode 12: Set Clock   ");
  lcd.setCursor(0,1);
  lcd.print("Press Knob to switch");
  lcd.setCursor(0,2);
  lcd.print("Date: ");
  lcd.setCursor(6,2);
  lcd.print("  ");
  lcd.setCursor(6,2);
  if (setday<10){ lcd.print("0");}
  lcd.print(setday);
  lcd.setCursor(8,2);
  lcd.print(".");
  lcd.setCursor(9,2);
  lcd.print("  ");
  lcd.setCursor(9,2);
  if (setmonth<10){ lcd.print("0");}
  lcd.print(setmonth);
  lcd.setCursor(11,2);
  lcd.print(".");
  lcd.setCursor(12,2);
  lcd.print("        ");
  lcd.setCursor(12,2);
  lcd.print(setyear);
  lcd.setCursor(16,2);  
  lcd.print(" ");  
  lcd.setCursor(0,3);
  lcd.print("Time: ");
  lcd.setCursor(6,3);
  lcd.print("  ");
  lcd.setCursor(6,3);
  if (sethour<10){ lcd.print("0");}
  lcd.print(sethour);
  lcd.setCursor(8,3);
  lcd.print(".");
  lcd.setCursor(9,3);
  lcd.print("  ");
  lcd.setCursor(9,3);
  if (setminute<10){ lcd.print("0");}  
  lcd.print(setminute);
  lcd.setCursor(11,3);
  lcd.print(".");
  lcd.setCursor(12,3);
  lcd.print("00      ");

  pagelock=1;
 }

  if (setpos==1){
  lcd.setCursor(6,2);
  if (setday<10){ lcd.print("0");}  
  lcd.print(setday);
  lcd.setCursor(8,2);
  lcd.print("<");  
  }

  if (setpos==2){
  lcd.setCursor(9,2);
  if (setmonth<10){ lcd.print("0");}
  lcd.print(setmonth);
  lcd.setCursor(11,2);  
  lcd.print("<");  
  }
  
  if (setpos==3){
  lcd.setCursor(12,2);
  lcd.print(setyear);  
  lcd.setCursor(16,2);  
  lcd.print("<");  
  }

  if (setpos==4){ 
  lcd.setCursor(6,3);
  if (sethour<10){ lcd.print("0");}
  lcd.print(sethour);  
  lcd.setCursor(8,3);
  lcd.print("<");  
  }
  
  if (setpos==5){
  lcd.setCursor(9,3);
  if (setminute<10){ lcd.print("0");}  
  lcd.print(setminute);  
  lcd.setCursor(11,3);    
  lcd.print("<");  
  }

  
  switchpress=digitalRead(SW);
   if (switchpress==0 and buttonrelease==0){
    setpos++; 
    pagelock=0;
    buttonrelease=1; 
  }
  if (switchpress==1){
    buttonrelease=0;
  }

if (setpos==1){
 // set day
  
   readEncoder();
   
    if (counter>31) { 
    counter=31;
    }
    if (counter<1) { 
    counter=1;
    }

     setday=counter;
   }

if (setpos==2){
 // set month
  
   readEncoder();

    if (counter>12) { 
    counter=12;
    }
    if (counter<1) { 
    counter=1;
    }

     setmonth=counter;
   } 

   if (setpos==3){
 // set year
  
   readEncoder();

    if (counter>2040) { 
    counter=2040;
    }
    if (counter<2020) { 
    counter=2020;
    }

     setyear=counter;
   } 

   if (setpos==4){
   // set hour
  
   readEncoder();
   
    if (counter>23) { 
    counter=23;
    }
    if (counter<0) { 
    counter=0;
    }

     sethour=counter;
   }

   if (setpos==5){
 // set minute
  
   readEncoder();

    if (counter>59) { 
    counter=59;
    }
    if (counter<0) { 
    counter=0;
    }

     setminute=counter;
   }

   if (setpos==6){
   // finish
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Mode 12: Set Time   ");
   
   clock.setDateTime(setyear,setmonth,setday,sethour,setminute, 0);     // Set the time to  (24hr format)
    
   lcd.setCursor(7,2);
   lcd.print("DONE");
  
   delay(3000);
   setpos=0;
   }

}

void resetExtWDT() {

 if (millis()>=(ExtWDTstartmillis+ExtWDTinterval)) {
    digitalWrite(ExtWDTpin,HIGH);
    delay(50);
    digitalWrite(ExtWDTpin,LOW);
    ExtWDTstartmillis=millis();
 }

} 

void simpleWDTreset() {

    digitalWrite(ExtWDTpin,HIGH);
    delay(50);
    digitalWrite(ExtWDTpin,LOW);
    
}

void displaymode() {

 if (mode==1){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);
  display.println(F("  1  ")); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Excess"));
  display.println(F("Metered"));
  display.display();
 }

  if (mode==2){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0); 
  display.println(F("  2  "));   
  display.setTextSize(2); 
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Timer"));
  display.println(F("Based"));
  display.display();
 } 

 if (mode==3){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);
  display.println(F("  3  ")); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Charger"));
  display.println(F("One ON"));
  display.display();
 }

  if (mode==4){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);
  display.println(F("  4  ")); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Charger"));
  display.println(F("Two ON"));
  display.display();
 }
 

  if (mode==5){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);
  display.println(F("  5  ")); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Charger"));
  display.println(F("Three ON"));
  display.display();
 }

  
  if (mode==6){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0); 
  display.println(F("  6  ")); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Chargers"));
  display.println(F("All ON"));
  display.display();
 }

 /*
  if (mode==7){
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);  
  display.println(F("  7  ")); 
  display.setTextSize(2); 
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Charger"));
  display.println(F("One+Two ON"));
  display.display();
 }
 */

 
 if (mode==8){
  display.clearDisplay();
  display.setTextSize(4); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0); 
  display.println(F("  8  ")); 
  display.setTextSize(2); 
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Inverter"));
  display.println(F("Discharge"));
  display.display();
 }

  if (mode==9){
     display.clearDisplay();
     display.setTextSize(4); 
     display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
     display.setCursor(0,0); 
     display.println(F("  9  ")); 
     display.setTextSize(2); 
     display.setTextColor(SSD1306_WHITE);
     display.println(F("Setup"));
     display.println(F("Timers"));
     display.display();
    }

  if (mode==10){
     display.clearDisplay();
     display.setTextSize(4);  
     display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
     display.setCursor(0,0); 
     display.println(F(" 10  ")); 
     display.setTextSize(2); 
     display.setTextColor(SSD1306_WHITE);
     display.println(F("Setup"));
     display.println(F("Limits"));
     display.display();
    }

  if (mode==11){
     display.clearDisplay();
     display.setTextSize(4); 
     display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
     display.setCursor(0,0);
     display.println(F(" 11  ")); 
     display.setTextSize(2);  
     display.setTextColor(SSD1306_WHITE);
     display.println(F("Setup"));
     display.println(F("Parameters"));
     display.display();
    }
 
   if (mode==12){
     display.clearDisplay();
     display.setTextSize(4);   
     display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
     display.setCursor(0,0); 
     display.println(F(" 12  ")); 
     display.setTextSize(2);
     display.setTextColor(SSD1306_WHITE);
     display.println(F("Setup"));
     display.println(F("Clock"));
     display.display();
   }

 }

void readEncoder(){
  
  pinAStateHigh = digitalRead(CLK);
  pinBStateHigh = digitalRead(DT);
  // Pin A changed? What direction?
  if (pinAStateHigh != oldPinAStateHigh) {

    // Pin A is HIGH
    if (pinAStateHigh) {
      // Pin B is LOW
      if (!pinBStateHigh) {
        // Clockwise
        virtualPosition++;
      }
      else {
        // Anticlockwise
        virtualPosition--;
      }
    }   
    else {
      // Pin A just went low and B is HIGH
      if (pinBStateHigh) {
        // Clockwise
        virtualPosition++;
      }
      else {
        // Anticlockwise
        virtualPosition--;
      } 
    }  
   
 
    // Keep track of state of Pin A
    oldPinAStateHigh = pinAStateHigh;
  }

  // Pin B changed? What direction?
  if (pinBStateHigh != oldPinBStateHigh) {

    if (pinBStateHigh) {
      if (pinAStateHigh) {
        virtualPosition++;
      }
      else {
        virtualPosition--;
      }
    } 
    else {
      // Pin B just went low
      if (!pinAStateHigh) {
        virtualPosition++;
      }
      else {
        virtualPosition--;
      }
    }
    oldPinBStateHigh = pinBStateHigh;
  }

  // If the current rotary switch position has changed then update everything
  
 if (virtualPosition != lastCount) {

  if(encoderblock==0){ 
   if (virtualPosition>lastCount){
    counter++;
    encoderblock=1;
    lastencodermillis=millis();
   }
    if (virtualPosition<lastCount){
    counter--;
    encoderblock=1;
    lastencodermillis=millis();
   }
  }
  if (millis()>(lastencodermillis+150)){ 
    lastCount = virtualPosition;
    encoderblock=0; 
  }
  
 }
}

void LEDsignals() {

  chargercounter++;
  invertercounter++;
  greencounter++;

  chargersnbr=0;
  if (chargerone==1){
  chargersnbr++;
  }
  if (chargertwo==1){
  chargersnbr++;
  }
  if (chargerthree==1){
  chargersnbr++;
  }
  
  if (mode<=8) {             // system showing ON only when in a running mode
  digitalWrite(greenpin,HIGH);     
  }
  else {
  digitalWrite(greenpin,LOW);
  }

  
  
  if (chargersnbr==3) {
   digitalWrite(yellowpin,HIGH); 
   chargercounter=0;
  }

  if (chargersnbr==1) {     //1 sec blink when only C1
   if (chargercounter==1) {
   digitalWrite(yellowpin,HIGH); 
   }
   else {
   digitalWrite(yellowpin,LOW);
   chargercounter=0; 
   }
  }
  
if (chargersnbr==2) {     //2 sec blink when only C2   
   if (chargercounter==1 or chargercounter==2) {
   digitalWrite(yellowpin,HIGH); 
   }
   else {
   digitalWrite(yellowpin,LOW);
   chargercounter=0; 
   }
  }

  if (chargersnbr==0) {
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

connectCounter=0;

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
if (chargerthree==1){
chargerthreelog=holdvolt*CthreeAmps;
}
else{
chargerthreelog=0;  
}

bothchargerslog=current*battvoltage;
averageSOC=round(averageSOCpool/powercounter);

realpower=realpowerpri+realpowerthree+averagepowerpool2;

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
  ACPWdata.print(mode);
  ACPWdata.print(",");
  ACPWdata.print(battvoltage);
  ACPWdata.print(",");
  ACPWdata.print(current);
  ACPWdata.print(",");
  ACPWdata.print(averageSOC);
  ACPWdata.print(",");
  ACPWdata.print(SOCCoul);
  ACPWdata.print(",");
  ACPWdata.print(realpower);
  ACPWdata.print(",");
  ACPWdata.print(averagepowerpoolT);
  ACPWdata.print(",");  
  ACPWdata.print(averagepowerpool1);
  ACPWdata.print(",");
  ACPWdata.print(averagepowerpool2);
  ACPWdata.print(",");
  ACPWdata.print(averagepowerpool3);
  ACPWdata.print(",");
  ACPWdata.print(chargeronelog);
  ACPWdata.print(",");
  ACPWdata.print(chargertwolog);
  ACPWdata.print(",");
  ACPWdata.print(chargerthreelog);
  ACPWdata.print(",");
  ACPWdata.print(bothchargerslog);
  ACPWdata.print(",");
  ACPWdata.print(averageinvwatts);
  ACPWdata.print(",");
  ACPWdata.print(tempC);
  ACPWdata.print(",");
  ACPWdata.print(humidity);
  ACPWdata.print(",");
  ACPWdata.print(diffpower);
  ACPWdata.print(",");
  ACPWdata.print(probetemp1);
  ACPWdata.print(","); 
  ACPWdata.print(probetemp2);
  ACPWdata.print(","); 
  ACPWdata.print(probetemp3);
  ACPWdata.print(","); 
  ACPWdata.print(coulcorr);
  ACPWdata.print(","); 
  ACPWdata.print(battcapacity);
  ACPWdata.print(","); 
  ACPWdata.print(millis()/10000);
  ACPWdata.println("");
  ACPWdata.close();
}
}

}

void startBlynk() {
  
  if (linkState == 0) {  
    disconnects++;
    if (disconnects>9999){
      disconnects=9999;                             
    }
    wifi.setDHCP(1, 1, 1);       //Enable dhcp in station mode and save in flash of esp8266
    Blynk.config(wifi, auth, "blynk-cloud.com", 8080); 
    if (ExtWDT==1){
      simpleWDTreset();
    }
    if (Blynk.connectWiFi(ssid, pass)) {
      if (ExtWDT==1){
        simpleWDTreset();
      }
      Blynk.connect();
        if (ExtWDT==1){
           simpleWDTreset();
        }
      if (Blynk.connected()){
      linkState = 1;
      }
    }
    if (ExtWDT==1){
      simpleWDTreset();
    }                                     //Blynk startup done
  }


}

void BlynkDataTransmit(){

 if (Blynk.connected()){
  
 if (sendpart==0){
    
 Blynk.virtualWrite(V5, millis() / 10000);
 Blynk.virtualWrite(V4, mode);
 Blynk.virtualWrite(V6, battvoltage);
 Blynk.virtualWrite(V7, SOCDisplay);
 Blynk.virtualWrite(V10, tempC);
 Blynk.virtualWrite(V17, current);
 Blynk.virtualWrite(V21,round(CCwatthours));
 Blynk.virtualWrite(V22,round(CDwatthours));
 
 sendprobetemp1=probetemp1+100;
 Blynk.virtualWrite(V16, round(sendprobetemp1)); 

 if (inverter==1){
 led4.setValue(255);
 }
 else {
 led4.setValue(28); 
 }


}

if (sendpart==1){

if (chargerone==1){
 led2.setValue(255);
 }
 else {
 led2.setValue(28); 
 }
 if (chargertwo==1){
 led3.setValue(255);
 }
 else {
 led3.setValue(28); 
 }
 if (chargerthree==1){
 led6.setValue(255);
 }
 else {
 led6.setValue(28); 
 }

 
 Blynk.virtualWrite(V5, millis() / 10000);
 Blynk.virtualWrite(V8, round(averagepowerpoolT));
 Blynk.virtualWrite(V9, round(measuredpowerT));
 Blynk.virtualWrite(V14, round(measuredpower1));
 Blynk.virtualWrite(V11, round(measuredpower2));
 Blynk.virtualWrite(V12, round(measuredpower3));
 
 sendprobetemp2=probetemp2+200;
 Blynk.virtualWrite(V16, round(sendprobetemp2)); 
 
}

if (sendpart==2){
 
 Blynk.virtualWrite(V5, millis() / 10000);
 Blynk.virtualWrite(V17, current);
 Blynk.virtualWrite(V13, round(inverterwatts));
 Blynk.virtualWrite(V20, coulcorr);
 Blynk.virtualWrite(V19, battcapacity);
 Blynk.virtualWrite(V18, cycles);
 
 if (chargerlock==1){
 led1.setValue(255);
 }
 else {
 led1.setValue(28); 
 }
 if (Dtest==1){
 led5.setValue(255);
 }
 else {
 led5.setValue(28); 
 }
 
   sendprobetemp3=probetemp3+300;
   Blynk.virtualWrite(V16, round(sendprobetemp3)); 
 
}

sendpart++;
if (sendpart>2){
  sendpart=0;
}

}
 
}




void  getTempprobes(){

  proberead1=analogRead(Probe1);
  proberead2=analogRead(Probe2);
  proberead3=analogRead(Probe3);

  // Moving Average
   totalT1 = totalT1-T1readings[readIndex];
   T1readings[readIndex]=proberead1;
   totalT1 = totalT1+T1readings[readIndex];
   totalT2 = totalT2-T2readings[readIndex];
   T2readings[readIndex]=proberead2;
   totalT2 = totalT2+T2readings[readIndex];
   totalT3 = totalT3-T3readings[readIndex];
   T3readings[readIndex]=proberead3;
   totalT3 = totalT3+T3readings[readIndex];
   
   readIndex++;

   if (readIndex>=numReadings){
     readIndex = 0;
   }

    totalT1a = totalT1/numReadings;
    totalT2a = totalT2/numReadings;
    totalT3a = totalT3/numReadings;

  probetemp1=((totalT1a-155)*0.125);
  probetemp2=((totalT2a-155)*0.125);
  probetemp3=((totalT3a-155)*0.125);
  
}


void Currentaquisition (){

if (inverter==1 and mode<9){

  oldinverterwatts=inverterwatts;

  inverterwatts=(current*battvoltage)-20;

  diffinverterwatts=inverterwatts-oldinverterwatts;
  if (diffinverterwatts<0){ 
  diffinverterwatts=diffinverterwatts*-1;
  }
  if (diffinverterwatts>400){
  SOCfreeze=SOCfreeze+10;  
  } 
 

}
else{
inverterwatts=0;
}

averageinvwattspool=averageinvwattspool+inverterwatts;
averageinvwatts=averageinvwattspool/invwattcounter;
invwattcounter++;

}



void get12Vsense(){

  Vvalue=analogRead(Vsense);

  if (Vvalue>200){
    Vpresent=1;
  }
  else{
    Vpresent=0;
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

void CheckChargerthree(){

 if (Cthree==activephase){
    Cthree=0;
  }

}

void Activatechargerthree(){

if (Cthree>=1){

    if (chargerthree==1) {
    realpowerthree=averagepowerpoolS+(battvoltage*CthreeAmps);
    }
    if (chargerthree==0) {
    realpowerthree=averagepowerpoolS;
    }
 

   if (realpowerthree<=(((triggerpowermode1/2)*10)+(battvoltage*CthreeAmps))) {
     digitalWrite(relayPin6,HIGH);
     chargerthree=0;
   } 
   if (realpowerthree>(((triggerpowermode1/2)*10)+(battvoltage*CthreeAmps))) {
     if (chargerlock==0){     
       digitalWrite(relayPin6,LOW);
       chargerthree=1;
     }
     else{
       digitalWrite(relayPin6,HIGH);
       chargerthree=0;
     }
   }
}
else{
  digitalWrite(relayPin6,HIGH);
  chargerthree=0;
}
  
}

void Powercalculations(){

    averagepower=averagepower+measuredpower;
    averagepowerpool=averagepower/powercounter;
    averagepowerS=averagepowerS+measuredpowerS;
    averagepowerpoolS=averagepowerS/powercounter;
    
    averagepowerpoolT=averagepowerpool+averagepowerpoolS;
    measuredpowerT=measuredpower+measuredpowerS;   
    
    averagepower1=averagepower1+measuredpower1;
    averagepowerpool1=averagepower1/powercounter;
    averagepower2=averagepower2+measuredpower2;
    averagepowerpool2=averagepower2/powercounter;
    averagepower3=averagepower3+measuredpower3;
    averagepowerpool3=averagepower3/powercounter;
    
    powercounter++;
       
    if (powercounter>15){
    
    if (dyntrigger==1 and mode==1){
      if (maximumpower<measuredpower){
        maximumpower=measuredpower;
      }
      if (minimumpower>measuredpower){
        minimumpower=measuredpower;
      }
      diffpower=maximumpower-minimumpower;
       if (powercounter>=relaycycletime) {
         if (diffpower<=2200){
         dynlevel=0;
         }
         if (diffpower>2200){
         dynlevel=15;
         }
         if (diffpower>3200){
         dynlevel=30;
         }
     
       }
    }

    }   
}

void getCycles(){

 wattscounter++;

 if (watts<0){
  Cwatts=watts*-1;
  averagewatts=averagewatts+Cwatts;
 }
  
 if (millis()>=(wattsstartmillis+wattsinterval)){
  wattsstartmillis=millis();
  averagewatts=(averagewatts/wattscounter)/(3600000/wattsinterval); 
  chgwatthours=chgwatthours+averagewatts;
  averagewatts=0;
  wattscounter=0;
 }

 if (chgwatthours>=battcapacity){
  cycles++;
  cycleshibyte=0;
  cycleslobyte=0;
  chgwatthours=chgwatthours-battcapacity;
  extractor=cycles;
    while (extractor>256){
      cycleshibyte++;
      extractor=extractor-256;
    }
    cycleslobyte=extractor;
    EEPROM.write(41,cycleslobyte);
    EEPROM.write(42,cycleshibyte);
    delay(50);

    if (capsave==1 and watthoursTot>10){ 
      if (watthoursTot<checkshunt-2000){
         watthoursTot=checkshunt;
         coulcorrarm=1;
         coulcorr=1.00;
     }
     else{
      if (watthoursTot>=(checkshunt+2000)){
         watthoursTot=checkshunt;
         coulcorrarm=1;
         coulcorr=1.00;
      }
     } 
   
          battcapacityHibyte=0;
          battcapacity=watthoursTot;           // saving estimated battery capacity as new battery capacity
          extractor=battcapacity;    
          while (extractor>256){
            battcapacityHibyte++;
            extractor=extractor-256;
          }
        battcapacityLobyte=extractor;
    EEPROM.write(43,battcapacityLobyte);
    EEPROM.write(44,battcapacityHibyte);
    delay(50);
    capsave=0;
    watthoursTot=0;
    blockdischargetest=0;
    Cbattcapacity=battcapacity;
    }
  }

}

void getDischargeData(){

 if (coulcorrarm==0 and Dtest==1){
 
 dwattcounter++;

 if (watts>=0){
  daveragewatts=daveragewatts+watts;
  daverageAmps=daverageAmps+current;
 }
  
 if (millis()>=(dwattsstartmillis+wattsinterval)){  
  dwattsstartmillis=millis();
  oldActWh=ActWh;
  daveragewatts=(daveragewatts/dwattcounter)/(3600000/wattsinterval); 
  ActWh=ActWh+daveragewatts;
  daverageAmps=(daverageAmps/dwattcounter)/(3600000/wattsinterval); 
  ActAh=ActAh+daverageAmps;
    if (battSOC<=10){
      EndSOC=battSOC;
    }
    else{
      EndSOC=SOCCoul;  
    }  
  daverageAmps=0; 
  daveragewatts=0;
  dwattcounter=0;
 }

 if (StartSOC<=80){
   capsave=0;
 }
 else{
   diffSOC=StartSOC-EndSOC;
 }
 
 if(diffSOC>89){
  watthoursTot=(ActWh/diffSOC)*100;
  watthoursTot=watthoursTot*1.02;                 // correct for missing 2% due to safety on bottom of curve
  watthoursTot=round(watthoursTot);
  AhoursTot=(ActAh/diffSOC)*100;
  AhoursTot=AhoursTot*1.03;                       // correct for missing 3% due to safety on bottom of curve
  AhoursTot=round(AhoursTot);
  capsave=1;
 }

 if(ActWh<=(oldActWh+5)){
  Dhoming=0;
 }
 else{
  Dhoming=1;
 }

 }
  
}

void Coulombmeter(){

 cwattscounter++;

 if (watts<0){                             // charge
  CCwatts=watts*-1;
  Caveragewatts=Caveragewatts+CCwatts;
 }
 else {                                    // discharge
  CDwatts=watts*coulcorr;
  Daveragewatts=Daveragewatts+CDwatts;
 }
  
 if (millis()>=(coulombstartmillis+wattsinterval)){
  coulombstartmillis=millis();
  Caveragewatts=(Caveragewatts/cwattscounter)/(3600000/wattsinterval); 
  Caveragewatts=Caveragewatts;                                  // correct for cell energy loss
  CCwatthours=CCwatthours+Caveragewatts;
  Daveragewatts=(Daveragewatts/cwattscounter)/(3600000/wattsinterval); 
  CDwatthours=CDwatthours+Daveragewatts;

  if (coulstart==1){
     CoulombPointer=CoulombPointer+(Caveragewatts-Daveragewatts);
     if (CoulombPointer<0){
        CoulombPointer=0;
     }
     if (CoulombPointer>Cbattcapacity){
        CoulombPointer=Cbattcapacity;
     }
     SOCCoul=(CoulombPointer/Cbattcapacity)*100;
  }   
  
  Daveragewatts=0; 
  Caveragewatts=0;
  cwattscounter=0;
  
 }

Calibratecoulomb();

}


void Calibratecoulomb(){

SOCdiff=SOCCoul-battSOC;
if (SOCdiff<0){
  SOCdiff=SOCdiff*-1;
}

if (SOCfreeze==0){
 if (battSOC==95 and inverter==0 and chargersnbr<=1){
   if (coulcorrarm==0){
      Dtest=1;
   }
   if (SOCdiff>=3){
      SOCCoul=battSOC;
      CoulombPointer=(Cbattcapacity/100)*SOCCoul;
      Meteraccur=2;
   }
 }
} 

if (battSOC>50){
  calistop=0;
}

 if (battSOC<=18 and battSOC>16){
    if (SOCdiff<=2){
      Meteraccur=1;
    }
    if (SOCdiff>2 and SOCdiff<=4){
      Meteraccur=2;
    }
    if (SOCdiff>4){
      Meteraccur=3;
    }
    
 }

if (chargersnbr==0){                
 if (battSOC<=13 and couldelay==0 and calistop==0){
    couldelaymillis=millis();
    couldelay=1;
 }
} 

if (couldelay==1){
  if (chargersnbr>=1){
    couldelay=0;
  }
  if (millis()>=couldelaymillis+200000){  
    SOCCoul=battSOC;
    CoulombPointer=(Cbattcapacity/100)*SOCCoul;
    Meteraccur=1;
    calistop=1;
    couldelay=0;
  }
}


if (battSOC<=12 and chargersnbr==0){
  SOCDisplay=battSOC;
}
else{
  SOCDisplay=SOCCoul;
}
  
}


void Newpowercycle(){ 

    cyclestartmillis=millis();
  
    lastaveragepower=averagepowerpool;
    averagepower=0;
    powercounter=1;
    if (inverter==0 and mode!=2){ 
      SOCfreeze=SOCfreeze+30;
    }
    maximumpower=averagepowerpool;
    minimumpower=averagepowerpool;
    diffpower=0; 
    averagepower1=0;
    averagepower2=0;
    averagepower3=0;
    averagepowerS=0;

    invwattcounter=1;
    averageinvwattspool=0;
    averageSOCpool=0;
}

void getCurrent(){

  shuntVolt=0;
  
  //ina219.startSingleMeasurement();                   // triggers single-shot measurement and waits until completed
  //shuntVolt = ina219.getShuntVoltage_mV();

  ina226.startSingleMeasurement();
  ina226.readAndClearFlags();
  shuntVolt = ina226.getShuntVoltage_mV();

  holdcurrent=(shuntVolt/shuntMaxmV)*shuntMaxAmps; 


   // Moving Average
   total = total-Areadings[AreadIndex];
   Areadings[AreadIndex]=holdcurrent;
   total = total+Areadings[AreadIndex];
   AreadIndex++;

   if (AreadIndex>=AnumReadings){
     AreadIndex = 0;
   }

    lastcurrent=current;
    current = total/AnumReadings;
     
    if (chargersnbr==0 and inverter==0){
      if (current>-2.0 and current<2.0){
       current=0.1;                        // current calibration for idle
      } 
    } 
   
 /*
   if (current<=0){

    if(chargersnbr==2){
      current=current*1.01;                  // corr for charging when more chargers engaged
    }
    if(chargersnbr==3){
      current=current*1.03;                  // corr for charging when more chargers engaged
   }
    
    
    //shunttemp=probetemp2-35;             // current corr by shunt temperature for charging
    //if (shunttemp<0){
    //  shunttemp=0;
    //}
    //current=current*(((shunttemp)/5)/100)+100);  

   }
   */
    
   if (current>40 and current<=48){
     current=current*0.97;                // current calibration for discharge
   }
   if (current>48){
     current=current*0.94;                // current calibration for discharge
   }

    
    current=current*10;
    current=round(current);
    current=current/10;
 

   if (current<-3.5 or current>3.5){         // setting window for "Homing" indication
    lastAlow=lastcurrent*0.97;
    lastAhigh=lastcurrent*1.03;
   }
   else{                                     // small currents require looser tolerance
    lastAlow=lastcurrent*0.95;
    lastAhigh=lastcurrent*1.05;   
   }

     if(current<0){
      if ((current>(lastAlow)) or (current<(lastAhigh))){
        homing=1;
      }
      else{
        homing=0;
      } 
     }
     if(current>=0){
      if ((current<(lastAlow)) or (current>(lastAhigh))){
        homing=1;
      }
      else{
        homing=0;
      } 
     }
     
 
}


void getStringvoltage(){

   int CHVread;
   float CHvolttemp;

   CHvoltage[vc]=0;
   CHvolttemp=0;

   for  (vt = 0; vt <=4; vt++) {
    CHVread=ads.readADC_SingleEnded(2);
    CHvolttemp=CHvolttemp+CHVread;
    delay(10);
   }

    CHvoltage[vc]=CHvolttemp/5.0;
    
    voltagecorrection();
    
    vc++;
    if (vc>15){
      vc=0;
      calculatestringV();
    }
    
    my_mux.channel(vc);
    
}


 void voltagecorrection(){

 if (vc==0){
   CHvoltage[vc]=CHvoltage[vc]*0.0001259;
 }
 if (vc==1){
   CHvoltage[vc]=CHvoltage[vc]*0.0002509;
 }
 if (vc==2){
   CHvoltage[vc]=CHvoltage[vc]*0.0003772;
 }
 if (vc==3){
   CHvoltage[vc]=CHvoltage[vc]*0.0004979;
 }
 if (vc==4){
   CHvoltage[vc]=CHvoltage[vc]*0.0006152;
 }
  if (vc==5){
   CHvoltage[vc]=CHvoltage[vc]*0.0007663;
 }
 if (vc==6){
   CHvoltage[vc]=CHvoltage[vc]*0.0009077;
 }
  if (vc==7){
   CHvoltage[vc]=CHvoltage[vc]*0.0009811;
 }
 if (vc==8){
   CHvoltage[vc]=CHvoltage[vc]*0.0011558;
 }
 if (vc==9){
   CHvoltage[vc]=CHvoltage[vc]*0.0012700;
 }
 if (vc==10){
   CHvoltage[vc]=CHvoltage[vc]*0.0013756;
 }
 if (vc==11){
   CHvoltage[vc]=CHvoltage[vc]*0.0015126;
 }
  if (vc==12){
   CHvoltage[vc]=CHvoltage[vc]*0.0015133;
 }
 if (vc==13){
   CHvoltage[vc]=CHvoltage[vc]*0.0017635;
 }
  if (vc==14){
   CHvoltage[vc]=CHvoltage[vc]*0.0017689;
 }
 if (vc==15){
   CHvoltage[vc]=CHvoltage[vc]*0.0019981;
 }

  
 }


void  calculatestringV(){

  float tempstringvoltage[16];

  tempstringvoltage[0]=CHvoltage[0];
  tempstringvoltage[1]=CHvoltage[1]-CHvoltage[0];
  tempstringvoltage[2]=CHvoltage[2]-CHvoltage[1]; 
  tempstringvoltage[3]=CHvoltage[3]-CHvoltage[2];
  tempstringvoltage[4]=CHvoltage[4]-CHvoltage[3];
  tempstringvoltage[5]=CHvoltage[5]-CHvoltage[4]; 
  tempstringvoltage[6]=CHvoltage[6]-CHvoltage[5];
  tempstringvoltage[7]=CHvoltage[7]-CHvoltage[6];
  tempstringvoltage[8]=CHvoltage[8]-CHvoltage[7]; 
  tempstringvoltage[9]=CHvoltage[9]-CHvoltage[8];
  tempstringvoltage[10]=CHvoltage[10]-CHvoltage[9];
  tempstringvoltage[11]=CHvoltage[11]-CHvoltage[10]; 
  tempstringvoltage[12]=CHvoltage[12]-CHvoltage[11];
  tempstringvoltage[13]=CHvoltage[13]-CHvoltage[12];
  tempstringvoltage[14]=CHvoltage[14]-CHvoltage[13]; 
  tempstringvoltage[15]=CHvoltage[15]-CHvoltage[14];


  // Moving Average
   totalSV[0] = totalSV[0]-S1readings[SreadIndex];
   S1readings[SreadIndex]=tempstringvoltage[0];
   totalSV[0] = totalSV[0]+S1readings[SreadIndex];
   totalSV[1] = totalSV[1]-S2readings[SreadIndex];
   S2readings[SreadIndex]=tempstringvoltage[1];
   totalSV[1] = totalSV[1]+S2readings[SreadIndex];
   totalSV[2] = totalSV[2]-S3readings[SreadIndex];
   S3readings[SreadIndex]=tempstringvoltage[2];
   totalSV[2] = totalSV[2]+S3readings[SreadIndex];
   totalSV[3] = totalSV[3]-S4readings[SreadIndex];
   S4readings[SreadIndex]=tempstringvoltage[3];
   totalSV[3] = totalSV[3]+S4readings[SreadIndex];
   totalSV[4] = totalSV[4]-S5readings[SreadIndex];
   S5readings[SreadIndex]=tempstringvoltage[4];
   totalSV[4] = totalSV[4]+S5readings[SreadIndex];
   totalSV[5] = totalSV[5]-S6readings[SreadIndex];
   S6readings[SreadIndex]=tempstringvoltage[5];
   totalSV[5] = totalSV[5]+S6readings[SreadIndex];
   totalSV[6] = totalSV[6]-S7readings[SreadIndex];
   S7readings[SreadIndex]=tempstringvoltage[6];
   totalSV[6] = totalSV[6]+S7readings[SreadIndex];
   totalSV[7] = totalSV[7]-S8readings[SreadIndex];
   S8readings[SreadIndex]=tempstringvoltage[7];
   totalSV[7] = totalSV[7]+S8readings[SreadIndex];
   totalSV[8] = totalSV[8]-S9readings[SreadIndex];
   S9readings[SreadIndex]=tempstringvoltage[8];
   totalSV[8] = totalSV[8]+S9readings[SreadIndex];
   totalSV[9] = totalSV[9]-S10readings[SreadIndex];
   S10readings[SreadIndex]=tempstringvoltage[9];
   totalSV[9] = totalSV[9]+S10readings[SreadIndex];
   totalSV[10] = totalSV[10]-S11readings[SreadIndex];
   S11readings[SreadIndex]=tempstringvoltage[10];
   totalSV[10] = totalSV[10]+S11readings[SreadIndex];
   totalSV[11] = totalSV[11]-S12readings[SreadIndex];
   S12readings[SreadIndex]=tempstringvoltage[11];
   totalSV[11] = totalSV[11]+S12readings[SreadIndex];
   totalSV[12] = totalSV[12]-S13readings[SreadIndex];
   S13readings[SreadIndex]=tempstringvoltage[12];
   totalSV[12] = totalSV[12]+S13readings[SreadIndex];
   totalSV[13] = totalSV[13]-S14readings[SreadIndex];
   S14readings[SreadIndex]=tempstringvoltage[13];
   totalSV[13] = totalSV[13]+S14readings[SreadIndex];
   totalSV[14] = totalSV[14]-S15readings[SreadIndex];
   S15readings[SreadIndex]=tempstringvoltage[14];
   totalSV[14] = totalSV[14]+S15readings[SreadIndex];
   totalSV[15] = totalSV[15]-S16readings[SreadIndex];
   S16readings[SreadIndex]=tempstringvoltage[15];
   totalSV[15] = totalSV[15]+S16readings[SreadIndex];

   
   SreadIndex++;

   if (SreadIndex>=SnumReadings){
     SreadIndex = 0;
   }

    stringvoltage[0] = totalSV[0]/SnumReadings;
    stringvoltage[1] = totalSV[1]/SnumReadings;
    stringvoltage[2] = totalSV[2]/SnumReadings;
    stringvoltage[3] = totalSV[3]/SnumReadings;
    stringvoltage[4] = totalSV[4]/SnumReadings;
    stringvoltage[5] = totalSV[5]/SnumReadings;
    stringvoltage[6] = totalSV[6]/SnumReadings;
    stringvoltage[7] = totalSV[7]/SnumReadings;
    stringvoltage[8] = totalSV[8]/SnumReadings;
    stringvoltage[9] = totalSV[9]/SnumReadings;
    stringvoltage[10] = totalSV[10]/SnumReadings;
    stringvoltage[11] = totalSV[11]/SnumReadings;
    stringvoltage[12] = totalSV[12]/SnumReadings;
    stringvoltage[13] = totalSV[13]/SnumReadings;
    stringvoltage[14] = totalSV[14]/SnumReadings;
    stringvoltage[15] = totalSV[15]/SnumReadings;


  lowestSV=stringvoltage[0];
  highestSV=stringvoltage[0];

  for (vi = 0; vi <= 15; vi++) {
      if(stringvoltage[vi]<lowestSV){
        lowestSV=stringvoltage[vi];
        lowestSnbr=vi+1;      
      }
      if(stringvoltage[vi]>highestSV){
        highestSV=stringvoltage[vi];
        highestSnbr=vi+1;     
      }
   }

    delta=highestSV-lowestSV; 

}


void getVoltage(){

 oldbattvoltage=battvoltage;

 voltread=ads.readADC_SingleEnded(0);  
 holdvolt=(voltread*(calibrationvalue+fineadjustment));   //calibrate your readout with a volt meter!
 
// Moving Average
   Vtotal = Vtotal-Vreadings[VreadIndex];
   Vreadings[VreadIndex]=holdvolt;
   Vtotal = Vtotal+Vreadings[VreadIndex];
   VreadIndex++;

   if (VreadIndex>=VnumReadings){
     VreadIndex = 0;
   }

   battvoltage = Vtotal/VnumReadings;
   watts=battvoltage*current;

    getCycles();
    Coulombmeter();
  
 diffbattvoltage=battvoltage-oldbattvoltage;
  if (diffbattvoltage<0){ 
  diffbattvoltage=diffbattvoltage*-1;
  }
  if (diffbattvoltage>0.012 and battvoltage<54.9){
  SOCfreeze=SOCfreeze+15;  
  }
  if (battvoltage>=55.25){
    SOCfreeze=0;
  }
  if (battvoltage<=49.6){
    SOCfreeze=0;
  }
 
  
 if (SOCfreeze>65){
  SOCfreeze=65; 
 }

// Absolute voltage limits
if (battvoltage<=44){
  MinLock=1;
}
else{
  MinLock=0;
}
if (battvoltage>=56.0){
  Maxlock=1;
}
 
}


void Startinverting(){                      // Modes 1,2,4 only

 invertertimeHr=(dt.hour);
 invertertimeMin=(dt.minute);

 invertertimeinminutes=(invertertimeHr*60)+invertertimeMin;
 invertertimeONinminutes=(NormInvHrOn*60)+NormInvMinOn;
 invertertimeOFFinminutes=(NormInvHrOff*60)+NormInvMinOff;
      
  if (inverteravailable==1){  
  if (NormInvHrOff<NormInvHrOn) {   
  if (invertertimeinminutes>=invertertimeONinminutes or invertertimeinminutes<invertertimeOFFinminutes){           
     if (battSOC>minimumSOC) {
         if (stopdelay==0){    
            startmillis=millis();
            invcorract=1;
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
        digitalWrite(relayPin3,LOW);
        delay(prechgreleasedelay);
        digitalWrite(relayPin5,HIGH);
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

 if (NormInvHrOff>NormInvHrOn) {   
 if (invertertimeinminutes>=invertertimeONinminutes and invertertimeinminutes<invertertimeOFFinminutes){            
     if (battSOC>minimumSOC) {
      if (stopdelay==0){ 
            startmillis=millis();
            invcorract=1;   
            digitalWrite(relayPin5,LOW);
            delay(prechargedelay);
            stopdelay=1;
         }
      digitalWrite(relayPin3,LOW);
      delay(prechgreleasedelay);
      digitalWrite(relayPin5,HIGH);
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

if (inverter==1) {

   if (firstdischarge==0 and SOCCoul>=1){
    firstdischarge=1;
    firstcharge=0;
    CDwatthours=0;
    dwattsstartmillis=millis();   

      if (coulcorrarm==0){       
        StartSOC=SOCCoul;
        EndSOC=SOCCoul;  
      } 
      else{
        StartSOC=0;
        EndSOC=0;
      }

      if (blockdischargetest==0){
        ActAh=0;
        ActWh=0;  
        watthoursTot=0;
        AhoursTot=0;
      }
      
    }
    
   getDischargeData(); 
         
}
else {
  if (firstcharge==0){
    Cbattcapacity=Cbattcapacity+25;     //slowly rising capacity to force a test
    firstcharge=1;
    firstdischarge=0;
    coulcorrCW=CCwatthours;
    coulcorrDW=CDwatthours;
    CCwatthours=0;
    Dtest=0;
    calistop=0;
    coulcorrend=battSOC;
    SOCfreeze=65;

   if (coulcorrarm==1){
      coulombmetercorrection();
   }
   
   coulcorrstart=battSOC; 
   if (capsave==1){
      blockdischargetest=1;
    }
  }
}

}

void coulombmetercorrection(){

    if (coulcorrstart>0){
      coulcorrend=battSOC;
      if (coulcorrend<=(coulcorrstart+1) and coulcorrend>=(coulcorrstart-1)){

        if ((millis()/10000)>9000){
        coulcorrarm=0;
        coulcorr=coulcorrCW/coulcorrDW;
        coulcorr=coulcorr*100;
        coulcorr=round(coulcorr);
        if (coulcorr>255){
          coulcorr=255;
        }
        coulcorrbyte=coulcorr;
        coulcorr=coulcorr/100;
        if (coulcorrbyte<=160 or coulcorrbyte>=70){
           EEPROM.write(36,coulcorrbyte);       
           delay(50); 
        }
        else{
           EEPROM.write(36,200);       
           delay(50); 
        }
        
      }
    }
    }
 
  
}


void Stopdevices(){

   digitalWrite(relayPin1,HIGH);
   digitalWrite(relayPin2,HIGH);
   digitalWrite(relayPin3,HIGH);
   digitalWrite(relayPin5,HIGH);
   digitalWrite(relayPin6,HIGH);
  
   stopdelay=0;
   chargerone=0;
   chargertwo=0;
   chargerthree=0;
   inverter=0;

   SOCfreeze=25;
   
}
 
void Checkminimumreached(){

   if (battSOC<=minimumSOC){

   if(minimumreach==0){
    minimumstartmillis=millis();
    minimumreach=1;
   }
   else{
    if (millis()>(minimumstartmillis+120000)){
       digitalWrite(relayPin5,HIGH);
       digitalWrite(relayPin3,HIGH);
       inverter=0;
       stopdelay=0;
       inverteravailable=0;
       SOCfreeze=SOCfreeze+55;
    }
   }
    
   }
   else{
   minimumreach=0; 
   }
   
}   

void InverterSOCcorrection(){

if (inverter==1 and invcorract==1){
  diffmillis=millis()-startmillis;
  if (diffmillis>300000){
    longinvcorr=1;
  }
  if (diffmillis>1800000){
    longinvcorr=2;
  }
  if (diffmillis>3600000){
    longinvcorr=3;
  }
  if (diffmillis>10800000){
    longinvcorr=4;
  }
  if (diffmillis>21600000){
    longinvcorr=5;
  } 
}
else{
  invcorract=0;
  longinvcorr=0;
}
  
}

void Checkidlestatus(){
  
    if (chargerone==0 and chargertwo==0){    // and inverter==0
         if (idleactive==0){
          idlestartmillis=millis();
          idleactive=1;
          idlestatus=0;
         }
         else {
          idlediffmillis=millis()-idlestartmillis;
           if (idlediffmillis>180000){
            idlestatus=1;
           }
           if (idlediffmillis>450000){
            idlestatus=2;
           }
           if (idlediffmillis>1100000){
            idlestatus=3;
           }
           if (idlediffmillis>1800000){
            idlestatus=4;
           }
           if (idlediffmillis>3600000){
            idlestatus=5;
           }
           if (idlediffmillis>18000000){
            idlestatus=6;
           }
         }
     }
     else{
      idleactive=0;
      idlestatus=0;
     }

}

void Limitercheck() {

if (Limiter==1  and typeofmeter!=9){

if (Limiterfirst==0 and measuredpower<-200 and dynlevel>0){
   LimiterOps=0;
   chargerone=0;
   chargertwo=0;
}
else {
   Limiterfirst=1;
}

if (Limitertimeout==0){
 if (chargerone==1 or chargertwo==1){
  Limiterstartmillis=millis();
  Limitertimeout=1;
 }
}


if (Limitertimeout==1){
 if (millis()>=Limiterstartmillis+25000){
  if (measuredpower<Limiterbreakout){
    Limitercounter++;
  }
  else{
    Limitercounter=0;
  }
 } 
}

if (Limitercounter>7 and Limitercounter<=12){
   LimiterOps=0;
   chargerone=0;
   SOCfreeze=2;
}

if (Limitercounter>12){
   LimiterOps=0;
   chargerone=0;
   chargertwo=0;
   Limitertimeout=0;
   Limitercounter=0;
   SOCfreeze=15;
}

}

}

void getSOC() {

OldbattSOC=battSOC;  
SOCvoltage=battvoltage;
corrindinv=0;

 if (current<-2){                       //charging

  SOCvoltage=SOCvoltage-0.15;        // basic LFP corr for going into charge
  
  if (battvoltage>=55.05){
    SOCvoltage=SOCvoltage+(current*(0.017/battCapFact));
   } 
   if (battvoltage>=54.80 and battvoltage<55.05){
    SOCvoltage=SOCvoltage+(current*(0.016/battCapFact));        // you might have to change current corrections according
   }  

   if (battvoltage>=54.45 and battvoltage<54.80){
    SOCvoltage=SOCvoltage+(current*(0.015/battCapFact));        
   }  

   if (battvoltage>=53.80 and battvoltage<54.45){
    SOCvoltage=SOCvoltage+(current*(0.0144/battCapFact));    
   }  

  if (battvoltage<53.80) {  
     SOCvoltage=SOCvoltage+(current*(0.013/battCapFact)); 
   }  
 }
 

if (inverter==1) {         //correcting by inverter voltage by Current module

  SOCvoltage=SOCvoltage+0.35;    // basic LFP corr for going into discharge

  if (battvoltage<52.9){
   SOCvoltage=SOCvoltage+0.06; 
  }
  if (battvoltage<52.7){
   SOCvoltage=SOCvoltage+0.05; 
  }
  if (battvoltage<52.4){
   SOCvoltage=SOCvoltage+0.04; 
  }
  if (battvoltage<52.2){
   SOCvoltage=SOCvoltage+0.04; 
  }
  if (battvoltage<52.0){
   SOCvoltage=SOCvoltage+0.04; 
  }
  if (battvoltage<51.6){
   SOCvoltage=SOCvoltage+0.05; 
  }
  if (battvoltage<50.8){
   SOCvoltage=SOCvoltage+0.06; 
  }
  if (battvoltage<50.1){
   SOCvoltage=SOCvoltage+0.07; 
  }
  
  if (longinvcorr==1){
    SOCvoltage=SOCvoltage+0.05; 
  }
  if (longinvcorr==2){
    SOCvoltage=SOCvoltage+0.09; 
  }
  if (longinvcorr==3){
    SOCvoltage=SOCvoltage+0.12; 
  }
  if (longinvcorr==4){
    SOCvoltage=SOCvoltage+0.15; 
  }

   if (battvoltage>=51.80){                                  //current correction
    SOCvoltage=SOCvoltage+(current*(0.013/battCapFact));    
   }  
   if (battvoltage<51.80 and battvoltage>=51.0){
    SOCvoltage=SOCvoltage+(current*(0.016/battCapFact));    
   }  
   if (battvoltage<51.0 and battvoltage>=50.40){
    SOCvoltage=SOCvoltage+(current*(0.019/battCapFact));    
   }  
   if (battvoltage<50.40 and battvoltage>=49.80){
    SOCvoltage=SOCvoltage+(current*(0.022/battCapFact));    
   }  
   if (battvoltage<49.80){
    SOCvoltage=SOCvoltage+(current*(0.025/battCapFact));    
   }  

  


}

SOCfreeze--;
if (SOCfreeze<0) {
  SOCfreeze=0;
}

if (coulstart==0){
  SOCfreeze=0;
}

if (SOCfreeze==0) {                   
  
battSOC=100;


corrindrest=0;


if (inverter==0){
if (idleactive==1 and chargerlock==1) {
   SOCvoltage=SOCvoltage+0.08;
   }

if (idlestatus==1) {                           // correcting voltage sag when batt idling for longer
  SOCvoltage=SOCvoltage+0.12;
  corrindrest=1;
}
if (idlestatus==2) {                           
  SOCvoltage=SOCvoltage+0.17;
  corrindrest=2;
}
if (idlestatus==3) {                           
  SOCvoltage=SOCvoltage+0.20;
  corrindrest=3;
}
if (idlestatus==4) {                           
  SOCvoltage=SOCvoltage+0.23;
  corrindrest=4;
}
if (idlestatus==5) {                           
  SOCvoltage=SOCvoltage+0.26;
  corrindrest=5;
}
if (idlestatus==6) {                           
  SOCvoltage=SOCvoltage+0.30;
  corrindrest=6;
}
}

if (longinvcorr==1){                          // correcting voltage sag when inverting for longer
  corrindrest=7;
}
if (longinvcorr==2 or longinvcorr==3){
  corrindrest=8;
}
if (longinvcorr>=4){
  corrindrest=9;
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
battSOC=86;
}
if (SOCvoltage<=53.61) {
battSOC=84;
}
if (SOCvoltage<=53.60) {
battSOC=83;
}  
if (SOCvoltage<=53.57) {
battSOC=80;
}
if (SOCvoltage<=53.55) {
battSOC=79;
}
if (SOCvoltage<=53.52) {
battSOC=78;
}
if (SOCvoltage<=53.50) {
battSOC=77;
}
if (SOCvoltage<=53.47) {
battSOC=76;
}
if (SOCvoltage<=53.45) {
battSOC=75;
}
if (SOCvoltage<=53.41) {
battSOC=74;
}
if (SOCvoltage<=53.38) {
battSOC=73;
}
if (SOCvoltage<=53.34) {
battSOC=72;
}
if (SOCvoltage<=53.31) {
battSOC=71;
}
if (SOCvoltage<=53.27) {
battSOC=70;
}
if (SOCvoltage<=53.25) {
battSOC=69;
}
if (SOCvoltage<=53.23) {
battSOC=68;
}  
if (SOCvoltage<=53.19) {
battSOC=67;
}
if (SOCvoltage<=53.17) {
battSOC=66;
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
if (SOCvoltage<=53.11) {
battSOC=85;
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
if (SOCvoltage<=53.06) {
battSOC=54;
}
if (SOCvoltage<=53.04) {
battSOC=53;
}  
if (SOCvoltage<=53.02) {
battSOC=52;
} 
if (SOCvoltage<=52.00) {
battSOC=51;
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
if (SOCvoltage<=52.95) {
battSOC=46;
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
if (SOCvoltage<=52.91) {
battSOC=41;
}
if (SOCvoltage<=52.90) {
battSOC=40;
}
if (SOCvoltage<=52.89) {
battSOC=39;
}
if (SOCvoltage<=52.88) {
battSOC=37;
}   
if (SOCvoltage<=52.87) {
battSOC=35;
}
if (SOCvoltage<=52.85) {
battSOC=34;
}
if (SOCvoltage<=52.83) {
battSOC=33;
}  
if (SOCvoltage<=52.80) {
battSOC=32;
}
if (SOCvoltage<=52.78) {
battSOC=31;
}
if (SOCvoltage<=52.76) {
battSOC=30;
}
if (SOCvoltage<=52.73) {
battSOC=29;
}
if (SOCvoltage<=52.69) {
battSOC=28;
}  
if (SOCvoltage<=52.65) {
battSOC=27;
}
if (SOCvoltage<=52.61) {
battSOC=26;
}
if (SOCvoltage<=52.57) {
battSOC=25;
}
if (SOCvoltage<=52.53) {
battSOC=24;
}
if (SOCvoltage<=52.48) {
battSOC=23;
}  
if (SOCvoltage<=52.40) {
battSOC=22;
}
if (SOCvoltage<=52.32) {
battSOC=21;
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
if (SOCvoltage<=47.40) {
battSOC=1;
}
if (SOCvoltage<=45.00) {
battSOC=0;
}  



if (battSOC>OldbattSOC+25){
  OldbattSOC=battSOC;
}

if (battvoltage<55 and battSOC<=94){
 if (battSOC<OldbattSOC-1){
  battSOC=OldbattSOC-1;
  SOCfreeze=SOCfreeze+2;  
 }
 if (battSOC>OldbattSOC+1){
  battSOC=OldbattSOC+1;
  SOCfreeze=SOCfreeze+2;  
 }
}

}

averageSOCpool=averageSOCpool+battSOC;

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
  if (_buffer[frameSize - 1] != crc >> 8 && (_buffer[frameSize - 2] != crc) & 0x00FF) {
    (*_mbSensorPtr).putStatus(MB_INVALID_CRC);
    return;
  }
  if (_buffer[1] & (0x80 == 0x80)) {
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
      // holdpower=String(temp.f);  // saves value as string to enable negative values
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
