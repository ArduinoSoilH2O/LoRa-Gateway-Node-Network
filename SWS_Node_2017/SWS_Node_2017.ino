
/* Data logger Node using Moteino and Acclima TDR315L sensors
    Components:
      - MoteinoMEGA microcontroller
      - 3 Acclima TDR315L SDI-12 sensors
      - 1 CS655
      - microSD card
      - RTC
      - Solar Panel
      - 6 rechargeable AA battery pack
    Function: to log hourly measurements from sensors with datas on a microSD card and
              send data to a central collector
    Program Summary:
      The program executes the following steps:
      - Find active SDI-12 addresses
      - Check clock
      - If time is the top of the hour, take measurements from sensors
      - Save IDnum, date and time, and data to SD card
      - Send data to gateway via LoRa radio

      store info to Moteino EEPROM:
   0:  BTorSerialOut
   1:  radioID
   2:  IDnum
   3:  GatewayID
   4:  FarmID
   5:
   6:
   7:
  MAXIMUM 4 SENSORS PER DATA PIN
  --> CS655 moved to separate data pin

  March 2, 2017
  Alondra Thompson
*/

//===================================================================================================

//------------ Libraries --------------------------------------

#include <SDI12.h>                                    // SDI-12 functions
#include <SD.h>                                       // SDcard functions
#include <SPI.h>                                      // SPI functions
#include <Wire.h>                                     // i2c functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <LowPower.h>                                 // low-power sleep functions
//#include <RFM69.h>                                    // Moteino radio library
#include <RH_RF95.h>                                  // Moteino LoRa library
#include <RHReliableDatagram.h>                       // allows acknowledgements and retries
//#include <SPIFlash.h>                                 // SPI flash memory library
#include "RTClib.h"                                   // for Adafruit RTC (PCF8523)
#include <OneWire.h>

RTC_PCF8523 rtc;
RH_RF95 driver;


//------------- Assign Pins --------------------------------------

#define TDRpin 10                                    // Pin for TDR SDI-12 sensors; handles interrupts
#define LEDPIN 15                                     // MEGA LED on D15
#define battPin 24                                    // A0 for reading Vout (to calculate battV)
#define SD_CS 3                                       // ChipSelect pin for SDcard SPI
//#define BTsleep 12                                    // Bluetooth KEY (on/off switch) LOW = on, HIGH = off
//#define BTstate 13                                    // Bluetooth State (tells if on or off)
#define CS655pin 11                                   // Pin for CS655 SDI-12 sensors; handles interrupts


//------------- Declare Variables --------------------------------------

int   IDnum;                                         // site/board identifier
int   FarmID;
char  filename[] = "000_Data.txt";                   // SDcard file name, 000 to be replaced by IDnum

char  a[4];                                          // char array for itoa function
byte  i;                                             // for-loop counter

byte   secs;                                         // time and date values
byte   mins;
byte   hrs;
byte   dow;
byte   days;
byte   mnths;
int    yrs;

//int   BTorSerialOut;                                // Bluetooth or serial indicator
boolean isMenuOn = false;
int   menuinput;                                    // user input to menu prompt
long  timeout;                                      // length of time to wait for user input
int   indata;                                       // user input data
int   input;
int   numincoming;
int   incoming[7];
boolean firstTime = true;                           // flag for storing data header on SDcard

uint8_t   radioID;                                   // the same on all nodes that talk to each other
uint8_t GatewayID;                                   // receiver ID
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
char Data[RH_RF95_MAX_MESSAGE_LEN];

String data = "";
String Response2 = "";
String Response1 = "";
String header = "";

float Vout;                                         // voltage divider voltage
float battV;                                        // battery voltage

float R1 = 22000;                                   // R1 of voltage divider circuit (22kohm)
float R2 = 10000;                                   // R2 of voltage divider circuit (10kohm)
float resist = (R1 + R2) / R2;                      // total resistance of voltage divider circuit

float X = 0.00322;                                  // scaling factor for analogRead X = 3.3V/ 1024 units

byte registerTDR[8] = {
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000
};

byte registerCS655[8] = {
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000
};

int DS18S20_Pin = 31;
float Temp;


// ------- Initialize ----------------------------------------------------

RHReliableDatagram manager(driver, radioID);
//RH_RF95 manager;      // for broadcast transmissions

SDI12 CS655(CS655pin);
SDI12 TDR(TDRpin);
OneWire ds(DS18S20_Pin);
File myfile;                                          // make SDcard file object

//===================================================================================================

//------------- Set Up --------------------------------------

void setup() {
  Serial.begin(9600);

  // while(!Serial){;}
  delay(100);

  pinMode(SD_CS, OUTPUT);                             // ChipSelect pin
  pinMode(LEDPIN, OUTPUT);
  pinMode(battPin, INPUT);

  //  mySDI12.begin();                                    // enable SDI12 functions
  // CS655.begin();
  Wire.begin();                                       // enable i2c bus for RTC
  delay(300);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

    if (!SD.begin(SD_CS)) {
      Serial.println("Initialization Failed");
      return;
    }
    Serial.println("Initialization Completed");

//      if (!manager.init()){
//      Serial.println("init failed");}


  radioID = EEPROM.read(1);                                // read board ID number from EEPROM
  manager.setThisAddress(radioID);
  IDnum = EEPROM.read(2);                                  // read board ID
  GatewayID = EEPROM.read(3);                              // read gateway ID
  FarmID = EEPROM.read(4);                                 // read Farm ID

  delay(1000);

  //----- Scan SDI-12 Addresses -----

  /*
     Quickly Scan the Address Space
  */

 // Serial.print("Looking for SDI-12 sensors...");
  
  SDI12 TDR(TDRpin);
  TDR.setActive();
  delay(500);

  Serial.println("Scanning for TDRs...");
  
  for (byte i = '0'; i <= '9'; i++) if (checkActiveTDR(i)) setTakenTDR(i); // scan address space 0-9

  for (byte i = 'a'; i <= 'z'; i++) if (checkActiveTDR(i)) setTakenTDR(i); // scan address space a-z

  for (byte i = 'A'; i <= 'Z'; i++) if (checkActiveTDR(i)) setTakenTDR(i); // scan address space A-Z
  
  // scan address space 0-9
  for(char i = '0'; i <= '9'; i++) if(isTakenTDR(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(":");
    printInfoTDR(i);   
    Serial.println(); 
  }
  
  // scan address space a-z
  for(char i = 'a'; i <= 'z'; i++) if(isTakenTDR(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    printInfoTDR(i);   
    Serial.println(); 
  } 
  
  // scan address space A-Z
  for(char i = 'A'; i <= 'Z'; i++) if(isTakenTDR(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(":");
    printInfoTDR(i);   
    Serial.println();  
  } 
  
  TDR.end();
  delay(100);

  SDI12 CS655(CS655pin);
  CS655.setActive();
  delay(500);

  Serial.println("Scanning for CS655s...");

  for (byte i = '0'; i <= '9'; i++) if (checkActiveCS655(i)) setTakenCS655(i); // scan address space 0-9

  for (byte i = 'a'; i <= 'z'; i++) if (checkActiveCS655(i)) setTakenCS655(i); // scan address space a-z

  for (byte i = 'A'; i <= 'Z'; i++) if (checkActiveCS655(i)) setTakenCS655(i); // scan address space A-Z

  // scan address space 0-9
  for(char i = '0'; i <= '9'; i++) if(isTakenCS655(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(":");
    printInfoCS(i);   
    Serial.println(); 
  }
  
  // scan address space a-z
  for(char i = 'a'; i <= 'z'; i++) if(isTakenCS655(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    printInfoCS(i);   
    Serial.println(); 
  } 
  
  // scan address space A-Z
  for(char i = 'A'; i <= 'Z'; i++) if(isTakenCS655(i)){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(":");
    printInfoCS(i);   
    Serial.println();  
  } 
  
  CS655.end();
  Serial.println("Done");


  delay(10);
  Serial.println(F("hello"));
  delay(10);

  menu();                                          // display menu on startup

}

//===================================================================================================

//------------- Main Loop --------------------------------------

void loop() {

  readRTC();          //Read RTC

  //---- Demo Loop - Take Measurements Every 5 min ----
//
//  if (mins % 10 == 0)
//  {
//    readTDR(i);
//    delay(50);
//    readCS655(i);
//    delay(100);
//    compile();
//    delay(50);
//    saveData();
//      digitalWrite(LEDPIN, HIGH);
//      transmitData();
//      digitalWrite(LEDPIN, LOW);
//    clearBuffer();
////    Serial.println();
//      sleepytime(6);      //low-power sleep mode
//  } else {
//    sleepytime(7);      //if not measurement or transmit interval, go to sleep
//  }

  //---- Real Loop - Take Measurements Every Hour ----

   if (mins == 0)      //Take measurements every hour
    {
      readTDR(i);
      delay(50);
      readCS655(i);
      delay(100);
      compile();
      delay(50);
      saveData();
      digitalWrite(LEDPIN, HIGH);
      transmitData();
      digitalWrite(LEDPIN, LOW);
      clearBuffer();
      sleepytime(6);      //low-power sleep mode
     } else {
      sleepytime(7);      //if not measurement or transmit interval, go to sleep
    }
  
}

//===================================================================================================

//----- Find Active Sensor Addresses -----

// this checks for activity at a particular address
// expects a char, '0'-'9', 'a'-'z', or 'A'-'Z'
boolean checkActiveTDR(char i) {
//  Serial.print("Checking address ");
//  Serial.print(i);
//  Serial.print("...");
  String myCommand = "";
  myCommand = "";
  myCommand += (char) i;                 // sends basic 'acknowledge' command [address][!]
  myCommand += "!";


  for (int j = 0; j < 3; j++) {          // goes through three rapid contact attempts
    TDR.sendCommand(myCommand);
    if (TDR.available() > 1) break;
    delay(30);
  }

  if (TDR.available() > 2) {   // if it hears anything it assumes the address is occupied
  //  Serial.println("TDR Occupied");
    TDR.flush();
    return true;
  }
  else {
 //  Serial.println("TDR Vacant");           // otherwise it is vacant.
    TDR.flush();
  }
  return false;
  
}

boolean isTakenTDR(byte i) {
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  return registerTDR[j] & (1 << k); // return bit status
}

boolean setTakenTDR(byte i) {
  boolean initStatusTDR = isTakenTDR(i);
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  registerTDR[j] |= (1 << k);
  return !initStatusTDR; // return false if already taken
}

boolean checkActiveCS655(char i) {

//  Serial.print("Checking address ");
//  Serial.print(i);
//  Serial.print("...");
  String myCommand = "";
  myCommand = "";
  myCommand += (char) i;                 // sends basic 'acknowledge' command [address][!]
  myCommand += "!";

  for (int j = 0; j < 3; j++) {
    CS655.sendCommand(myCommand);
    if (CS655.available() > 1) break;
    delay(30);
  }

  if (CS655.available() > 2) {   // if it hears anything it assumes the address is occupied
 //  Serial.println("CS655 Occupied");
    CS655.flush();
    return true;
  }
  else {
 //  Serial.println("CS655 Vacant");           // otherwise it is vacant.
    CS655.flush();
  }

  return false;

}

boolean isTakenCS655(byte i) {
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  return registerCS655[j] & (1 << k); // return bit status
}


boolean setTakenCS655(byte i) {
  boolean initStatusCS655 = isTakenCS655(i);
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  registerCS655[j] |= (1 << k);
  return !initStatusCS655; // return false if already taken
}

byte charToDec(char i) {
  if ((i >= '0') && (i <= '9')) return i - '0';
  if ((i >= 'a') && (i <= 'z')) return i - 'a' + 10;
  if ((i >= 'A') && (i <= 'Z')) return i - 'A' + 37;
}

// this sets the bit in the proper location within the addressRegister
// to record that the sensor is active and the address is taken.

// gets identification information from a sensor, and prints it to the serial port
// expects a character between '0'-'9', 'a'-'z', or 'A'-'Z'. 
char printInfoTDR(char i){
  int j; 
  String command = "";
  command += (char) i; 
  command += "I!";
  for(j = 0; j < 1; j++){
    TDR.sendCommand(command);
    delay(30); 
    if(TDR.available()>1) break;
    if(TDR.available()) TDR.read(); 
  }
  
  TDR.read(); //consume sensor address (you can keep it if you'd like)

  while(TDR.available()){
    Serial.write(TDR.read()); 
    delay(5); 
  } 
}

char printInfoCS(char i){
  int j; 
  String command = "";
  command += (char) i; 
  command += "I!";
  for(j = 0; j < 1; j++){
    CS655.sendCommand(command);
    delay(30); 
    if(CS655.available()>1) break;
    if(CS655.available()) CS655.read(); 
  }
  
 // CS655.read(); //consume sensor address (you can keep it if you'd like)

  while(CS655.available()){
    Serial.write(CS655.read()); 
    delay(5); 
  } 
}

//===================================================================================================

//------------- Take Measurements from Active Sensors ------------------

void readTDR(char i) {
  SDI12 TDR(TDRpin);
  TDR.begin();
  delay(500);
 // Serial.println("From TDRs...");
 
  // scan address space 0-9
  for (char i = '0'; i <= '9'; i++) if (isTakenTDR(i)) {
        measureTDR(i);
    }
   

//  // scan address space a-z
  for (char i = 'a'; i <= 'z'; i++) if (isTakenTDR(i)) {
         measureTDR(i);
    }

//  // scan address space A-Z
  for (char i = 'A'; i <= 'Z'; i++) if (isTakenTDR(i)) {
         measureTDR(i);
    }
 //Serial.println(Response1);  
    delay(100);
    TDR.flush();
    TDR.end();
    delay(100);
     
}

void readCS655(char i) {
  SDI12 CS655(CS655pin);
  CS655.begin();
  delay(500);
 // Serial.println("From CS655...");
  
  // scan address space 0-9
  for (char i = '0'; i <= '9'; i++) if (isTakenCS655(i)) {
      measureCS655(i);
    }

  // scan address space a-z
  for (char i = 'a'; i <= 'z'; i++) if (isTakenCS655(i)) {
      measureCS655(i);
    }

  // scan address space A-Z
  for (char i = 'A'; i <= 'Z'; i++) if (isTakenCS655(i)) {
      measureCS655(i);
    }
   // Serial.println(Response2);
    delay(100);
    CS655.flush();
    CS655.end();  
    delay(100);
}

//------------- Take Measurement --------------------------------------

void measureTDR(char i) {

  String readData = "";
  readData += i;
  readData += "M!";
  TDR.sendCommand(readData);
  delay(3000);

  String sdiResponse;

  while (TDR.available()) {
    char c = TDR.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
// Serial.println(sdiResponse);

  String sendData = "";
  sendData += i;
  sendData += "D0!";
  TDR.sendCommand(sendData);
  delay(1000);

  while (TDR.available()) {
    char c = TDR.read();
    if ((c != '\n') && (c != '\r')) {
      if (c ==  '+' || c == '-') {
        Response1 += ',';
        if (c == '-') Response1 += '-';
      } else {
        Response1 += c;
      }
      delay(50);
    }
  }
 
  Response1 += ',';
   //   sdiResponse = "";
// Serial.println(Response1); 

}

void measureCS655(char i) {
  String sdiResponse;
   String readData = "";
  readData += i;
  readData += "M!";
  CS655.sendCommand(readData);
  delay(3000);



  while (CS655.available()) {
    char c = CS655.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }

  String sendData = "";
  sendData += i;
  sendData += "D0!";
  CS655.sendCommand(sendData);
  delay(300);


  while (CS655.available()) {
    char c = CS655.read();
    if ((c != '\n') && (c != '\r')) {
      if (c ==  '+' || c == '-') {
        Response2 += ',';
        if (c == '-') Response2 += '-';
      } else {
        Response2 += c;
      }
          delay(50);
    }
  }

  Response2 += ',';
// sdiResponse = "";
 // Serial.println(Response2);
}
//===================================================================================================

//---------- Clear bufferA ---------------
void clearBuffer() {
  Response2 = "";
  Response1 = "";
  delay(100);
}

//================================================================================================

//--------------- Read DS18B20 Temperature Sensor ------------------------------------------------------------

float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
   // Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
  //  Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  delay(750); // Wait for temperature conversion to complete

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

  //  Serial.print("Temp: ");
  //  Serial.println(Temp);

}


//================================================================================================

//--------------- Calculate Battery Voltage ------------------------------------------------------------

float calcbattV() {
  int Aout;
  Aout = analogRead(battPin); // analogRead returns an integer between 0 and 1023
  //  Serial.print("Aout: ");
  //     Serial.println(Aout);

  Vout = Aout * X;
  //     Serial.print("Vout: ");
  //     Serial.println(Vout);

  battV = Vout * resist;
  //  Serial.print("battV: ");
  //  Serial.println(battV);
  //  Serial.println();
}

//===================================================================================================

//------------- Compile data and data  --------------------------------------

void compile() {      //compile data

//  int n1;
//  n1 = Response1.length();
//  int n2;
//  n2 = Response2.length();
//  
//  char R1[n1];
//  char R2[n2];
//  Response1.toCharArray(R1,n1);
//  Response2.toCharArray(R2,n2);
//  
//  Serial.println(Response1);
//  Serial.println(Response2);
  
  Temp = getTemp();
  calcbattV();
 data = "";

header = "";

delay(20);

  header += IDnum;
  header += ',';
  header += battV;
  header += ',';
  header += mnths;
  header += '/';
  header += days;
  header += '/';
  header += yrs;
  header += ' ';
  header += hrs;
  header += ':';
  if (mins < 10) data += '0';
  header += mins;
  //  data += ':';                         // don't need seconds
  //  if (secs < 10) data += '0';
  //  data += secs;
  header += ',';
 header += Temp;
 header += ',';
  delay(20);


 data= header + Response2 +Response1;
delay(50);


  data.remove(data.length()-1,1);             // remove last comma
 Serial.println(data);
  delay(20);
}

//==============================================================================================

//------------- Store data to mircoSD --------------------------------------

void saveData() {
  if (!SD.begin(SD_CS))                              // enable SD functions
  {
  }
  delay(50);

  myfile = SD.open(filename, FILE_WRITE);            // open file for writing
  delay(10);

  if (myfile) {
    if (firstTime == true)                             // write data output description on startup
    {
      myfile.println();
      myfile.println(F("Data values stored are as follows:"));
      myfile.println(F(" Node ID, Node BattV (V), Timestamp, Residue Temp (C), "));
      myfile.println(F("   For Acclima TDR310S: Sensor Address, VWC (%), Soil Temp (C), Permittivity, Bulk EC uS/cm, Pore Water EC uS/cm"));
      myfile.println(F("   For CS655: Sensor Address, VWC (m3/m3), EC (dS/m), Soil Temp (C)")); 
      myfile.println();

      firstTime = false;                               // set flag so it only writes once
    }

    myfile.println(data);
    delay(200);
    myfile.close();
    delay(200);
  } else {
    Serial.println("Error opening file");
  }
}

//==============================================================================================

//------------- Send data to receiver --------------------------------------

void transmitData() {



  if (!manager.init())
    Serial.println("radio failed");

  byte Len1;

  Len1 = data.length() + 1;

  char Pack1[Len1];

  data.toCharArray(Pack1, Len1);

  //Serial.println(Pack1);
  //Serial.println((uint8_t*)Pack1);

  uint8_t Data[Len1];

  //Data=atoi(Pack1);

  for (i = 0; i <= Len1; i++) {
    Data[i] = Pack1[i];
  }

  delay(50);

  Serial.println((char*)Data);

  delay(10);

//  Serial.print(mnths);                               // date
//  Serial.print('-');
//  Serial.print(days);
//  Serial.print('-');
//  Serial.print(yrs);
//  Serial.print(' ');
//  Serial.print(hrs);                                 // time
//  Serial.print(':');
//  if (mins < 10)
//  {
//    Serial.print('0');
//  }
//  Serial.print(mins);
//  Serial.print(':');
//  if (secs < 10)
//  {
//    Serial.print('0');
//  }
//  Serial.println(secs);

  Serial.println("Sending...");
  //delay(20);

  //------ for Reliable Datagram ------

  //  int timeoutr = 10000;
  //  manager.setTimeout(timeoutr);

  manager.sendtoWait(Data, Len1, GatewayID);      // send data, don't wait for ack

  //if (manager.sendtoWait(Data, Len1, GatewayID)){
  //  uint8_t len = sizeof(buf);
  //  uint8_t from;
  //  if(manager.recvfromAckTimeout(buf, &len, timeoutr, &from)){
  //     Serial.print("got reply from : 0x");
  //      Serial.print(from, HEX);
  //      Serial.print(": ");
  //      Serial.println((char*)buf);
  //  }
  //  else
  //    {
  //      Serial.println("No reply, is rf95_reliable_datagram_server running?");
  //    }
  //}

  //------ for Broadcasting Transmissions ------
  //  manager.send(Data, sizeof(Data));
  //
  //  manager.waitPacketSent();
  //  // Now wait for a reply
  //  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  //  uint8_t len = sizeof(buf);
  //
  //  if (manager.waitAvailableTimeout(3000))
  //  {
  //    // Should be a reply message for us now
  //    if (manager.recv(buf, &len))
  //   {
  //      Serial.print("got reply: ");
  //      Serial.println((char*)buf);
  //      Serial.print("RSSI: ");
  //      Serial.println(manager.lastRssi(), DEC);
  //
  //
  //  }
  //  }

  //-------

  //    else
  //    {
  //      Serial.println("No reply, is Gateway running?");
  //    }
  //}
  //  else {
  //    Serial.println("sendtoWait failed");

  //delay(500);

}

//==============================================================================================

//------------- Read DS1337 real-time clock/calendar --------------------------------------
void readRTC()
{
  //---- if using Adafruit RTC breakout PCF8532

  DateTime now = rtc.now();

  secs = now.second();
  mins = now.minute();
  hrs = now.hour();
  days = now.day();
  mnths = now.month();
  yrs = now.year();

}
//===============================================================================================

//------------- Low-power sleep --------------------------------------

void sleepytime(int sleepint)
{
  for (i = 1; i <= sleepint; i++)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   // max low power for 8 sec
  }
}

//==============================================================================================

//------------- Menu Routine --------------------------------------

void menu()
{

  if (Serial.available() > 0)
  {
    Serial.read();       // clear serial input buffer
  }

  IDnum = EEPROM.read(2);                            // read IDnum from EEPROM
  itoa(IDnum, a, 10);                                // convert ID number to character array

  if (IDnum < 10) {
    filename[0] = '0';                              // put into filename[] array
    filename[1] = '0';
    filename[2] = a[0];
  }
  else if (IDnum < 100)
  {
    filename[0] = '0';                              // put into filename[] array
    filename[1] = a[0];
    filename[2] = a[1];
  }
  else
  {
    filename[0] = a[0];                              // put into filename[] array
    filename[1] = a[1];
    filename[2] = a[2];
   }


  Serial.println();
  Serial.println(F("SDI-12 Datalogger"));               // print out board info
  Serial.print(F("Farm ID: "));
  Serial.println(FarmID);
  Serial.print(F("Board ID:  "));                // IDnum
  Serial.println(IDnum);
  Serial.print(F("Radio ID: "));
  Serial.println(radioID);
  Serial.print(F("Gateway ID: "));
  Serial.println(GatewayID);

  Serial.println(F("Data filename: "));                // SDcard filename
  Serial.println(filename);

  readRTC();
  Serial.print(F("Current date:  "));
  Serial.print(mnths);                               // date
  Serial.print('-');
  Serial.print(days);
  Serial.print('-');
  Serial.println(yrs);

  Serial.print(F("Current time:  "));
  Serial.print(hrs);                                 // time
  Serial.print(':');
  if (mins < 10)
  {
    Serial.print('0');
  }
  Serial.print(mins);
  Serial.print(':');
  if (secs < 10)
  {
    Serial.print('0');
  }
  Serial.println(secs);

  Serial.println(F("   c  <--  Set clock"));
  Serial.println(F("   i  <--  Set ID numbers"));  // set IDnum, radioID, GatewayID
  Serial.println(F("   t  <--  Test measurements"));    // take test measurements from sensors
  //  Serial.println(F("   r  <--  Download range of data"));  // set beginning date to download
  Serial.println(F("   d  <--  Download all data"));       // get all data
  //  Serial.println(F("   e  <--  Erase all data"));          // erase all data
  //  Serial.println(F("   m  <--  Repeat menu"));             // erase all data
  Serial.println(F("   x  <--  Exit"));                    // exit, turn off Bluetooth


  timeout = millis() + 10000;                         // wait 5 secs for input
  while (millis() < timeout)
  {
    menuinput = 120;
    if (Serial.available() > 0)                        // if something typed, go to menu
    {
      menuinput = Serial.read();               // get user input
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }
  }

  // menuinput -= 48;
  switch (menuinput)
  {

    case 99:           // ------ c ----------------------------------------------

      Serial.println(F("Set clock: "));

      Serial.print(F("  input month: "));
      getinput();
      mnths = indata;
      Serial.print(F("  input day:    "));
      getinput();
      days = indata;
      Serial.print(F("  input year:   "));
      getinput();
      yrs = indata;
      Serial.print(F("  input hour:   "));
      getinput();
      hrs = indata;
      Serial.print(F("  input minute: "));
      getinput();
      mins = indata;
      //secs = 0;

      rtc.adjust(DateTime(yrs, mnths, days, hrs, mins, secs));
      delay(50);

      menu();
      break;


    case 105:          // ----------i----------------------------------------------------
      Serial.println(F("Set network ID numbers:"));    // set radioID, IDnum and GatewayID numbers
      Serial.flush();

      Serial.print(F(" Farm ID:     "));             // get BoardID
      Serial.flush();
      getinput();                                  // decode user input
      FarmID = indata;

      Serial.print(F(" Board ID:     "));             // get BoardID
      Serial.flush();
      getinput();                                  // decode user input
      IDnum = indata;



      Serial.print(F(" Radio ID:  "));            // get radioID
      Serial.flush();
      getinput();
      radioID = indata;


      Serial.print((" Gateway ID:  "));             // get GatewayID
      Serial.flush();
      getinput();
      GatewayID = indata;

      EEPROM.write(1, radioID);                     // store settings to EEPROM
      manager.setThisAddress(radioID);
      delay(20);
      EEPROM.write(2, IDnum);
      delay(20);
      EEPROM.write(3, GatewayID);
      delay(20);
      EEPROM.write(4, FarmID);
      delay(20);

      menu();                                      // go back to menu
      break;

    case 100:          // ------ d ----------------------------------------------
      Serial.println(F("Download all data:"));     // download all data in SD card
      delay(100);

      myfile = SD.open(filename);               // open file
      //delay(1000);

      if (myfile) {

        while (myfile.available()) {            // read file and print to screen
          Serial.write(myfile.read());
        }
        myfile.close();

      } else {
        Serial.println("Error opening file");
      }
      menu();
      break;


    case 116:          // ------ t ----------------------------------------------
      Serial.println(F("Test measurements:"));     // take 3 readings to check sensors
      Serial.println();
      delay(10);

      for (int jj = 1; jj <= 3; jj++)
      {
        readRTC();
        readTDR(i);
        delay(50);
        readCS655(i);
        delay(50);
        compile();
      //Serial.println(data);
        delay(3000);
      }

      menu();
      break;


    //    case 109:          // ------ m ----------------------------------------------
    //      menu();                                           // repeat menu
    //      break;


    case 120:          // ------ x ----------------------------------------------
      Serial.println(F("Exit"));                           // exit, turn off Bluetooth
      Serial.println();
      delay(10);

      break;                                               // exit switch(case) routine

    default:          // -------------------------------------------------------
      Serial.println(F("Exit"));                           // exit, turn off Bluetooth
      Serial.println();
      delay(10);

      break;                                               // if no valid user input, leave menu
  }
  //
  //  for (i = 1; i <= 3; i++)                                 // output test readings
  //  {
  //    readRTC();                                             // get current date, time
  //    readSensors(i);
  //    compile();
  //    Serial.println(data);
  //    delay(2000);
  //  }

  // digitalWrite(BTsleep, HIGH);                      // turn off power to BT
  delay(100);
}

//==============================================================================================

//------------- Get User Input --------------------------------------

void getinput()
{
  timeout = millis() + 10000;         // time to wait for user to input something

  indata = 0;                                      // initialize
  while (millis() < timeout)                       // wait for user to input something
  {
    if (Serial.available() > 0)                    // something came in to BT serial buffer
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      for (i = 1; i <= numincoming; i++)           // read in everything
      {
        incoming[i] = Serial.read();               // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)   // ignore CR or LF
        {
        }
        else                                       // otherwise
        {
          input = incoming[i] - 48;                // convert ASCII value to decimal
          indata = indata * 10 + input;            // assemble to get total value
        }
      }
      break;                                       // exit before timeout
    }
  }
  Serial.println(indata); delay(10);
}

//==============================================================================================




