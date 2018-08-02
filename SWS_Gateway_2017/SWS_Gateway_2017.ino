/*
  Wireless SDI-12 Gateway receiver
    Components:
      - Moteino microcontroller board
      - RTC
      - microSD card
      - SHT31-D Temp/RH sensor

    Function: to wake at different times to receive incoming data from nodes (4)
              and store all received data in a .txt file on the microSD card
    Program Summary:
      - Check time
      - If the time corresponds to one of the node transmission times, wake the
        radio and receive the 2 data arrays
      - Compile arrays into one and store that array onto the microSD
      - Sleep

  store info to Moteino EEPROM:
   0:  BTorSerialOut
   1:  GatewayID
   2:  FarmID
   3:
   4:
   5:
   6:
   7:

   Alondra Thompson
   November 2016

   Vout read on A7
*/

//===================================================================================================

// ------- Libraries ----------------------------------------------------

#include <SD.h>                                    // SDcard libraries
#include <SPI.h>                                   // SPI library for flash memory
#include <Wire.h>                                  // standard I2C library
//#include <RFM69.h>                                 // radio library
#include <EEPROM.h>                                // ATmega328 internal memory
#include <LowPower.h>                              // low-power sleep
#include "RTClib.h"                                // for Adafruit RTC (PCF8523)
#include <RH_RF95.h>                               // Moteino LoRa library
#include <RHReliableDatagram.h>                    // allows acknowledgements and retries
#include "Adafruit_SHT31.h"

RTC_PCF8523 rtc;
RH_RF95 driver;

// ------- Assign Pins ----------------------------------------------------

#define battPin 24                                  // A0 for reading Vout (to calculate Vin)
#define LED     15
#define SD_CS   3                                  // D3 for SD card ChipSelect 
#define BTsleep 12                                    // Bluetooth KEY (on/off switch) LOW = on, HIGH = off
#define BTstate 13                                    // Bluetooth State (tells if on or off)
#define FLASH_SS 23

// ------- Declare Variables ----------------------------------------------------

char    filename[] = "00-Data.txt";                    // SDcard file name
char    filename2[]= "00-RHT.txt";
char    data[RH_RF95_MAX_MESSAGE_LEN];                 // array to concatenate data arrays

uint8_t GatewayID;
int     FarmID;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

byte    i;                                         // counter
byte    j;                                         // counter
char    a[4];

byte    secs;                                      // time and date values
byte    mins;
byte    hrs;
byte    dow;
byte    days;
byte    mnths;
int     yrs;

int    EEPROMaddr;                                  // ATMega EEPROM memory address (16-bit)
byte   Membyte;                                     // data stored to/read from EEPROM

int    BTorSerialOut;                                // Bluetooth or serial indicator
int    menuinput;                                   // user input in menu
int    numincoming;                                 // number of bytes coming from user input
long   timeout;                                     // msec to wait for user input
int    indata;                                      //
int    input;                                       //
int    incoming[7];                                 // number of bytes coming from user input
int    memdata;
boolean isMenuOn = false;                           // enable/disable 10 sec wait when BT first turned on
boolean firstTime = true;                           // flag for storing data header on SDcard
boolean firstTime2 = true;

float Vout;
float Vin;
float Vref;

float R1 = 22000;
float R2 = 10000;
float resist = (R1 + R2) / R2;

float X = 0.00322;

float Temp;
float RH;
boolean heatON = false;

uint8_t len;
boolean taken = false;

// ------- Initialize ----------------------------------------------------

RHReliableDatagram manager(driver, GatewayID);
// RH_RF95 manager;   // for broadcast transmissions

Adafruit_SHT31 sht31 = Adafruit_SHT31();

File myfile;
File myfile2;

//================================================================================================

//-------------- Setup ----------------------------------------------------------------------
void setup()
{
  pinMode(LED, OUTPUT);                            // transistor pin to sleep Bluetooth
  pinMode(SD_CS, OUTPUT);                          // set ChipSelect pin as output
  pinMode(battPin, INPUT);
  pinMode(BTsleep, OUTPUT);
  pinMode(BTstate, INPUT);

  Wire.begin();
  delay(300);


  BTorSerialOut = EEPROM.read(0);                      // BT or Serial output
  GatewayID = EEPROM.read(1);                      // read gateway ID
  manager.setThisAddress(GatewayID);
  FarmID = EEPROM.read(2);

  digitalWrite(BTsleep, LOW);                     // turn on BT
  delay(15000);                                    // wait 10 sec to connect

  int BTconnected;
  BTconnected = pulseIn(BTstate, LOW);

  if (BTconnected == 0)
  {

    //---- Bluetooth Output -------

    Serial1.begin(9600);
    delay(100);

    if (! rtc.begin()) {
      Serial1.println("Couldn't find RTC");
      while (1);
    }

Serial1.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial1.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  
    if (!SD.begin(SD_CS)) {
      Serial1.println("Initialization Failed");
      return;
    }
    Serial1.println("Initialization Completed");

    delay(10);
    Serial1.println(F("hello"));
    delay(10);

    Serial1.print(F("Currently configured for "));    // show current output setting
    if (BTorSerialOut == 0)                          // output via Bluetooth
    {
      Serial1.println(F("Bluetooth output"));
    }
    else
    {
      Serial1.println(F("Serial output"));            // output via serial
    }

    Serial1.println(F("Desired output: (b) = Bluetooth, (s) = Serial "));    // give option to change output
    long onTime = millis() + 10000;                   // give user 10 sec to input something
    while (millis() < onTime)                        // while in the 10-sec time period
    {
      if (Serial1.available() > 0)                    // look for user input
      {
        char incoming = Serial1.read();
        if (incoming == 'b')                         // if 'b' is entered
        {
          BTorSerialOut = 0;                         // Bluetooth ouput
          EEPROM.write(0, 0);                        // write to EEPROM
          delay(20);                                 // give time for write to complete
        }
        else if (incoming == 's')
        {
          BTorSerialOut = 1;                         // serial output
          EEPROM.write(0, 1);                        // write to EEPROM
          delay(20);
          break;
        }
        while (Serial1.available() > 0) Serial1.println(Serial1.read());
      }
    }
    BTmenu();
  }

  else {

    //---- Serial Output ---------
    digitalWrite(BTsleep, HIGH);

    delay(50);

    Serial.begin(9600);     // initialize serial port
    delay(100);


    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      while (1);
    }

Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  
    if (!SD.begin(SD_CS)) {
      Serial.println("Initialization Failed");
      return;
    }
    Serial.println("Initialization Completed");

    delay(10);
    Serial.println(F("hello"));
    delay(10);

    Serial.print(F("Currently configured for "));    // show current output setting
    if (BTorSerialOut == 0)                          // output via Bluetooth
    {
      Serial.println(F("Bluetooth output"));
    }
    else
    {
      Serial.println(F("Serial output"));            // output via serial
    }

    Serial.println(F("Desired output: (b) = Bluetooth, (s) = Serial "));    // give option to change output
    long onTimeS = millis() + 10000;                   // give user 10 sec to input something
    while (millis() < onTimeS)                        // while in the 10-sec time period
    {
      if (Serial.available() > 0)                    // look for user input
      {
        char incoming = Serial.read();
        if (incoming == 'b')                         // if 'b' is entered
        {
          BTorSerialOut = 0;                         // Bluetooth ouput
          EEPROM.write(0, 0);                        // write to EEPROM
          delay(20);                                 // give time for write to complete
        }
        else if (incoming == 's')
        {
          BTorSerialOut = 1;                         // serial output
          EEPROM.write(0, 1);                        // write to EEPROM
          delay(20);
        }
        while (Serial.available() > 0) Serial.println(Serial.read());
      }
    }
    menu();
  }


}
//================================================================================================

//----------------- Main Loop -------------------------------------------------------------------

void loop()
{
  readRTC();                                       // get current date, time

  //---- Demo Loop -----

//  if ((mins + 1) % 10 == 0 || mins % 10 == 0 || (mins % 10) == 1)
//  {
//    if (taken == false){
//      storeRH();
//    }
//    Serial.println("ON");
//    getRadioData();                                // get data transmitted over radio, save to microSD
//    Serial.println("OFF");
//    delay(5000);
//  }
//
//  else {
//
////if ((mins + 1) % 10 != 0 || mins % 10 != 0 || (mins % 10) != 1){
//    taken = false;
//    sleepytime(7);                                 // low-power sleep for 56 seconds
//
//  }

  //---- Real Loop - Listen every 59th, 0th and 1st Minute ----
  if (mins == 53){                // turn heater on SHT31-D on at the 52nd minute of every hour
    heaterON();
    heatON = true;
    sleepytime(7); 
  } else if (mins == 56){         // turn heater off after 3 minutes, leave 3 minutes for recovery
    heaterOFF();
    heatON = false;
    sleepytime(7);
  } 
 
  else if (mins == 59 || mins == 0 || mins == 1)
  {
      if (taken == false){
      storeRH();                  // measure battery voltage, measure T and RH, store data 
      }
    Serial.println("ON");
    getRadioData();                                // get data transmitted over radio, save to microSD
    Serial.println("OFF");
    delay(5000);
  }

  else {
    taken = false;
    sleepytime(7);                                 // low-power sleep for 56 seconds
  }
}

//================================================================================================

//--------------- Turn SHT31-D Heater On/Off ------------------------------------------------------------

void heaterON(){
  sht31.heater(true);
}

void heaterOFF(){
  sht31.heater(false);
}


//================================================================================================

//--------------- Get data via radio ------------------------------------------------------------

void getRadioData()
{
  digitalWrite(LED, HIGH);
  delay(50);

  if (!manager.init())
    Serial.println("radio failed");
  delay(20);

  //  Serial.print(mnths);                             // date
  //  Serial.print(F("-"));
  //  Serial.print(days);
  //  Serial.print(F("-"));
  //  Serial.print(yrs);
  //  Serial.print(" ");
  //  Serial.print(hrs);                               // time
  //  Serial.print(F(":"));
  //  if (mins < 10) Serial.print(F("0"));
  //  Serial.print(mins);
  //  Serial.print(F(":"));
  //  if (secs < 10) Serial.print(F("0"));
  //  Serial.println(secs);

  //  int timeoutr = 45000;
  //  manager.setTimeout(timeoutr);                      // set timeout to 60 seconds
  //  manager.setRetries(5);

  //------ for Reliable Datagram -------



  if (manager.available())
  {
    Serial.println("Listening...");
    // Wait for a message addressed to us from the client
    len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      //      uint8_t ok[] = "OK";
      //
      //      // Send a reply back to the originator client
      //      if (!manager.sendtoWait(ok, sizeof(ok), from))
      //        Serial.println("sendtoWait failed");
    }
    for (i = 0; i < len; i++) {
      data[i] = buf[i];
    }

   if (len > 0) {
      storeData();
   }
  }

  //------ for Broadcast Transmissions -------

  //  if (manager.available())
  //  {
  //    Serial.println("Listening...");
  //
  //    // Wait for a message addressed to us from the client
  //    uint8_t len = sizeof(buf);
  //    uint8_t from;
  //
  //    if (manager.recv(buf, &len))
  //    {
  //
  //      Serial.print("got request: ");
  //      Serial.println((char*)buf);
  //      Serial.print("RSSI: ");
  //      Serial.println(manager.lastRssi(), DEC);
  //
  //      uint8_t ok[] = "OK";
  //
  //      manager.send(ok, sizeof(ok));
  //      manager.waitPacketSent();
  //      Serial.println("Sent a reply");

  //      if (!manager.sendtoWait(ok, sizeof(ok), from)) {
  //        Serial.println("sendtoWait failed");
  //      }
  //for (i = 0; i < len; i++) {
  //        data[i] = buf[i];
  //      }
  //   }

  //      for (i = 0; i < len; i++) {
  //        data[i] = buf[i];
  //      }

  //    storeData(); // save new array to microSD file

  //Serial.println("It didn't work");
  digitalWrite(LED, LOW);
  delay(50);
  // Serial.println(F("  off"));
}

//================================================================================================

//--------------- Calculate Battery Voltage ------------------------------------------------------------

float calcVin() {
  int Aout;
  Aout = analogRead(battPin); // analogRead returns an integer between 0 and 1023
  //  Serial.print("Aout: ");
  //     Serial.println(Aout);

  Vout = Aout * X;
  //     Serial.print("Vout: ");
  //     Serial.println(Vout);

  Vin = Vout * resist;
  //    Serial.print("Vin: ");
  //    Serial.println(Vin);
  //    Serial.println();
}

//================================================================================================

//--------------- Read SHT31-D Sensor ------------------------------------------------------------

float humidity() {

//  sht31.begin(0x44);
//  delay(20);

  Temp = sht31.readTemperature();
  delay(10);
  RH = sht31.readHumidity();

  //  Serial.print("Temp: ");
  //  Serial.println(Temp);
  //  Serial.print("RH: ");
  //  Serial.println(RH);
}


//===================================================================================================

// ------- Store Humidity to SDcard --------------------------------------------------------

void storeRH(){
   if (!SD.begin(SD_CS))                            // check SD card
  {
  }

  calcVin();
  humidity();

  myfile2 = SD.open(filename2, FILE_WRITE);
  delay(10);
  
  if (myfile2) {
    if (firstTime2 == true)                           // write data output description on startup
    {
      myfile2.println();
      myfile2.print(F("Farm ID number: "));
      myfile2.println(FarmID);
      myfile2.println(F("Data values stored are as follows:"));
      myfile2.println(F(" Timestamp, Gateway ID, Gateway BattV (V), Air Temp (C), RH (%)"));

       firstTime2 = false;
    }
      
      myfile2.print(mnths);
      myfile2.print('/');
      myfile2.print(days);
      myfile2.print('/');
      myfile2.print(yrs);
      myfile2.print(' ');
      myfile2.print(hrs);
      myfile2.print(':');
      myfile2.print(mins);
      myfile2.print(',');
      myfile2.print(GatewayID);
      myfile2.print(',');
      myfile2.print(Vin);
      myfile2.print(',');
      myfile2.print(Temp);
      myfile2.print(',');
      myfile2.println(RH);
   
   delay(100);
    myfile2.close();                                  // close file
    delay(200);                  // give time to complete write to SDcard

  } else {
    Serial.println(F("Error opening file"));
  }
  taken = true;
}
      


//===================================================================================================

// ------- Store data to SDcard --------------------------------------------------------

void storeData()
{

  if (!SD.begin(SD_CS))                            // check SD card
  {
  }

  calcVin();
  humidity();
  //Serial.println("Opening file");

  myfile = SD.open(filename, FILE_WRITE);              // open SDcard data file
  delay(10);

  if (myfile) {
    if (firstTime == true)                           // write data output description on startup
    {
      myfile.println();
      myfile.print(F("Farm ID number: "));
      myfile.println(FarmID);
      myfile.println(F("Data values stored are as follows:"));
      myfile.println(F(" Gateway ID, Gateway BattV (V), Air Temp (C), RH (%), Node ID, Timestamp, "));
      myfile.println(F("   For Acclima TDR310S: Sensor Address, VWC (%), Soil Temp (C), Permittivity, Bulk EC uS/cm, Pore Water EC uS/cm"));
      myfile.println(F("   For CS655: Sensor Address, VWC (m3/m3), EC (dS/m), Soil Temp (C)"));
      myfile.println();

      firstTime = false;                             // set flag so it only writes once
    }
    myfile.print(GatewayID);
    myfile.print(',');
    myfile.print(Vin);
    myfile.print(',');
    myfile.print(Temp);
    myfile.print(',');
    myfile.print(RH);
    myfile.print(',');

//if (len > 0){
    myfile.print(data);
    myfile.println();                 // ending linefeed
  //   }
    delay(100);
    myfile.close();                                  // close file
    delay(200);                  // give time to complete write to SDcard

  } else {
    Serial.println(F("Error opening file"));
  }
}

//======================================================================================

//---------- Read DS1337 real-time clock/calendar -----------------------------------------------------

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

//==========================================================================================================

//---------------- low power sleep mode --------------------------------------------------------------------

void sleepytime(int sleepint)
{
  for (i = 1; i <= sleepint; i++)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   // max low power for 8 sec
  }
}

//=========================================================================================================

//--------------- Menu Options (Serial) ---------------------------------------------------------------------------

void menu()
{
  FarmID = EEPROM.read(2);
  itoa(FarmID, a, 10);                    // convert ID number to character array

  filename[0] = a[0];                     // put into filename[] array
  filename[1] = a[1];
  filename2[0] = a[0];                     // put into filename2[] array
  filename2[1] = a[1];
  
  Serial.println();                                // print out menu
  Serial.println(F("Moteino SDI-12 receiver board:  "));
  Serial.print(F("Farm ID:  "));             // network IDs
  Serial.println(FarmID);
  Serial.print(F("Gateway ID:  "));
  Serial.println(GatewayID);
  Serial.print(F("Filenames: "));
  Serial.print(filename);
  Serial.print(", ");
  Serial.println(filename2);
  Serial.flush();

  readRTC();                                       // get current date, time
  delay(20);
  Serial.print(F("Current date:  "));
  Serial.print(mnths);                             // date
  Serial.print(F("-"));
  Serial.print(days);
  Serial.print(F("-"));
  Serial.println(yrs);
  Serial.flush();

  Serial.print(F("Current time:  "));
  Serial.print(hrs);                               // time
  Serial.print(F(":"));
  if (mins < 10) Serial.print(F("0"));
  Serial.print(mins);
  Serial.print(F(":"));
  if (secs < 10) Serial.print(F("0"));
  Serial.println(secs);
  Serial.flush();

  Serial.println(F("   c  <--  Set clock"));    // set clock
  Serial.println(F("   i  <--  Set ID numbers"));    // set NodeID, board ID number
  Serial.println(F("   d  <--  Download all node data"));       // get all data
  Serial.println(F("   r  <--  Download gateway data"));
  Serial.println(F("   x  <--  Exit"));             // exit, turn off Bluetooth
  Serial.flush();

  timeout = millis() + 30000;                         // wait 5 secs for input
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
  switch (menuinput)                               // take action based on user input
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


    case 105:          // ------ i ----------------------------------------------
      //case 73:           //        I                 // Android apps seem to send 'I' when 'i' pressed
      Serial.println(F("Set ID numbers: "));   // set GatewayID and FarmID
      delay(10);

      Serial.print(F(" FarmID:  "));            // get FarmID
      Serial.flush();
      getinput();
      FarmID = indata;


      Serial.print(F(" GatewayID:  "));            // get GatewayID
      Serial.flush();
      getinput();
      GatewayID = indata;

      EEPROM.write(2, FarmID);
      delay(20);
      EEPROM.write(1, GatewayID);
      delay(20);

      menu();                                      // go back to menu
      break;

    case 100:          // ------ d ----------------------------------------------
      Serial.println(F("Download all node data:"));     // download all data in SD card
      delay(100);

      myfile = SD.open(filename);               // open file
      //  delay(1000);

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

       case 114:          // ------ r ----------------------------------------------
      Serial.println(F("Download gateway data:"));     // download all data in SD card
      delay(100);

      myfile2 = SD.open(filename2);               // open file
      //  delay(1000);

      if (myfile2) {

        while (myfile2.available()) {            // read file and print to screen
          Serial.write(myfile2.read());
        }
        myfile2.close();

      } else {
        Serial.println("Error opening file");
      }
      menu();
      break;

    case 120:          // ------ x ----------------------------------------------
      break;                                       // exit switch(case) routine

    default:          // -------------------------------------------------------
      break;                                       // if no valid user input, leave menu
  }

  Serial.println(F("Exit"));                       // exit
  Serial.println();
  delay(100);

}

//--------------- Menu Options (Bluetooth) ---------------------------------------------------------------------------

void BTmenu()
{
  //  int   BTconnected;
  //  long  BTconnectTime;
  //
  //  if (isMenuOn == false)                           // first time through menu
  //  {
  //    if (BTorSerialOut == 0)                        // if output set to Bluetooth
  //    {
  //      digitalWrite(BTsleep, LOW);                 // turn on power to BT
  //      delay(100);
  //
  //      BTconnectTime = millis() + 30000;            // give 30 secs max to connect
  //      while (millis() < BTconnectTime)
  //      {
  //        BTconnected = pulseIn(BTstate, LOW);       // read Bluetooth Status pin
  //        if (BTconnected == 0)                      // connected
  //        {
  //          isMenuOn = true;                         // after that, don't wait 30 secs
  //          break;                                   // move on
  //        }
  //        delay(1000);
  //      }
  //    }
  //  }

  FarmID = EEPROM.read(2);
  itoa(FarmID, a, 10);                    // convert ID number to character array

  filename[0] = a[0];                     // put into filename[] array
  filename[1] = a[1];
  filename2[0] = a[0];                     // put into filename2[] array
  filename2[1] = a[1];
  delay(100);

  Serial1.println();                                // print out menu
  Serial1.println(F("Moteino SDI-12 receiver board:  "));
  Serial1.print(F("   Farm ID:  "));             // network IDs
  Serial1.println(FarmID);
  Serial1.print(F("   Gateway ID:  "));
  Serial1.println(GatewayID);
  Serial1.print(F("Filenames: "));
  Serial1.print(filename);
  Serial1.print(", ");
  Serial1.println(filename2);
  Serial1.flush();

  readRTC();                                       // get current date, time
  delay(20);
  Serial1.print(F("Current date:  "));
  Serial1.print(mnths);                             // date
  Serial1.print(F("-"));
  Serial1.print(days);
  Serial1.print(F("-"));
  Serial1.println(yrs);
  Serial1.flush();

  Serial1.print(F("Current time:  "));
  Serial1.print(hrs);                               // time
  Serial1.print(F(":"));
  if (mins < 10) Serial1.print(F("0"));
  Serial1.print(mins);
  Serial1.print(F(":"));
  if (secs < 10) Serial1.print(F("0"));
  Serial1.println(secs);
  Serial1.flush();

  Serial1.println(F("   c  <--  Set clock"));    // set clock
  Serial1.println(F("   i  <--  Set ID numbers"));    // set NodeID, board ID number
  Serial1.println(F("   d  <--  Download all node data"));       // get all data
  Serial1.println(F("   r  <--  Download gateway data"));       // get all data
  Serial1.println(F("   x  <--  Exit"));             // exit, turn off Bluetooth
  Serial1.flush();

  timeout = millis() + 30000;                         // wait 5 secs for input
  while (millis() < timeout)
  {
    menuinput = 120;
    if (Serial1.available() > 0)                        // if something typed, go to menu
    {
      menuinput = Serial1.read();               // get user input
      while (Serial1.available() > 0)
      {
        Serial1.read();
      }
      break;
    }
  }
  switch (menuinput)                               // take action based on user input
  {

    case 99:           // ------ c ----------------------------------------------

      Serial1.println(F("Set clock: "));

      Serial1.println(F("  input month: "));
      getinput();
      mnths = indata;
      Serial1.print(F("  input day:    "));
      getinput();
      days = indata;
      Serial1.print(F("  input year:   "));
      getinput();
      yrs = indata;
      Serial1.print(F("  input hour:   "));
      getinput();
      hrs = indata;
      Serial1.print(F("  input minute: "));
      getinput();
      mins = indata;
      //secs = 0;

      rtc.adjust(DateTime(yrs, mnths, days, hrs, mins, secs));
      delay(50);

      BTmenu();
      break;


    case 105:          // ------ i ----------------------------------------------
      //case 73:           //        I                 // Android apps seem to send 'I' when 'i' pressed
      Serial1.println(F("Set ID numbers: "));   // set GatewayID and FarmID
      delay(10);

      Serial1.print(F(" FarmID:  "));            // get FarmID
      Serial1.flush();
      BTgetinput();
      FarmID = indata;


      Serial1.print(F(" GatewayID:  "));            // get GatewayID
      Serial1.flush();
      BTgetinput();
      GatewayID = indata;

      EEPROM.write(2, FarmID);
      delay(20);
      EEPROM.write(1, GatewayID);
      delay(20);

      BTmenu();                                      // go back to menu
      break;

    case 100:          // ------ d ----------------------------------------------
      Serial1.println(F("Download all data:"));     // download all data in SD card
      delay(100);

      myfile = SD.open(filename);               // open file
      delay(100);

      if (myfile) {

        while (myfile.available()) {            // read file and print to screen
          Serial1.write(myfile.read());
        }
        myfile.close();

      } else {
        Serial1.println("Error opening file");
      }
      BTmenu();
      break;

          case 114:          // ------ r ----------------------------------------------
      Serial1.println(F("Download gateway data:"));     // download all data in SD card
      delay(100);

      myfile2 = SD.open(filename2);               // open file
      //  delay(1000);

      if (myfile2) {

        while (myfile2.available()) {            // read file and print to screen
          Serial1.write(myfile2.read());
        }
        myfile2.close();

      } else {
        Serial1.println("Error opening file");
      }
      BTmenu();
      break;

    case 120:          // ------ x ----------------------------------------------
      break;                                       // exit switch(case) routine

    default:          // -------------------------------------------------------
      break;                                       // if no valid user input, leave menu
  }

  Serial1.println(F("Exit"));                       // exit, turn off Bluetooth
  Serial1.println();
  delay(5000);

  digitalWrite(BTsleep, HIGH);                      // turn off power to BT
  Serial1.end();
  delay(100);
}


//======================================================================================

//-------------- Get User Input ---------------------------------------------------------

void getinput()
{
  timeout = millis() + 10000;        // allow user time to input

  indata = 0;
  while (millis() < timeout)                       // lopp while waiting for input
  {
    if (Serial.available() > 0)                    // something came in
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      for (i = 1; i <= numincoming; i++)           // read in everything
      {
        incoming[i] = Serial.read();                  // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)   // if CR or LF, ignore
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

void BTgetinput()
{
  timeout = millis() + 10000;        // allow user time to input

  indata = 0;
  while (millis() < timeout)                       // lopp while waiting for input
  {
    if (Serial1.available() > 0)                    // something came in
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial1.available();            // number of incoming bytes
      for (i = 1; i <= numincoming; i++)           // read in everything
      {
        incoming[i] = Serial1.read();                  // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)   // if CR or LF, ignore
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
  Serial1.println(indata); delay(10);
}


//=================================================================================================

