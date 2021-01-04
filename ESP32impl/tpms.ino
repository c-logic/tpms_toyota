//Changes:
//V1.0 - Originally published version
//V2.0 - Altered to allow support for spare tyre reporting (although not displayed on LCD display)
//V3.0 - Added #define in globals.h to switch between BAR and PSI
//V4.0 - Now uses synchronous data from CC1101 and option of using either the SPI or the hardware pin for the CD signal (#define USE_HW_CD in globals.h). Also tideied up debug information if required (globals.h #defines)
//V5.0 - Corrected possible sync error when looking for start of data stream. Improves hit rate on valid TPMS detection


#define VERSION "5.0"

#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>

#include "globals.h"
#include "CC1101.h"
#include "Display.h"
#include "ToyotaRead.h"

IRAM_ATTR void SendDebug(String Mess)
{
  Serial.println(Mess);
}

IRAM_ATTR int DecodeBitArray()
{
  //convert 1s and 0s array to byte array
  int n = 0;
  byte b = 0;

  RXByteCount = 0;
  for (int i = 0; i < BitCount; i++)
  {
    b = b << 1;
    b = b + IncomingBits[i];
    n++;
    if (n == 8)
    {
      RXBytes[RXByteCount] = b;
      RXByteCount++;
      n = 0;
      b = 0;
    }

  }
  return (RXByteCount);
}

void setup() {

  byte resp;
  unsigned int t;
  int LEDState = LOW;
  int i;
  int mcount;

  //SPI CC1101 chip select set up
  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);

  WiFi.mode(WIFI_OFF);
  InitDisplay();
  ShowTitle();
  
  Serial.begin(115200);
  Serial.println(F("SSD1306 initialised OK"));

  pinMode(LED_RX, OUTPUT);
  pinMode(RXPin, INPUT);
  pinMode(CDPin, INPUT);

  SPI.begin();
  delay(2000);
  Serial.println("Starting...");
  
  //initialise the CC1101
  Serial.print(F("Resetting CC1101 "));
  for(int retrycount=0; retrycount< 5; retrycount++)
  {
     Serial.print(F("."));
     CC1101_reset();
     delay(100);
     if (readConfigReg(0) == 0x29)
        break;
     retrycount++;
  }
  Serial.println("");

  if (readConfigReg(0) == 0x29)
  {
    Serial.println(F("CC1101 reset successful"));
  }
  else
  {
    Serial.println(F("CC1101 reset failed. Try rebooting"));
  }

  ConfigureCC1101();
  Serial.println(F("CC1101 configured"));

  setIdleState();
  digitalWrite(LED_RX, LED_OFF);

  resp = readStatusReg(CC1101_PARTNUM);
  Serial.print(F("Part no: "));
  Serial.println(resp, HEX);

  resp = readStatusReg(CC1101_VERSION);
  Serial.print(F("Version: "));
  Serial.println(resp, HEX);

  digitalWrite(LED_RX, LED_ON);
  LEDState = HIGH;

  pinMode(DEBUGPIN, OUTPUT);
  digitalWrite(DEBUGPIN, LOW);

#ifndef USE_PROGMEMCRC
  CalulateTable_CRC8();
#endif

  InitTPMS();

  UpdateDisplay();

  digitalWrite(LED_RX, LED_OFF);
  setRxState();
}

IRAM_ATTR void loop() {
  // put your main code here, to run repeatedly:
  int i;
  static long lastts = millis();
  float diff;
  int RXBitCount = 0;
  int ByteCount = 0;
  byte crcResult;
  boolean TPMS_Changed;


  TPMS_Changed = Check_TPMS_Timeouts();

  InitDataBuffer();
  while (GetCarrierStatus()); //wait for carrier status to go low
  while (!GetCarrierStatus()){
    //wait for carrier status to go high  looking for rising edge
  }
  
  RXBitCount = ReceiveMessage();
  if (RXBitCount == EXPECTEDBITCOUNT )
  {
    ByteCount = DecodeBitArray();
//      crcResult = Compute_CRC8(ByteCount);
    if (!Compute_CRC8(ByteCount))
    {
      //decode the message...
      DecodeTPMS();
      UpdateDisplay();
      TPMS_Changed = true;  //indicates the display needs to be updated.
    }
  }
}
