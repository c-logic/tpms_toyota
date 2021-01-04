IRAM_ATTR void ClearTPMSData(int i)
{
  if (i < TYRECOUNT_ON_DISPLAY) {
    TPMS[i].TPMS_ID = TPMS[i].lastupdated = 0;
    TPMS[i].Typename='-';
  }
}

IRAM_ATTR void PulseDebugPin(int width_us)
{
  digitalWrite(DEBUGPIN, HIGH);
  delayMicroseconds(width_us);
  digitalWrite(DEBUGPIN, LOW);
}

IRAM_ATTR int GetPreferredIndex(unsigned long ID)
{
  for (int i = 0; i  < TYRECOUNT_IN_DBASE; i++)
  {
    if (IDLookup[i][0] == ID)
      return (i);
  }
  return (-1);
}

IRAM_ATTR void PrintTimings(byte StartPoint, byte Count)
{
  for (int i = 0; i < Count; i++)
  {
    Serial.print(Timings[StartPoint + i]);
    Serial.print(F(","));
  }
  Serial.println(F(""));
}

IRAM_ATTR void PrintData(byte Count)
{
  byte hexdata;
  for (int i = 0; i < Count; i++)
  {
    Serial.print(IncomingBits[i]);
    hexdata = (hexdata << 1) + IncomingBits[i];
    if ((i + 1) % 8 == 0)
    {
      Serial.print(F(" ["));
      Serial.print(hexdata, HEX);
      Serial.print(F("] "));
      hexdata = 0;
    }
  }
  Serial.println(F(""));
}

IRAM_ATTR void InitTPMS()
{
  for (int i = 0; i < TYRECOUNT_ON_DISPLAY; i++)
  {
    ClearTPMSData(i);
  }
  UpdateDisplay();
}

IRAM_ATTR void UpdateTPMSData(int index, unsigned long ID, unsigned int status, float Temperature, float Pressure, char Typename)
{
  if (index < TYRECOUNT_ON_DISPLAY)
  {
    TPMS[index].TPMS_ID = ID;
    TPMS[index].TPMS_Status = status;
    TPMS[index].lastupdated = millis();
    TPMS[index].TPMS_Temperature = Temperature;
    TPMS[index].Typename=Typename;
    #ifdef USE_BAR
      TPMS[index].TPMS_Pressure = Pressure/PSI2BAR;
    #else
       TPMS[index].TPMS_Pressure = Pressure;
    #endif
  }
}

IRAM_ATTR void DisplayStatusInfo()
{
  Serial.print (F("FreqOffset: "));
  Serial.print (FreqOffset);
  Serial.print (F("  DemodLinkQuality: "));
  Serial.print (DemodLinkQuality);
  Serial.print (F("  RSSI: "));
  Serial.println (RSSIvalue);
}

IRAM_ATTR boolean Check_TPMS_Timeouts()
{
  boolean ret = false;
  //clear any data not updated in the last 5 minutes
  for (int i = 0; i < TYRECOUNT_ON_DISPLAY; i++)
  {
    if ((TPMS[i].TPMS_ID != 0) && (millis() - TPMS[i].lastupdated > TPMS_TIMEOUT))
    {
      ClearTPMSData(i);
      ret = true;
    }
  }
  return(ret);
}

IRAM_ATTR void DecodeTPMS()
{
  unsigned long id = 0;
  unsigned int status, pressure1, pressure2, temp;
  float realpressure;
  float realtemp;
  bool IDFound = false;
  int prefindex;

  for (int i = 0; i < 4; i++)
  {
    id = id << 8;
    id = id + RXBytes[i];
  }

  // id = (unsigned)RXBytes[0] << 24 | RXBytes[1] << 16 | RXBytes[2] << 8 | RXBytes[3];
  status = (RXBytes[4] & 0x80) | (RXBytes[6] & 0x7f); // status bit and 0 filler
  pressure1 = (RXBytes[4] & 0x7f) << 1 | RXBytes[5] >> 7;
  temp = (RXBytes[5] & 0x7f) << 1 | RXBytes[6] >> 7;
  pressure2 = RXBytes[7] ^ 0xff;

  if (pressure1 != pressure2)
  {
    Serial.println(F("Pressure check mis-match"));
    return;
  }

  realpressure = pressure1 * 0.25 - 7.0;
  realtemp = temp - 40.0;
#ifdef SHOWVALIDTPMS
  Serial.print(F("ID: "));
  Serial.print(id, HEX);
  Serial.print(F("   Status: "));
  Serial.print(status);
  Serial.print(F("   Temperature: "));
  Serial.print(realtemp);
  Serial.print(F("   Tyre Pressure: "));
  Serial.print(realpressure);
  Serial.print(F(" (psi)  "));
  Serial.print(realpressure/PSI2BAR);
  Serial.print(F(" (bar)"));
  Serial.println(F(""));
#endif

  //DisplayStatusInfo();

  //update the array of tyres data
  for (int i = 0; i < 4; i++)
  { //find a matching ID if it already exists
    if (id == TPMS[i].TPMS_ID)
    {
      UpdateTPMSData(i, id, status, realtemp, realpressure, TPMS[i].Typename);
      IDFound = true;
      break;
    }
  }

  if (IDFound == false) {
    prefindex = GetPreferredIndex(id);
    if (prefindex >= 0) { //not found a specified index, so use the next available one..
      for (byte i = 0; i < 4; i++) {
        if (TPMS[i].TPMS_ID == 0) {
          UpdateTPMSData(i, id, status, realtemp, realpressure, IDLookup[prefindex][1]);
          break;
        }
      }
    }
  }

  #ifdef SHOWVALIDTPMS
     Serial.println(F(""));
  #endif
  //UpdateDisplay();
}


#ifndef USE_PROGMEMCRC
IRAM_ATTR void CalulateTable_CRC8()
  {
    const byte generator = 0x07;
  
    /* iterate over all byte values 0 - 255 */
    for (int divident = 0; divident < 256; divident++)
    {
      byte currByte = (byte)divident;
      /* calculate the CRC-8 value for current byte */
      for (byte bit = 0; bit < 8; bit++)
      {
        if ((currByte & 0x80) != 0)
        {
          currByte <<= 1;
          currByte ^= generator;
        }
        else
        {
          currByte <<= 1;
        }
      }
      /* store CRC value in lookup table */
      crctable[divident] = currByte;
      Serial.print("0x");
      if (currByte < 16)
         Serial.print("0");
      Serial.print(currByte,HEX);
      Serial.print(", ");
    }
  }
#endif

IRAM_ATTR byte Compute_CRC8( int bcount)
{
  byte crc = 0x80;
  for (int c = 0; c < bcount; c++)
  {
    byte b = RXBytes[c];
    /* XOR-in next input byte */
    byte data = (byte)(b ^ crc);
    /* get current CRC value = remainder */
    #ifdef USE_PROGMEMCRC
        crc = (byte)(pgm_read_byte(&crctable2[data]));
    #else
        crc = (byte)(crctable[data]);
    #endif

  }
  return crc;
}

IRAM_ATTR void ClearRXBuffer()
{
  memset(RXBytes,0,sizeof(RXBytes));
}

IRAM_ATTR void EdgeInterrupt()
{
  unsigned long ts = micros();
  unsigned long BitWidth;

  if (TimingsIndex == 255)
  {
    return;
  }

  if (WaitingFirstEdge)
  {
    if (digitalRead(RXPin) == LOW)
    {
      FirstEdgeIsHighToLow = true;
    }
    else
    {
      FirstEdgeIsHighToLow = false;
    }
  }
  WaitingFirstEdge = false; 

  BitWidth = ts - LastEdgeTime_us;
  if (BitWidth <= 12)  //ignore glitches
  {
    return;
  }
  if (BitWidth > 8000)
    BitWidth = 8000;

  LastEdgeTime_us = ts;
  //    if ((BitWidth >= 38) && (BitWidth <= 250))
  //    {//ignore out of spec pulses
  Timings[TimingsIndex++] = (unsigned int)BitWidth;
  //    }

  //    digitalWrite(DEBUGPIN,HIGH);
  //    delayMicroseconds(3);
  //    digitalWrite(DEBUGPIN,LOW);
}
/*
IRAM_ATTR bool IsValidSync(unsigned int Width)
{
  return (Width >= 175) && (Width <= 750);
}

IRAM_ATTR bool IsValidShort(unsigned int Width)
{
  return (Width >= 40) && (Width <= 70);
}

IRAM_ATTR bool IsValidLong(unsigned int Width)
{
  return (Width >= 80) && (Width <= 120);
}
*/

#define IsValidSync(Width) ((Width >= 175) && (Width <= 750))

#define IsValidShort(Width) ((Width >= 40) && (Width <= 70))

#define IsValidLong(Width) ((Width >= 80) && (Width <= 120))

IRAM_ATTR int ValidateBit()
{
  unsigned int BitWidth = Timings[CheckIndex];
  unsigned int BitWidthNext = Timings[CheckIndex + 1];

  if (IsValidLong(BitWidth))
    return (1);

  if (IsValidShort(BitWidth))
    return (0);

  if (IsValidSync(BitWidth))
    return (2);

  return (-1);
}

IRAM_ATTR void ValidateTimings()
{
  unsigned int BitWidth;
  unsigned int BitWidthNext;
  unsigned int BitWidthNextPlus1;
  unsigned int BitWidthPrevious;
  unsigned int diff = TimingsIndex - CheckIndex;
  //unsigned long tmp;
  bool WaitingTrailingZeroEdge = false;
  int ret;

  StartDataIndex = 0;

  if (diff < EXPECTEDBITCOUNT)
  { //not enough in the buffer to consider a valid message
    #ifdef SHOWDEBUGINFO
      Serial.print(F("Insufficient data in buffer ("));
      Serial.print(diff);
      Serial.println(")";
    #endif
    return;
  }

  SyncFound = false;

  while ((diff > 0) && (BitCount < EXPECTEDBITCOUNT))
  { //something in buffer to process...
    diff = TimingsIndex - CheckIndex;

    BitWidth = Timings[CheckIndex];

    if (SyncFound == false)
    {
      if (IsValidSync(BitWidth))
      {
        SyncFound = true;
        BitIndex = 0;
        BitCount = 0;
        WaitingTrailingZeroEdge = false;
        StartDataIndex = CheckIndex + 1;
      }

    }
    else
    {
      ret = ValidateBit();
      switch (ret)
      {
        case -1:
          //invalid bit
//          BitIndex = 0;
//          BitCount = 0;
//          WaitingTrailingZeroEdge = false;
//          StartDataIndex = CheckIndex + 1;
          SyncFound = false;
          break;

        case 0:
          if (WaitingTrailingZeroEdge)
          {
            //BitTimings[BitIndex] = BitWidth;
            IncomingBits[BitIndex++] = 0;
            BitCount++;
            WaitingTrailingZeroEdge = false;
          }
          else
          {
            WaitingTrailingZeroEdge = true;
          }
          break;

        case 1:
          if (WaitingTrailingZeroEdge)
          { //shouldn't get a long pulse when waiting for the second short pulse (i.e. expecting bit = 0)
            //try to resync from here?
            BitIndex = 0;
            BitCount = 0;
            WaitingTrailingZeroEdge = false;
            CheckIndex--;  //recheck this entry
            StartDataIndex = CheckIndex + 1;
            SyncFound = false;
          }
          else
          {
            //BitTimings[BitIndex] = BitWidth;
            IncomingBits[BitIndex++] = 1;
            BitCount++;
          }
          break;

        case 2:
          SyncFound = true;
          BitIndex = 0;
          BitCount = 0;
          WaitingTrailingZeroEdge = false;
          StartDataIndex = CheckIndex + 1;
          break;
      }
    }
    CheckIndex++;
  }
}

IRAM_ATTR void InitDataBuffer()
{
  BitIndex = 0;
  BitCount = 0;
  ValidBlock = false;
  //WaitingTrailingZeroEdge = false;
  WaitingFirstEdge  = true;
  CheckIndex = 0;
  TimingsIndex = 0;
  SyncFound = false;
  //digitalWrite(DEBUGPIN, LOW);

}

IRAM_ATTR void UpdateStatusInfo()
{
  FreqOffset = readStatusReg(CC1101_FREQEST);
  DemodLinkQuality = readStatusReg(CC1101_LQI);
  RSSIvalue = readStatusReg(CC1101_RSSI);
}

IRAM_ATTR int ReceiveMessage()
{
  //Check bytes in FIFO
  int FIFOcount;
  int resp;

  //set up timing of edges using interrupts...
  LastEdgeTime_us = micros();
  CD_Width = micros();

  attachInterrupt(digitalPinToInterrupt(RXPin), EdgeInterrupt, CHANGE);
  while (GetCarrierStatus());
  delayMicroseconds(500);  //there is a delay on the serial data stream so ensure we allow a bit of extra time after CD finishes to ensure all the data is captured
  detachInterrupt(digitalPinToInterrupt(RXPin)); 
  
  CD_Width = micros() - CD_Width;
  if ((CD_Width >= CDWIDTH_MIN) && (CD_Width <= CDWIDTH_MAX))
  {
      PulseDebugPin(100);
    #ifdef SHOWDEGUGINFO
       Serial.println(F("Checking.."));
    #endif
    digitalWrite(LED_RX,LED_ON);
    CheckIndex = 0;
    ValidateTimings();
    #ifdef SHOWDEGUGINFO
       Serial.print(F("CD_Width="));
       Serial.println(CD_Width);
       Serial.print(F("TimingsIndex="));
       Serial.println(TimingsIndex);
       Serial.print(F("Checking complete. Bitcount: "));
       Serial.print(BitCount);
       Serial.print(F("  StartDataIndex: "));
       Serial.println(StartDataIndex);
       if (BitCount <= EXPECTEDBITCOUNT)
       {
          PrintTimings(0,TimingsIndex+1);
          PrintData(BitCount);
       }
    #endif
    digitalWrite(LED_RX,LED_OFF);
    return (BitCount);
  }
  else
  {
    #ifdef SHOWDEGUGINFO
      if (CD_Width > CDWIDTH_MIN)
      {
       Serial.print(F("CD_Width*="));
       Serial.println(CD_Width);        
      }
    #endif
    return (0);
  }
}
