enum RFSTATE
{
  RFSTATE_IDLE = 0,
  RFSTATE_RX,
  RFSTATE_TX
};

SPIClass spi;

/**
 * Frequency channels
 */
#define NUMBER_OF_FCHANNELS      10

/**
 * Type of transfers
 */
#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0

/**
 * Type of register
 */
#define CC1101_CONFIG_REGISTER   READ_SINGLE
#define CC1101_STATUS_REGISTER   READ_BURST


/**
 * Buffer and data lengths
 */
#define CCPACKET_BUFFER_LEN        64
#define CCPACKET_DATA_LEN          CCPACKET_BUFFER_LEN - 3

/**
 * Class: CCPACKET
 * 
 * Description:
 * CC1101 data packet class
 */
struct CCPACKET
{
  public:
     //Data length
    unsigned char length;
     // Data buffer
    unsigned char data[CCPACKET_DATA_LEN];
     //* CRC OK flag
    bool crc_ok;
     // Received Strength Signal Indication
    unsigned char rssi;
     // Link Quality Index
    unsigned char lqi;
};

/**
 * Macros
 */

//#define wait_Miso()  delay(3)
IRAM_ATTR void wait_Miso()
{
   unsigned long st = micros();
   while(digitalRead(MISO) == HIGH)
   {
      if (micros() - st > 500)
         break;
   }
}
// Get GDO0 pin state
#define getGDO0state()  digitalRead(PORT_GDO0)
// Wait until GDO0 line goes high
#define wait_GDO0_high()  while(!getGDO0state()) {delay(1);}
// Wait until GDO0 line goes low
#define wait_GDO0_low()  while(getGDO0state()) {delay(1);}



/**
 * PATABLE & FIFO's
 */
#define CC1101_PATABLE           0x3E        // PATABLE address
#define CC1101_TXFIFO            0x3F        // TX FIFO address
#define CC1101_RXFIFO            0x3F        // RX FIFO address

/**
 * Command strobes
 */
#define CC1101_SRES              0x30        // Reset CC1101 chip
#define CC1101_SFSTXON           0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             // Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        // Turn off crystal oscillator
#define CC1101_SCAL              0x33        // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             // setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             // If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             // WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        // Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        // No operation. May be used to get access to the chip status byte

/**
 * CC1101 configuration registers
 */
#define CC1101_IOCFG2            0x00        // GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        // GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        // GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        // RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        // Sync Word, High Byte
#define CC1101_SYNC0             0x05        // Sync Word, Low Byte
#define CC1101_PKTLEN            0x06        // Packet Length
#define CC1101_PKTCTRL1          0x07        // Packet Automation Control
#define CC1101_PKTCTRL0          0x08        // Packet Automation Control
#define CC1101_ADDR              0x09        // Device Address
#define CC1101_CHANNR            0x0A        // Channel Number
#define CC1101_FSCTRL1           0x0B        // Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        // Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        // Frequency Control Word, High Byte
#define CC1101_FREQ1             0x0E        // Frequency Control Word, Middle Byte
#define CC1101_FREQ0             0x0F        // Frequency Control Word, Low Byte
#define CC1101_MDMCFG4           0x10        // Modem Configuration
#define CC1101_MDMCFG3           0x11        // Modem Configuration
#define CC1101_MDMCFG2           0x12        // Modem Configuration
#define CC1101_MDMCFG1           0x13        // Modem Configuration
#define CC1101_MDMCFG0           0x14        // Modem Configuration
#define CC1101_DEVIATN           0x15        // Modem Deviation Setting
#define CC1101_MCSM2             0x16        // Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        // Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        // Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        // Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        // Bit Synchronization Configuration
#define CC1101_AGCCTRL2          0x1B        // AGC Control
#define CC1101_AGCCTRL1          0x1C        // AGC Control
#define CC1101_AGCCTRL0          0x1D        // AGC Control
#define CC1101_WOREVT1           0x1E        // High Byte Event0 Timeout
#define CC1101_WOREVT0           0x1F        // Low Byte Event0 Timeout
#define CC1101_WORCTRL           0x20        // Wake On Radio Control
#define CC1101_FREND1            0x21        // Front End RX Configuration
#define CC1101_FREND0            0x22        // Front End TX Configuration
#define CC1101_FSCAL3            0x23        // Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        // Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        // Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        // Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        // RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        // RC Oscillator Configuration
#define CC1101_FSTEST            0x29        // Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        // Production Test
#define CC1101_AGCTEST           0x2B        // AGC Test
#define CC1101_TEST2             0x2C        // Various Test Settings
#define CC1101_TEST1             0x2D        // Various Test Settings
#define CC1101_TEST0             0x2E        // Various Test Settings

/**
 * Status registers
 */
#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High Byte of WOR Time
#define CC1101_WORTIME0          0x37        // Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        // Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result 


#define CC1101_DEFVAL_IOCFG2     0x0C        // GDO2 Output Pin Configuration - Serial out
#define CC1101_DEFVAL_IOCFG1     0x2E        // GDO1 Output Pin Configuration - not used
#define CC1101_DEFVAL_IOCFG0     0x0E        // GDO0 Output Pin Configuration - Carrier Sense output
#define CC1101_DEFVAL_FIFOTHR    0x0F        // RX FIFO and TX FIFO Thresholds - 64 bytes in FIFO
#define CC1101_DEFVAL_SYNC1      0xD5        // Synchronization word, high byte  11010101 01001111
#define CC1101_DEFVAL_SYNC0      0x4F        // Synchronization word, low byte
#define CC1101_DEFVAL_PKTLEN     0x09        // Packet Length
#define CC1101_DEFVAL_PKTCTRL1   0x00        // Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x12 //0x30        // Packet Automation Control
#define CC1101_DEFVAL_ADDR       0x00        // Device Address
#define CC1101_DEFVAL_CHANNR     0x00        // Channel Number
#define CC1101_DEFVAL_FSCTRL1    0x0F        // Frequency Synthesizer Control (was 0x06)
#define CC1101_DEFVAL_FSCTRL0    0x00        // Frequency Synthesizer Control
// Carrier frequency = 868 MHz (not used)
#define CC1101_DEFVAL_FREQ2_868  0x21        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_868  0x65        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_868  0xCC        // Frequency Control Word, Low Byte
// Carrier frequency = 433.8798 MHz
#define CC1101_DEFVAL_FREQ2_433  0x10        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_433  0xB0        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_433  0x0C        // Frequency Control Word, Low Byte

#define CC1101_DEFVAL_MDMCFG4    0x59        // Modem Configuration (59 = data rate = 20kHz - actual data rate is 10kHz but due to bi-phase coding need to double the rate, RX bandwidth 325kHz)
#define CC1101_DEFVAL_MDMCFG3    0x93        // Modem Configuration (now 93 = data rate = 20kHz)
#define CC1101_DEFVAL_MDMCFG2    0x10        // Modem Configuration (GFSK, No Sync or Manchester coding)

#define CC1101_DEFVAL_MDMCFG1    0x22        // Modem Configuration Channel spacing 200kHz
#define CC1101_DEFVAL_MDMCFG0    0xF8        // Modem Configuration
#define CC1101_DEFVAL_DEVIATN    0x41        // Modem Deviation Setting (+/-28.56kHz)
#define CC1101_DEFVAL_MCSM2      0x07        // Main Radio Control State Machine Configuration
//#define CC1101_DEFVAL_MCSM1      0x30        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM1      0x3C        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_MCSM0      0x18        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x16        // Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0x6C        // Bit Synchronization Configuration
//#define CC1101_DEFVAL_AGCCTRL2   0x43        // AGC Control
#define CC1101_DEFVAL_AGCCTRL2   0xC6        // AGC Control
//#define CC1101_DEFVAL_AGCCTRL1   0x40        // AGC Control
#define CC1101_DEFVAL_AGCCTRL1   0x50        // AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0x80        // AGC Control

#define CC1101_DEFVAL_WOREVT1    0x87        // High Byte Event0 Timeout
#define CC1101_DEFVAL_WOREVT0    0x6B        // Low Byte Event0 Timeout
#define CC1101_DEFVAL_WORCTRL    0xFB        // Wake On Radio Control
#define CC1101_DEFVAL_FREND1     0x56        // Front End RX Configuration
#define CC1101_DEFVAL_FREND0     0x10        // Front End TX Configuration

#define CC1101_DEFVAL_FSCAL3     0xE9        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        // Frequency Synthesizer Calibration

#define CC1101_DEFVAL_RCCTRL1    0x41        // RC Oscillator Configuration
#define CC1101_DEFVAL_RCCTRL0    0x00        // RC Oscillator Configuration

#define CC1101_DEFVAL_FSTEST     0x59        // Frequency Synthesizer Calibration Control

#define CC1101_DEFVAL_PTEST      0x7F        // Production Test
#define CC1101_DEFVAL_AGCTEST    0x3F        // AGC Test

#define CC1101_DEFVAL_TEST2      0x81        // Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        // Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        // Various Test Settings


/**
 * Alias for some default values
 */
#define CCDEF_CHANNR  CC1101_DEFVAL_CHANNR
#define CCDEF_SYNC0  CC1101_DEFVAL_SYNC0
#define CCDEF_SYNC1  CC1101_DEFVAL_SYNC1
#define CCDEF_ADDR  CC1101_DEFVAL_ADDR

/**
 * Macros
 */
// Read CC1101 Config register
#define readConfigReg(regAddr)    readReg(regAddr, CC1101_CONFIG_REGISTER)
// Read CC1101 Status register
#define readStatusReg(regAddr)    readReg(regAddr, CC1101_STATUS_REGISTER)
// Enter Rx state
//#define setRxState()              cmdStrobe(CC1101_SRX)
// Enter Tx state
//#define setTxState()              cmdStrobe(CC1101_STX)
// Enter IDLE state
#define setIdleState()            cmdStrobe(CC1101_SIDLE)
// Flush Rx FIFO
#define flushRxFifo()             cmdStrobe(CC1101_SFRX)
// Flush Tx FIFO
#define flushTxFifo()             cmdStrobe(CC1101_SFTX)
// Disable address check
#define disableAddressCheck()     writeReg(CC1101_PKTCTRL1, 0x04)
// Enable address check
#define enableAddressCheck()      writeReg(CC1101_PKTCTRL1, 0x06)
// Disable CCA
#define disableCCA()              writeReg(CC1101_MCSM1, 0)
// Enable CCA
#define enableCCA()               writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1)
// Set PATABLE single byte
#define setTxPowerAmp(setting)    paTableByte = setting
// PATABLE values
#define PA_LowPower               0x60
#define PA_LongDistance           0xC0




// Select (SPI) CC1101
IRAM_ATTR void cc1101_Select(){

  //delayMicroseconds(150);
  //spi.begin();
  spi.beginTransaction(SPISettings(5000000,MSBFIRST,SPI_MODE0));
  digitalWrite(CC1101_CS, LOW);
}

IRAM_ATTR void cc1101_Deselect(){
  //spi.end();

  //delayMicroseconds(50);
  digitalWrite(CC1101_CS, HIGH);
  spi.endTransaction();


}

/**
 * wakeUp
 * 
 * Wake up CC1101 from Power Down state
 */
IRAM_ATTR void wakeUp(void)
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  cc1101_Deselect();                    // Deselect CC1101
}


/**
 * writeReg
 * 
 * Write single register into the CC1101 IC via SPI
 * 
 * 'regAddr'  Register address
 * 'value'  Value to be writen
 */
IRAM_ATTR void writeReg(byte regAddr, byte value) 
{
  cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  spi.transfer(regAddr);                    // Send register address
  spi.transfer(value);                      // Send value
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * readReg
 * 
 * Read CC1101 register via SPI
 * 
 * 'regAddr'  Register address
 * 'regType'  Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * 
 * Return:
 *  Data byte returned by the CC1101 IC
 */
IRAM_ATTR byte readReg(byte regAddr, byte regType)
{
  byte addr, val;

  addr = regAddr | regType;
  cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  spi.transfer(addr);                       // Send register address
  val = spi.transfer(0x00);                 // Read result
  cc1101_Deselect();                    // Deselect CC1101

  return val;
}

/**
 * setCCregs
 * 
 * Configure CC1101 registers
 */
IRAM_ATTR void setCCregs(void) 
{
  writeReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
  writeReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);

  // Set default synchronization word
  //setSyncWord(syncWord);

  // Set default device address
  //setDevAddress(devAddress);

  // Set default frequency channel
  //setChannel(channel);
  
  writeReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);

//  // Set default carrier frequency = 868 MHz
//  //setCarrierFreq(carrierFreq);
//  writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
//  writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
//  writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);

  // Set default carrier frequency = 433 MHz
  //setCarrierFreq(carrierFreq);
  writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_433);
  writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_433);
  writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_433);

  writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);
  writeReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);
  writeReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
  writeReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
  writeReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
  writeReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
  writeReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
  writeReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
  writeReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
  writeReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
  writeReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);
  
  // Send empty packet
//  CCPACKET packet;
//  packet.length = 0;
//  sendData(packet);
}

/**
 * cmdStrobe
 * 
 * Send command strobe to the CC1101 IC via SPI
 * 
 * 'cmd'  Command strobe
 */     
IRAM_ATTR void cmdStrobe(byte cmd) 
{
  cc1101_Select();                      // Select CC1101
  //wait_Miso();                          // Wait until MISO goes low
  spi.transfer(cmd);                        // Send strobe command
  cc1101_Deselect();                    // Deselect CC1101
}


/**
 * setRxState
 * 
 * Enter Rx state
 */
void setRxState()
{
  cmdStrobe(CC1101_SRX);

}

void setTxState()
{
  cmdStrobe(CC1101_STX);

}


/**
 * reset
 * 
 * Reset CC1101
 */
IRAM_ATTR void CC1101_reset(void) 
{
  //begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
  spi.begin(CC1101_SCKL,CC1101_MISO,CC1101_MOSI,-1);

  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(5);
  cc1101_Select();                      // Select CC1101
  delayMicroseconds(10);
  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(41);
  cc1101_Select();                      // Select CC1101

  wait_Miso();                          // Wait until MISO goes low
  spi.transfer(CC1101_SRES);                // Send reset command strobe
  wait_Miso();                          // Wait until MISO goes low

  cc1101_Deselect();                    // Deselect CC1101
}

IRAM_ATTR void ConfigureCC1101()
{
    setCCregs();                          // Reconfigure CC1101
}

IRAM_ATTR boolean GetCarrierStatus()
{
#ifndef USE_HW_CD
  byte ret;
  ret = readStatusReg(CC1101_PKTSTATUS);
  if ((ret & 0x40) == 0)
  {
    return(false);
  }
  else
  {
    return(true);
  }
#else
    if (digitalRead(CDPin) == HIGH)
    {
      return(true);
    }
    else
    {
      return(false);
    }
#endif
}

IRAM_ATTR void WaitCarrierEnd()
{
   while (GetCarrierStatus() == true)
   {
       //wait for carrier detect to change to low state
       delayMicroseconds(100);
   }
}
