#include "tpms_toyota.h"

#define RXLED 17
#define OUTP_CD 2
#define OUTP_RX 3

void setup() {
  TCCR1A=0;
  TCCR1B=0;  

  pinMode(OUTP_CD,OUTPUT);
  pinMode(OUTP_RX,OUTPUT);
  
  OCR1A=49; //Value for ORC1A for 50uS
 
  TIMSK1|=(1<<OCIE1A);   //Set the interrupt request

  TCCR1B=(1<<WGM12)|(1<<CS11);   //Set the prescale 1/8 clock
  sei(); //Enable interrupt

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
}

volatile byte send[256];
volatile byte count=0;
volatile byte pos=0;

ISR(TIMER1_COMPA_vect){    //This is the interrupt request
  if(!count) {
      TIMSK1&=~(1<<OCIE1A);   //Set the interrupt request
      digitalWrite(OUTP_CD,0);
      digitalWrite(OUTP_RX,0);
      return;
  }
  digitalWrite(OUTP_CD,1);
  digitalWrite(OUTP_RX,send[pos++]);
  count--;
}

byte codecount=0;
uint32_t codes[]={0xf15454f4, 0xf12672d0, 0xf3fd4559, 0xf1da3451, 0xf0be7886, 0xf78c78b2, 0xf0cf06f0, 0xf63c663f};

void loop() {
  byte u[9];
  if(codecount<(sizeof(codes)/sizeof(uint32_t)))
  	codecount++;
  else
  	codecount=0;

  encode(u,10+(millis() & 0xf),2.1+(millis() & 0xf)/100.,codes[codecount],128);
  byte val=0;
  count=2;
  for(byte x=0;x<255;x++) send[x]=0;  
  
  send[0]=send[1]=0;  
  for(byte x=0;x<8;x++) {
    send[count++]=val;
    val^=1;
  }
  for(byte x=0;x<2;x++) {
    send[count++]=val;
  }
  val^=1;
  for(byte x=0;x<4;x++) {
    send[count++]=val;
  }
  val^=1;
  
  for(byte x=0;x<9;x++) {
    byte w=u[x];
    Serial.print(u[x],HEX);
    Serial.print(' ');
    for(byte b=0;b<8;b++) {
      send[count++]=val;
      if(!(w & 128))
        val^=1;
      send[count++]=val;
      val^=1;
      w<<=1;
    }
  }
  for(byte x=0;x<2;x++) {
    send[count++]=val;
  }

  count+=2;
  pos=0;
  TIMSK1|=(1<<OCIE1A);   //Set the interrupt request
  while(count>0);
  delay(1000);
}
