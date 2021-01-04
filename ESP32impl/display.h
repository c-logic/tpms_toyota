#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

void InitDisplay() {
#if OLED_RESET>=0
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW); // sets the digital pin 13 on
  delay(10);            // waits for a second
  digitalWrite(OLED_RESET, HIGH);  // sets the digital pin 13 off
  delay(10);            // waits for a second
#endif

  Wire.begin(OLED_SDA,OLED_SCL);
  Wire.setClock(400000L);
  display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS);
  display.display();
  delay(500);
}

IRAM_ATTR void ShowTitle()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextSize(1);    // Normal 1:1 pixel scale
  //display.setTextColor(WHITE, BLACK);       // Draw white text
  
  display.setCursor(0, 0);
  display.println("Toyota TPMS Monitor");
  display.print("  (JSM Solutions ");
  display.print(VERSION);
  display.println(")");  
  display.display();
}

IRAM_ATTR char DisplayTimeoutBar(unsigned long TimeSinceLastUpdate)
{
  int HowCloseToTimeout;
  HowCloseToTimeout = (int)(TimeSinceLastUpdate/(TPMS_TIMEOUT/5));

  switch(HowCloseToTimeout)
  {
    case 0: 
       //return(FONTBAR_7);
       return('5');
       break;
    case 1: 
       //return(FONTBAR_5);
       return('4');
       break;
    case 2: 
       //return(FONTBAR_3);
       return('3');
       break;
    case 3: 
       //return(FONTBAR_2);
       return('2');
       break;
    case 4: 
       //return(FONTBAR_1);
       return('1');
       break;
    default: 
       //return(FONTBAR_0);
       return('0');
       break;
                  
  }
}

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_winter[] =
{ 0x09, 0x20, 0x05, 0x40, 0x23, 0x88, 0x11, 0x10, 0x8b, 0xa2, 0x45, 0x44, 0x2b, 0xa8, 0xfe,  0xfe,
  0x2b, 0xa8, 0x45, 0x44, 0x8b, 0xa2, 0x11, 0x10, 0x23, 0x88, 0x05, 0x40, 0x09, 0x20, 0x00, 0x00
 };

static const unsigned char PROGMEM logo_summer[] =
{ 0x01, 0x00, 0x01, 0x00, 0x20, 0x08, 0x13, 0xd0, 0x0f, 0xe0, 0x0f, 0xf0, 0x1f, 0xf0, 0xdf, 0xf6, 
  0x1f, 0xf0, 0x0f, 0xf0, 0x0f, 0xe0, 0x17, 0xd0, 0x20, 0x08, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00
};
  
IRAM_ATTR void UpdateDisplay() {
  int i, x = 0, y = 0;
  char s[12];
  display.clearDisplay();

  for (i = 0; i < 4; i++) {
    switch (i) {
      case 0:
        x = 0; y = 0; break;
      case 1:
        x = 64; y = 0; break;
      case 2:
        x = 0; y = 32; break;
      case 3:
        x = 64; y = 32; break;
    }

   // display.setFont(System5x7);

    display.setCursor(x, y);

    switch(TPMS[i].Typename){
      case 'w':
      case 'W':
        display.drawBitmap(x,y+4,logo_winter,LOGO_WIDTH,LOGO_HEIGHT, 1);
        break;
      case 's':
      case 'S':
        display.drawBitmap(x,y+4,logo_summer,LOGO_WIDTH,LOGO_HEIGHT, 1);
    }

//    sprintf(s,"%c ",TPMS[i].Typename);
//    display.print(s);

    
    display.setCursor(x+16, y);
    display.setTextSize(2);
    sprintf(s,"%3.2f ",TPMS[i].TPMS_Pressure);
    display.print(s);
    display.setCursor(x+22, y+16);
    display.setTextSize(1);
    sprintf(s,"% 2.0f\xF8""C ", TPMS[i].TPMS_Temperature);
    display.print(s);
    display.print(DisplayTimeoutBar(millis() - TPMS[i].lastupdated));
    display.setCursor(x, y+24);
    sprintf(s,"  %08X", TPMS[i].TPMS_ID);
    display.print(s);
    //display vertical bar showing how long since last update 7 bars = recent 1 bar = nearing timeout (at timeout it will be removed from display altogether)
  }
  display.dim(true);
  display.display();
}
