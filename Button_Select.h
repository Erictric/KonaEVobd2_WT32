#ifndef BUTTON_SELECT
#define BUTTON_SELECT

#include <TFT_eSPI.h> 
#include <Adafruit_FT6206.h>

TFT_eSPI tft = TFT_eSPI();
Adafruit_FT6206 ts = Adafruit_FT6206();

// Variables for touch x,y
static int32_t x, y;
TS_Point p;
static int xBox = 20, yBox = 420, margin = 20;
char* BtnAtext = "MAIN";
char* BtnBtext = "BATT";
char* BtnCtext = "POWER";

unsigned long initTouchTime = 0;
unsigned long TouchTime = 0;
bool TouchLatch = false;
bool Btn1Clicked = false;
bool Btn2Clicked = false;
bool Btn3Clicked = false;
bool Btn1SetON = true;
bool Btn2SetON = false;
bool Btn3SetON = false;
bool buttonReleased = true;
int screenSel;

struct RoundedRect {
  int xStart;
  int yStart;
  int xWidth;
  int yHeight;
  byte cornerRadius;
  uint16_t color;
  char* BtnText;
};

RoundedRect btnAgreen = {
  xBox,
  yBox,
  80,
  60,
  4,
  TFT_GREEN,
  BtnAtext
};

RoundedRect btnBgreen = {
  btnAgreen.xStart + btnAgreen.xWidth + margin,
  yBox,
  80,
  60,
  4,
  TFT_GREEN,
  BtnBtext
};

RoundedRect btnCgreen = {
  btnBgreen.xStart + btnBgreen.xWidth + margin,
  yBox,
  80,
  60,
  4,
  TFT_GREEN,
  BtnCtext
};

RoundedRect btnAred = {
  xBox,
  yBox,
  80,
  60,
  4,
  TFT_WHITE,
  BtnAtext
};

RoundedRect btnBred = {
  btnAred.xStart + btnAred.xWidth + margin,
  yBox,
  80,
  60,
  4,
  TFT_WHITE,
  BtnBtext
};

RoundedRect btnCred = {
  btnBred.xStart + btnBred.xWidth + margin,
  yBox,
  80,
  60,
  4,
  TFT_WHITE,
  BtnCtext
};

void drawRoundedRect(RoundedRect toDraw){
  tft.fillRoundRect(
    toDraw.xStart,
    toDraw.yStart,
    toDraw.xWidth, 
    toDraw.yHeight, 
    toDraw.cornerRadius,
    toDraw.color
  );
  int box1TextX = toDraw.xStart + (toDraw.xWidth / 2);
  int box1TextY = toDraw.yStart + (toDraw.yHeight / 2);
  tft.setCursor(box1TextX, box1TextY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(toDraw.xWidth);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK,toDraw.color);  
  tft.drawString(toDraw.BtnText, box1TextX, box1TextY);    
}

int button(){
  if (ts.touched()) {
    p = ts.getPoint();
    x = p.x;
    y = p.y;      
    //Button 1 test
    if ((x >= btnAgreen.xStart && x <= btnAgreen.xStart + btnAgreen.xWidth) && (y >= btnAgreen.yStart && y <= btnAgreen.yStart + btnAgreen.yHeight)) {
      //btn1Clicked = true;
      TouchTime = (millis() - initTouchTime) / 1000;      
      if (TouchTime > 8 & !TouchLatch){
        TouchLatch = true;
        
        Serial.println("Button1 Long Press");
      }            
      if (!Btn1SetON)
      {  
        screenSel = 0;
        Serial.println("Button1 Touched");        
        Serial.println("Button1 set to ON");
        drawRoundedRect(btnAgreen);  //Green        
        Btn1SetON = true;
        if (Btn2SetON){ 
          drawRoundedRect(btnBred);
          Btn2SetON = false;        
        }
        if (Btn3SetON){ 
          drawRoundedRect(btnCred);
          Btn3SetON = false;        
        }
      }      
    } 
        
    //Button 2 test
    if ((x >= btnBgreen.xStart && x <= btnBgreen.xStart + btnBgreen.xWidth) && (y >= btnBgreen.yStart && y <= btnBgreen.yStart + btnBgreen.yHeight)) {
      //btn2Clicked = true;
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 8 & !TouchLatch){
        TouchLatch = true;
        
        Serial.println("Button2 Long Press");
      }
      if (!Btn2SetON)
      {            
        screenSel = 2;
        Serial.println("Button2 Touched");        
        Serial.println("Button2 set to ON");
        drawRoundedRect(btnBgreen);  //Green
        Btn2SetON = true; 
        if (Btn1SetON){ 
          drawRoundedRect(btnAred);
          Btn1SetON = false;        
        }
        if (Btn3SetON){ 
          drawRoundedRect(btnCred);
          Btn3SetON = false;        
        }
      }           
    }

    //Button 3 test
    if ((x >= btnCgreen.xStart && x <= btnCgreen.xStart + btnCgreen.xWidth) && (y >= btnCgreen.yStart && y <= btnCgreen.yStart + btnCgreen.yHeight)) {
      //btn3Clicked = true;
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 8 & !TouchLatch){
        TouchLatch = true;
        
        Serial.println("Button3 Long Press");
      }
      if (!Btn3SetON)
      {            
        screenSel = 3;
        Serial.println("Button3 Touched");        
        Serial.println("Button3 set to ON");
        drawRoundedRect(btnCgreen);  //Green
        Btn3SetON = true;              
        if (Btn1SetON){ 
          drawRoundedRect(btnAred);
          Btn1SetON = false;        
        }
        if (Btn2SetON){ 
          drawRoundedRect(btnBred);
          Btn2SetON = false;       
        }
      }      
    }
  }
  else{
    initTouchTime = millis();
    TouchLatch = false;
    //btn1Clicked = false;
    //btn2Clicked = false;
    //btn3Clicked = false;
    buttonReleased = true;      
  }
  return screenSel;
}

#endif
