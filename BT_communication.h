#ifndef BT_COMMUNICATION
#define BT_COMMUNICATION

#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <LovyanGFX.hpp>

BluetoothSerial SerialBT; //Object for Bluetooth

ELM327 myELM327;    //Object for OBD2 device

#define ELM_PORT SerialBT

void ConnectToOBD2(LGFX& lcd){
  char strRetries[2];
  ELM_PORT.setPin("1234");
  ELM_PORT.begin("ESP32", true);    
  
  lcd.fillScreen(TFT_BLACK);
  lcd.drawString("Connecting", lcd.width() / 2, lcd.height() / 2 - 16);
  lcd.drawString("To", lcd.width() / 2, lcd.height() / 2);
  lcd.drawString("OBDII", lcd.width() / 2, lcd.height() / 2 + 16);
  lcd.drawString("Device", lcd.width() / 2, lcd.height() / 2 + 32);
  Serial.println("...Connecting to OBDII...");
  
  int retries = 0;
  while (!ELM_PORT.connect("Android-Vlink") && (retries++ < 3)) // Device name of iCar Vgate pro BT4.0 OBD adapter
  {
    dtostrf(retries,1,0,strRetries);
    Serial.println("Couldn't connect to OBD scanner - Phase 1");
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextSize(2);
    lcd.drawString("Couldn't", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString("connect to", lcd.width() / 2, lcd.height() / 2);
    lcd.drawString("OBDII", lcd.width() / 2, lcd.height() / 2 + 16);
    lcd.drawString("scanner", lcd.width() / 2, lcd.height() / 2 + 32);
    lcd.drawString(" Phase 1", lcd.width() / 2, lcd.height() / 2 + 48); 
    delay(500);    
    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Connecting", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString("Retry:", lcd.width() / 2, lcd.height() / 2);
    lcd.drawString(strRetries, lcd.width() / 2, lcd.height() / 2 + 16);        
  }

  if (!myELM327.begin(ELM_PORT,'6')) // select protocol '6'
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");    
    lcd.fillScreen(TFT_BLACK);    
    lcd.drawString("Couldn't", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString("connect to", lcd.width() / 2, lcd.height() / 2);
    lcd.drawString("OBDII", lcd.width() / 2, lcd.height() / 2 + 16);
    lcd.drawString("scanner", lcd.width() / 2, lcd.height() / 2 + 32);
    lcd.drawString(" Phase 2", lcd.width() / 2, lcd.height() / 2 + 48);
    delay(500);       
    
    //esp_deep_sleep_start();
  }

  else{
  Serial.println("Connected to OBDII");
      
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString("Connected",  lcd.width() / 2, lcd.height() / 2 - 16);
  lcd.drawString("to OBDII", lcd.width() / 2, lcd.height() / 2);

  delay(500);
  lcd.fillScreen(TFT_BLACK);
  }
}
#endif
