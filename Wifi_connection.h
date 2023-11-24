#ifndef WIFI_CONNECTION
#define WIFI_CONNECTION

#include "WiFi.h"
#include "WiFiMulti.h"
#include <LovyanGFX.hpp>

WiFiMulti wifiMulti;

const char* ssid = "VIRGIN131";
const char* password = "3D4F2F3311D5";
const char* ssid2 = "SM-G950W2093";
const char* password2 = "5311Fond";

void ConnectWifi(LGFX& lcd){
  char strRetries[2];
  Serial.print("Connecting to Wifi "); 
  
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid2, password2);
    
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString("Connecting", lcd.width() / 2, lcd.height() / 2 - 16);
  lcd.drawString("To", lcd.width() / 2, lcd.height() / 2);
  lcd.drawString("Wifi", lcd.width() / 2, lcd.height() / 2 + 16);
  
  int retries = 0; 
  while (wifiMulti.run() != WL_CONNECTED  && (retries++ < 3)) { // 2 attempts 
    dtostrf(retries,1,0,strRetries);        
    Serial.print("attempts: ");Serial.println(retries);
    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Retry:", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString(strRetries, lcd.width() / 2, lcd.height() / 2);
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected in: "); 
    Serial.print(millis());
    Serial.print(", IP address: "); 
    Serial.println(WiFi.localIP());
  
    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Wifi", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString("Connected", lcd.width() / 2, lcd.height() / 2);    
    delay(500);  
  }
  else
  {
    Serial.print("Failed to connect"); 
     
    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Wifi", lcd.width() / 2, lcd.height() / 2 - 16);
    lcd.drawString("Failed", lcd.width() / 2, lcd.height() / 2);
    lcd.drawString("To", lcd.width() / 2, lcd.height() / 2 + 16);
    lcd.drawString("Connect", lcd.width() / 2, lcd.height() / 2 + 32);
    delay(1000); 
  }
}
#endif
