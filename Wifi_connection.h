#ifndef WIFI_CONNECTION
#define WIFI_CONNECTION

#include "WiFi.h"
#include "WiFiMulti.h"
#include "TFT_eSPI.h"

WiFiMulti wifiMulti;

const char* ssid = "VIRGIN131";
const char* password = "3D4F2F3311D5";
const char* ssid2 = "SM-G950W2093";
const char* password2 = "5311Fond";

void ConnectWifi(TFT_eSPI& tft){
  char strRetries[2];
  Serial.print("Connecting to Wifi "); 
  
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid2, password2);
    
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.drawString("Connecting", tft.width() / 2, tft.height() / 2 - 50);
  tft.drawString("To", tft.width() / 2, tft.height() / 2);
  tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 + 50);
  
  int retries = 0; 
  while (wifiMulti.run() != WL_CONNECTED  && (retries++ < 20)) { // 2 attempts 
    dtostrf(retries,1,0,strRetries);        
    Serial.print("attempts: ");Serial.println(retries);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Retry", tft.width() / 2, tft.height() / 2 - 50);
    tft.drawString(strRetries, tft.width() / 2, tft.height() / 2);    
    vTaskDelay(1);
    delay(500);
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected in: "); 
    Serial.print(millis());
    Serial.print(", IP address: "); 
    Serial.println(WiFi.localIP());
  
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 - 50);
    tft.drawString("Connected", tft.width() / 2, tft.height() / 2);    
    delay(500);  
  }
  else
  {
    Serial.print("Failed to connect"); 
     
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 - 50);
    tft.drawString("Failed", tft.width() / 2, tft.height() / 2);
    tft.drawString("To", tft.width() / 2, tft.height() / 2 + 50);
    tft.drawString("Connect", tft.width() / 2, tft.height() / 2 + 100);
    delay(1000); 
  }
}
#endif
