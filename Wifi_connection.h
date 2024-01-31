#ifndef WIFI_CONNECTION
#define WIFI_CONNECTION

#include "WiFi.h"
//#include "WiFiMulti.h"
#include <WiFiClient.h>
#include "TFT_eSPI.h"

//WiFiMulti wifiMulti;

const char* ssid2 = "VIRGIN131";
const char* password2 = "3D4F2F3311D5";
const char* ssid = "WT32-SC01";
const char* password = "5311Fond";

void ConnectWifi(TFT_eSPI& tft, uint16_t Wifi_select){
  
  char strRetries[2];
  Serial.println("Connecting to Wifi "); 

  Serial.print("Wifi_Select=  ");
  Serial.println(Wifi_select);
  
  //wifiMulti.addAP(ssid, password);
  //wifiMulti.addAP(ssid2, password2);
  
  WiFi.disconnect();
  if (Wifi_select == 0){
    WiFi.begin(ssid, password);
  }
  else{
    WiFi.begin(ssid2, password2);
  }
    
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.drawString("Connecting", tft.width() / 2, tft.height() / 2 - 50);
  tft.drawString("To", tft.width() / 2, tft.height() / 2);
  tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 + 50);
  
  int retries = 0; 
  while (WiFi.status() != WL_CONNECTED  && (retries++ < 5)) { // 2 attempts     
    delay(4000);
    dtostrf(retries,1,0,strRetries);        
    Serial.print("attempts: ");Serial.println(retries);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Retry", tft.width() / 2, tft.height() / 2 - 50);
    tft.drawString(strRetries, tft.width() / 2, tft.height() / 2);    
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
