
/*  KonaEvObd for Hyundai Kona EV + OBD Vgate iCar Pro BT4.0 + WT32-SC01 3.5" display
    Version: v3.01

    SafeString by Matthew Ford: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
    Elmduino by PowerBroker2: https://github.com/PowerBroker2/ELMduino
    https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/ now requires a paid membership to use the IFTTT applet

  notes:
  ELMduino library used is version 2.41, problem not resolved with newer version
  ELMduino.h needs to be modified as follow: bool begin(Stream& stream, char protocol='6', uint16_t payloadLen = 240);
  For TFT_eSPI library, comment: //#include <User_Setup.h> and add: #include <User_Setups/WT32-SC01-User_Setup.h>
  in User_Setup_Select.h file. WT32-SC01-User_Setup.h needs to be downloaded for github and saved in the User_Setups folder.
*/
#include "Arduino.h"
#include "SafeString.h"
#include "ELMduino.h"
#include "EEPROM.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "BT_communication.h"
#include "Wifi_connection.h"
#include "FreeRTOSConfig.h"
#include "WiFi.h"
#include "Free_Fonts.h"
#include <Adafruit_FT6206.h>
#include <ESP_Google_Sheet_Client.h>
#include <TimeLib.h>

#define DEBUG_PORT Serial

// Google Project ID
#define PROJECT_ID "konaev-datalogging"

// Service Account's client email
#define CLIENT_EMAIL "konaev-datalogging@konaev-datalogging.iam.gserviceaccount.com"

// Service Account's private key
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCnnCHE5paKUCH3\nMvz65J3qNo+e6jsdwciuVyGU8nUps2bZ/upY0mSqHRmqrGiHA76npNWidjcEOWJ1\n0rsE75siuUdS6v0eUAnK0a4o4SsXrqUqXNK9qYxGhc/QBV2kfCDkahRGBKr3JleH\nMSTMb0kR3I7JS9TfgXQ7QfmJ1eh3iW+y3citMTNLjqGZFkeFxNt9xFgH8LFDHuUi\nUAzgmwI65qI0KkKFw7kyv2sd8WwltD3hkS+k/JBBxQTv96APmPZOXMF0f+WmvdJy\nzpMVKXG0BQBvAF73mvzUpfszuHwj1+zwqMw4GRD2ScNlBRpWgBIcj53Fj35COs7e\nz8V+V1XHAgMBAAECggEAUXMSh/REMJeLQezhuexx/tifx2ps6uN6KZqG47JFFEwt\njX8Ok7Y+G9rDV8irjPzZX+8+r9HBn4hhW/9ZSadEXMXrrpQqB9p+P7TQbOYrAjmo\n4qKz+F3VoImzOJP68w0tEMKp8nKfQDY+L6DGkJ/9wrPLIW/71Nc8S/WeFYjBDKEQ\nJDYwnW1ZRCqMIjMUxhn1q8n0EyB/Sf9vURchY3iPBdXVC8v4NdjZ1PxL42u9OzFc\nsvnIce6xCfB6hdR+hp06OylUC683Xln61tjZRXTp4QlJ5sq/LyUn0W5htQh/fcuf\ndUkqyGK4wffpTsUIp4PksDBE0HNQofNgtZ4U4rx+1QKBgQDU3MMqEZPhaG68FnQY\n/b64eY4Dzfk0WE0PSRJuxJXiw87yVPB2kkEsSU/qSN70Di262ABHTo66tVv9hNVn\nmYsxZuNZ86YvKrPVW8+RfjNej6LXomm+ffZtHAWMzDtAx7xY+lHMOW3by46iAg5S\nW0YwheDjH1u8GAIMitU/sXEa7QKBgQDJk69PPqXUtN6hj3wsPzsv+vFr9XnMMfLo\nDBA1mkQdewhPiCWLSuPQAUGMnFc8UMkbKN0D0Jkg9G0DI77NzdJwjRQ3Z/AH4Di+\n9Yj/tt0hE7HOtIZG8gOEqvbHFFwAVg7X3+huuNkW1TS/bzAAoowPKDhv9aPEgH8t\nahbzOjJ5AwKBgBH4lW2O0Fpec8LjbmfRvHFcqdW+ZQS7U74voCPD6xebCnTBIRAR\npvjzM5EHF/Oo4sl8hQGAK2Kt/xc3SMEXYH4KPrWQcX5X75jayHpzGikonUnxR1Yy\n0kRB8mIBuBrvAgLNF2zTiGffFqqs28KuPA3Kr8LdGeSWbk3axsg61d69AoGAbhjm\n0J6EDqh3TMDE7pnepvcl83RRAPFrHciw9cX7XCq9wEq5TtopkYuOFNGzZ/Mr1FS+\nWn4NlQ1LmUJlzZyUSvsTRqvTU0npVIthN2HWZ2GNZTv+dzNqLoT+Yn/BPEHEu63F\nEuyNTcZHmCOPkVk2rHSoVqZQu1v/mntua4ym0qcCgYASTSMXNL2uOXff1fiSQBP/\ngzO2p+guUWcfR5Q/pQrqGSxQzhi3ebntVnVv93jXCOq0REn0gehzgAK+M/0I9SpQ\njYeVZCLLFgG+yJaIfnXt7+GUHrzRM2otR/jSqwiRiZzhjwQnG+xdrf3ViiKIRw05\nYiVzz6jcIbYZ6k58c8NiWw==\n-----END PRIVATE KEY-----\n";

// The ID of the spreadsheet where you'll publish the data
const char spreadsheetId[] = "1Ho5H6qfHyVTo3fvcvrGUEGIylHTAWWfTAO8dBwZUqnI";

// Token Callback function
void tokenStatusCallback(TokenInfo info);

TaskHandle_t Task1;

TFT_eSPI tft = TFT_eSPI();
Adafruit_FT6206 ts = Adafruit_FT6206();

#define Threshold 40 /* threshold for touch wakeup - Greater the value[, more the sensitivity */
#define ST7789_DISPOFF    0x28
#define ST7789_DISPON   0x29
#define ST7789_SLPIN    0x10
#define ST7789_SLPOUT   0x11

int ledBacklight = 150; // Initial TFT backlight intensity on a scale of 0 to 255. Initial value is 120.
bool low_backlight = false;
bool display_off = false;

/*////// Setting PWM properties, do not change this! /////////*/
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;

//RTC_DATA_ATTR int bootCount = 0;

//TFT y positions for texts and numbers
uint16_t textLvl[10] = {65, 135, 205, 275, 345, 65, 135, 205, 275, 345};  // y coordinates for text
uint16_t drawLvl[10] = {100, 170, 240, 310, 380, 100, 170, 240, 310, 380}; // and numbers

#define pagenumbers 7  // number of pages to display
#define N_km 10        //variable for the calculating kWh/100km over a N_km

boolean ResetOn = true;
int screenNbr = 0;

uint8_t record_code = 0;
float mem_energy = 0;
float mem_PrevSoC = 0;
float mem_SoC = 0;
float mem_Power = 0;
float mem_LastSoC = 0;
uint16_t nbr_powerOn = 0;
bool data_ready = false;
bool code_sent = false;
bool sd_condition1 = false;
bool sd_condition2 = false;
bool SoC_saved = false;
bool code_received = false;
bool shutdown_esp = false;
bool wifiReconn = false;
bool datasent = false;

float BattMinT;
float BattMaxT;
float AuxBattV;
float AuxBattC;
float AuxBattSoC;
float Batt12V;
float BATTv;
float BATTc;
float MAXcellv;
float MINcellv;
int MAXcellvNb;
int MINcellvNb;
float CellVdiff;
float CEC;
float CED;
float CDC;
float CCC;
float BmsSoC;
float Max_Pwr;
float Max_Reg;
float SoC;
float SoCratio;
float Calc_kWh_corr;
float SOH;
float Deter_Min;
int MinDetNb;
int MaxDetNb;
float Heater;
float COOLtemp;
float OUTDOORtemp;
float INDOORtemp;
char SpdSelect;
char* SpdSelected = "X";
uint32_t Odometer;
float Speed;
byte TransSelByte;
byte Park;
byte Reverse;
byte Neutral;
byte Drive;
char selector[1];
byte StatusWord;
byte BMS_ign;
byte StatusWord2;
byte BMS_relay;
float OPtimemins;
float OPtimehours;
float TireFL_P;
float TireFR_P;
float TireRL_P;
float TireRR_P;
float TireFL_T;
float TireFR_T;
float TireRL_T;
float TireRR_T;
float Power;
float CurrInitOdo = 0;
float CurrInitCEC = 0;
float CurrInitCED = 0;
float CurrInitSoC = 0;
float CurrTripOdo;
float CurrNet_kWh;
float CurrUsedSoC;
float CurrTripDisc;
float CurrTripReg;
float CurrInitAccEnergy;
float CurrAccEnergy;
float Prev_kWh = 0;
float Net_kWh = 0;
float UsedSoC = 0;
float Net_Ah = 0;
float DischAh = 0;
float RegenAh = 0;
float TripOdo = 0;
int InitOdo = 0;
float PrevOPtimemins = 0;
float TripOPtime = 0;
float CurrTimeInit = 0;
float CurrOPtime = 0;
float InitSoC = 0;
float InitCEC = 0;
float InitCED = 0;
float InitCCC = 0;
float InitCDC = 0;
float PrevSoC = 0;
float PrevBmsSoC = 0;
float Regen = 0;
float Discharg = 0;
float LastSoC = 0;
float integrate_timer = 0.0;
float start_kwh;
float acc_energy = 0.0;
float acc_regen;
float acc_Ah = 0.0;
float last_energy = 0.0;
float last_time = 0.0;
float last_odo = 0.0;
int energy_array_index = 0;
float energy_array[11]; //needs to hold 11 values in order to calculate last 10 energy values
float span_energy = 0.0;
float speed_interval = 0.0;
float init_speed_timer = 0.0;
float int_speed = 0.0;
float distance = 0.0;
float prev_dist = 0;
float interval_dist = 0;
float Trip_dist = 0;
float prev_odo = 0;
float prev_power = 0.0;
int pwr_changed = 0;
int loop_count = 0;
float full_kwh;
float EstFull_kWh;
float EstFull_Ah;
float kWh_corr;
float left_kwh;
float used_kwh;
float degrad_ratio;
float old_PIDkWh_100km = 14;
float old_kWh_100km = 14;
float old_lost = 1;
float EstLeft_kWh;
float kWh_100km;
float span_kWh_100km;
float PIDkWh_100;
float Est_range;
float Est_range2;
float Est_range3;
unsigned long RangeCalcTimer;
uint32_t  RangeCalcUpdate = 2000; 
float acc_kWh_25;
float acc_kWh_10;
float acc_kWh_0;
float acc_kWh_m10;
float acc_kWh_m20;
float acc_kWh_m20p;
float acc_time_25;
float acc_time_10;
float acc_time_0;
float acc_time_m10;
float acc_time_m20;
float acc_time_m20p;
float acc_dist_25;
float acc_dist_10;
float acc_dist_0;
float acc_dist_m10;
float acc_dist_m20;
float acc_dist_m20p;
bool DriveOn = false;
bool StartWifi = true;
bool initscan = false;
bool InitRst = false;
bool TrigRst = false;
bool kWh_update = false;
bool corr_update = false;
bool ESP_on = false;
bool DrawBackground = true;
char titre[10][13];
char value[10][7];
char prev_value[10][7];
bool negative_flag[10];
float value_float[10];
int nbr_decimal[10];
bool Charge_page = false;
bool Power_page = false;
unsigned long read_timer;
uint32_t read_data_interval = 2000;
unsigned long ELM_Port_timer;

// Variables for touch x,y
static int32_t x, y;
TS_Point p;
static int xMargin = 20, yMargin = 420, margin = 20, btnWidth = 80, btnHeigth = 55;
char* BtnAtext = "CONS";
char* BtnBtext = "BATT";
char* BtnCtext = "POWER";
char Maintitre[][13] = {"Consommation", "Batt. Info", "Energie", "Batt. Info 2"};
uint16_t MainTitleColor = TFT_WHITE;
uint16_t BtnOnColor = TFT_GREEN;
uint16_t BtnOffColor = TFT_LIGHTGREY;

unsigned long initTouchTime = 0;
unsigned long TouchTime = 0;
bool TouchLatch = false;
bool Btn1SetON = true;
bool Btn2SetON = false;
bool Btn3SetON = false;

struct RoundedRect {
  int xStart;
  int yStart;
  int xWidth;
  int yHeight;
  byte cornerRadius;
  uint16_t color;
  char* BtnText;
};

RoundedRect btnAon = {
  xMargin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOnColor,
  BtnAtext
};

RoundedRect btnBon = {
  btnAon.xStart + btnAon.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOnColor,
  BtnBtext
};

RoundedRect btnCon = {
  btnBon.xStart + btnBon.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOnColor,
  BtnCtext
};

RoundedRect btnAoff = {
  xMargin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOffColor,
  BtnAtext
};

RoundedRect btnBoff = {
  btnAoff.xStart + btnAoff.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOffColor,
  BtnBtext
};

RoundedRect btnCoff = {
  btnBoff.xStart + btnBoff.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  BtnOffColor,
  BtnCtext
};

/*///////ESP shutdown variables///////*/
unsigned long ESPinitTimer = 0;
unsigned long ESPTimerInterval = 1200;  // time in seconds to turn off ESP when it power-up during 12V battery charge cycle.
unsigned long shutdown_timer = 0;
unsigned long stopESP_timer = 0;

/*////// Variables for Google Sheet data transfer ////////////*/
bool send_enabled = false;
bool send_data = false;
bool data_sent = false;
uint16_t sendInterval = 10000;  // in millisec
unsigned long GSheetTimer = 0;
bool sendIntervalOn = false;
time_t t = 0;
bool ready = false;
uint16_t nbrDays[13] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366};
const char* EventCode0 = "New Trip";
const char* EventCode1 = "Recharge Reset";
const char* EventCode2 = "Button Reset";
const char* EventCode3 = "Reset on Acc Energy less 0.2";
const char* EventCode4 = "Reset 100 to 99%";
const char* EventCode5 = "Normal Shutdown";
const char* EventCode6 = "Timer Shutdown";
const char* EventCode7 = "Low 12V Shutdown";
const char* Mess_SoC = "SoC:";
const char* Mess_Power = "Power:";
const char* Mess_LastSoC = "LastSoC:";
const char* Mess_PrevSoC = "PrevSoC:";
const char* Mess_Energy = "Energy:";
const char* Mess_SD = "Shutdown Timer:";
const char* Mess_12vSoC = "AuxBattSoC:";

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// Variable to save current epoch time
char EventTime[18];

/*////// Variables for OBD data timing ////////////*/
unsigned long currentMillis;       // timing variable to sample OBD data
unsigned long previousMillis = 0;  // timing variable to sample OBD data
uint8_t pid_counter = 0;

/*///////Define a structure to store the PID query data frames ///////////*/
struct dataFrames_struct {
  char frames[9][20];  // 9 frames each of 20chars
};

typedef struct dataFrames_struct dataFrames;  // create a simple name for this type of data
dataFrames results;                           // this struct will hold the results

void callback(){  //required function for touch wake
  //placeholder callback function
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}


/*////////////////////////////////////////////////////////////////////////*/
/*                         START OF SETUP                                 */
/*////////////////////////////////////////////////////////////////////////*/

void setup() {

  /*////////////////////////////////////////////////////////////////*/
  /*              Open serial monitor communications                */
  /*////////////////////////////////////////////////////////////////*/

  Serial.begin(115200);

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Serial Monitor - STARTED");

  /*//////////////Initialise Touch screen ////////////////*/
  // Pins 18/19 are SDA/SCL for touch sensor on this device
  // 40 is a touch threshold
  if (!ts.begin(18, 19, 40)) {
    Serial.println("Couldn't start touchscreen controller");
    while (true);
  }
  
  /*//////////////Initialise OLED display ////////////////*/
  tft.init();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans18pt7b);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 128);

  Serial.print("Configuring PWM for TFT backlight... ");
  ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
  ledcAttachPin(TFT_BL, pwmLedChannelTFT);
  Serial.println("DONE");

  Serial.print("Setting PWM for TFT backlight to default intensity... ");
  ledcWrite(pwmLedChannelTFT, ledBacklight);
  Serial.println("DONE");

  /*////// initialize EEPROM with predefined size ////////*/
  EEPROM.begin(148);

  /* uncomment if you need to display Safestring results on Serial Monitor */
  //SafeString::setOutput(Serial);
  
  //Configure time
  configTime(0, 0, ntpServer);

  GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

  // Set the callback for Google API access token generation status (for debug only)
  GSheet.setTokenCallback(tokenStatusCallback);

  // Set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
  GSheet.setPrerefreshSeconds(10 * 60);

  // Begin the access token generation for Google API authentication
  GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);

  //Setup interrupt on Touch Pad 2 (GPIO2)
  touchAttachInterrupt(T2, callback, Threshold);

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup(); // initialize ESP wakeup on Touch activation  

  /*////// Get the stored value from last re-initialisation /////*/

  Net_kWh = EEPROM.readFloat(0);
  InitCED = EEPROM.readFloat(4);
  InitCEC = EEPROM.readFloat(8);
  InitSoC = EEPROM.readFloat(12);
  UsedSoC = EEPROM.readFloat(16);
  InitOdo = EEPROM.readFloat(20);
  InitCDC = EEPROM.readFloat(24);
  InitCCC = EEPROM.readFloat(28);
  old_lost = EEPROM.readFloat(32);
  old_PIDkWh_100km = EEPROM.readFloat(36);
  nbr_powerOn = EEPROM.readFloat(40);
  PrevOPtimemins = EEPROM.readFloat(44);
  kWh_corr = EEPROM.readFloat(48);
  acc_energy = EEPROM.readFloat(52);
  LastSoC = EEPROM.readFloat(56);
  old_kWh_100km = EEPROM.readFloat(60);
  acc_Ah = EEPROM.readFloat(64);
  acc_kWh_25 = EEPROM.readFloat(68);
  acc_kWh_10 = EEPROM.readFloat(72);
  acc_kWh_0 = EEPROM.readFloat(76);
  acc_kWh_m10 = EEPROM.readFloat(80);
  acc_kWh_m20 = EEPROM.readFloat(84);
  acc_kWh_m20p = EEPROM.readFloat(88);
  acc_time_25 = EEPROM.readFloat(92);
  acc_time_10 = EEPROM.readFloat(96);
  acc_time_0 = EEPROM.readFloat(100);
  acc_time_m10 = EEPROM.readFloat(104);
  acc_time_m20 = EEPROM.readFloat(108);
  acc_time_m20p = EEPROM.readFloat(112);
  acc_dist_25 = EEPROM.readFloat(116);
  acc_dist_10 = EEPROM.readFloat(120);
  acc_dist_0 = EEPROM.readFloat(124);
  acc_dist_m10 = EEPROM.readFloat(128);
  acc_dist_m20 = EEPROM.readFloat(132);
  acc_dist_m20p = EEPROM.readFloat(136);
  acc_regen = EEPROM.readFloat(140);

  //initial_eeprom(); //if a new eeprom memory is used it needs to be initialize to something first

  /*/////////////////////////////////////////////////////////////////*/
  /*                    CONNECTION TO OBDII                          */
  /*/////////////////////////////////////////////////////////////////*/

  ConnectToOBD2(tft);

  /*/////////////////////////////////////////////////////////////////*/
  /*                     CONNECTION TO WIFI                         */
  /*/////////////////////////////////////////////////////////////////*/

  if (StartWifi) {
    ConnectWifi(tft);
    if (WiFi.status() == WL_CONNECTED) {
      send_enabled = true;
    }
    initscan = true;  // To write header name on Google Sheet on power up
  }  

  /*//////////////Initialise Task on core0 to send data on Google Sheet ////////////////*/

  xTaskCreatePinnedToCore(
    sendGoogleSheet,   /* Function to implement the task */
    "sendGoogleSheet", /* Name of the task */
    10000,              /* Stack size in words */
    NULL,               /* Task input parameter */
    5,                  /* Priority of the task */
    &Task1,             /* Task handle. */
    0);                 /* Core where the task should run */
  delay(500);

  tft.fillScreen(TFT_BLACK);

  integrate_timer = millis() / 1000;
  RangeCalcTimer = millis();
  read_timer = millis();
  GSheetTimer = millis();
  ELM_Port_timer = millis();     

  //nbr_powerOn += 1;
  //EEPROM.writeFloat(40, nbr_powerOn);
  //EEPROM.commit();  
}

/*////////////////////////////////////////////////////////////////////////*/
/*                         END OF SETUP                                   */
/*////////////////////////////////////////////////////////////////////////*/

//----------------------------------------------------------------------------------------
//              OBDII Payloads Processing Functions
//----------------------------------------------------------------------------------------

void clearResultFrames(dataFrames& results) {
  for (int i = 0; i < 9; i++) {
    results.frames[i][0] = '\0';
  }
}

// format is <headerBytes> then <frameNumberByte>:<frameDataBytes> repeated
void processPayload(char* OBDdata, size_t datalen, dataFrames& results) {
  cSFPS(data, OBDdata, datalen);  // wrap in a SafeString
  clearResultFrames(results);
  size_t idx = data.indexOf(':');  // skip over header and find first delimiter
  while (idx < data.length()) {
    int frameIdx = data[idx - 1] - '0';      // the char before :
    if ((frameIdx < 0) || (frameIdx > 8)) {  // error in frame number skip this frame, print a message here

      //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" outside range data: "); data.debug();
      idx = data.indexOf(':', idx + 1);  // step over : and find next :
      continue;
    }
    cSFA(frame, results.frames[frameIdx]);    // wrap a result frame in a SafeString to store this frame's data
    idx++;                                    // step over :
    size_t nextIdx = data.indexOf(':', idx);  // find next :
    if (nextIdx == data.length()) {
      data.substring(frame, idx);  // next : not found so take all the remaining chars as this field
    } else {
      data.substring(frame, idx, nextIdx - 1);  // substring upto one byte before next :
    }
    //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" "); frame.debug();
    idx = nextIdx;  // step onto next frame
  }
}

//------------------------------------------------------------------------------------------
//                  End of Payloads Processing
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//             Bytes extraction from dataFrame
//------------------------------------------------------------------------------------------

int convertToInt(char* dataFrame, size_t startByte, size_t numberBytes) {
  int offset = (startByte - 1) * 2;
  // define a local SafeString on the stack for this method
  cSFP(frame, dataFrame);
  cSF(hexSubString, frame.capacity());                                // allow for taking entire frame as a substring
  frame.substring(hexSubString, offset, offset + (numberBytes * 2));  // endIdx in exclusive in SafeString V2
  hexSubString.debug(F(" hex number "));
  long num = 0;
  if (!hexSubString.hexToLong(num)) {
    hexSubString.debug(F(" invalid hex number "));
  }
  return num;
}

//------------------------------------------------------------------------------------------
//               End of Bytes extraction from dataFrame
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//         Data retreived from OBD2 and extract value of it
//------------------------------------------------------------------------------------------

void read_data() {

  //pid_counter++;
  Serial.println(pid_counter);

  button();

  // read in rawData via ODBII
  
  //  Read PID 220101 each iteration to get faster battery power update
  myELM327.sendCommand("AT SH 7E4");  // Set Header for BMS

  if (myELM327.queryPID("220101")) {  // Service and Message PID = hex 22 0101 => dec 34, 257
   
    char* payload = myELM327.payload;
    size_t payloadLen = myELM327.recBytes;

    processPayload(payload, payloadLen, results);

    int BattMinTraw = convertToInt(results.frames[2], 6, 1);  //specify frame#, starting Byte(o in TorquePro) and # of bytes required
    if (BattMinTraw > 127) {                                  //conversition for negative value[
      BattMinT = -1 * (256 - BattMinTraw);
    } else {
      BattMinT = BattMinTraw;
    }
    int BattMaxTraw = convertToInt(results.frames[2], 5, 1);  //specify frame#, starting Byte(o in TorquePro) and # of bytes required
    if (BattMaxTraw > 127) {                                  //conversition for negative value[
      BattMaxT = -1 * (256 - BattMaxTraw);
    } else {
      BattMaxT = BattMaxTraw;
    }
    AuxBattV = convertToInt(results.frames[4], 6, 1) * 0.1;
    BATTv = convertToInt(results.frames[2], 3, 2) * 0.1;
    int CurrentByte1 = convertToInt(results.frames[2], 1, 1);
    int CurrentByte2 = convertToInt(results.frames[2], 2, 1);
    if (CurrentByte1 > 127) {  // the most significant bit is the sign bit so need to calculate commplement value[ if true
      BATTc = -1 * (((255 - CurrentByte1) * 256) + (256 - CurrentByte2)) * 0.1;
    } else {
      BATTc = ((CurrentByte1 * 256) + CurrentByte2) * 0.1;
    }
    CEC = convertToInt(results.frames[6], 1, 4) * 0.1;
    CED = ((convertToInt(results.frames[6], 5, 3) << 8) + convertToInt(results.frames[7], 1, 1)) * 0.1;
    CCC = ((convertToInt(results.frames[4], 7, 1) << 24) + convertToInt(results.frames[5], 1, 3)) * 0.1;
    CDC = convertToInt(results.frames[5], 4, 4) * 0.1;
    BmsSoC = convertToInt(results.frames[1], 2, 1) * 0.5;
    StatusWord = convertToInt(results.frames[7], 6, 1);  // Extract byte that contain BMS status bits
    BMS_ign = bitRead(StatusWord, 2);
    StatusWord2 = convertToInt(results.frames[1], 7, 1);  // Extract byte that contain BMS status bits
    BMS_relay = bitRead(StatusWord2, 0);
    MAXcellv = convertToInt(results.frames[3], 7, 1) * 0.02;
    MAXcellvNb = convertToInt(results.frames[4], 1, 1);
    MINcellv = convertToInt(results.frames[4], 2, 1) * 0.02;
    MINcellvNb = convertToInt(results.frames[4], 3, 1);
    OPtimemins = convertToInt(results.frames[7], 2, 4) * 0.01666666667;
    OPtimehours = OPtimemins * 0.01666666667;
  }
  if (BMS_ign) {
    ESP_on = true;
  }
  UpdateNetEnergy();
  pwr_changed += 1;
  
  if (BMS_relay){
  // Read remaining PIDs only if BMS relay is ON
    switch (pid_counter) {
      case 1:
  
        button();
        myELM327.sendCommand("AT SH 7E4");  // Set Header for BMS
        Serial.println("read PID105");
  
        if (myELM327.queryPID("220105")) {  // Service and Message PID = hex 22 0105 => dec 34, 261
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          Max_Pwr = convertToInt(results.frames[3], 2, 2) * 0.01;
          Max_Reg = (((convertToInt(results.frames[2], 7, 1)) << 8) + convertToInt(results.frames[3], 1, 1)) * 0.01;
          SoC = convertToInt(results.frames[5], 1, 1) * 0.5;
          SOH = convertToInt(results.frames[4], 2, 2) * 0.1;
          MaxDetNb = convertToInt(results.frames[4], 4, 1);
          MinDetNb = convertToInt(results.frames[4], 7, 1);
          Deter_Min = convertToInt(results.frames[4], 5, 2) * 0.1;
          int HeaterRaw = convertToInt(results.frames[3], 7, 1);
          if (HeaterRaw > 127) {  //conversition for negative value[
            Heater = -1 * (256 - HeaterRaw);
          } else {
            Heater = HeaterRaw;
          }
        }
        break;
  
      case 2:
  
        button();
        myELM327.sendCommand("AT SH 7E4");  // Set Header for BMS
  
        if (myELM327.queryPID("220106")) {  // Service and Message PID = hex 22 0106 => dec 34, 262
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          int COOLtempRaw = convertToInt(results.frames[1], 2, 1) * 0.01;  // Cooling water temperature
          if (COOLtempRaw > 127) {                                         //conversition for negative value[
            COOLtemp = -1 * (256 - COOLtempRaw);
          } else {
            COOLtemp = COOLtempRaw;
          }
        }
        break;
  
      case 3:
        
        button();
        myELM327.sendCommand("AT SH 7E2");  // Set Header for Vehicle Control Unit
  
        if (myELM327.queryPID("2101")) {  // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          TransSelByte = convertToInt(results.frames[1], 2, 1);  // Extract byte that contain transmission selection bits
          //Serial.print("SelByte: "); Serial.println(TransSelByte, 1);
          Park = bitRead(TransSelByte, 0);
          Reverse = bitRead(TransSelByte, 1);
          Neutral = bitRead(TransSelByte, 2);
          Drive = bitRead(TransSelByte, 3);
  
          if (Park) selector[0] = 'P';
          if (Reverse) selector[0] = 'R';
          if (Neutral) selector[0] = 'N';
          if (Drive) selector[0] = 'D';
          SpdSelect = selector[0];
          SpdSelected = &SpdSelect;
        }
        break;
  
      case 4:
        
        button();
        myELM327.sendCommand("AT SH 7E2");  // Set Header for Vehicle Control Unit
        if (myELM327.queryPID("2102")) {    // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          //AuxBattV = convertToInt(results.frames[3], 2, 2)* 0.001; //doesn't work...
          int AuxCurrByte1 = convertToInt(results.frames[3], 4, 1);
          int AuxCurrByte2 = convertToInt(results.frames[3], 5, 1);
          if (AuxCurrByte1 > 127) {  // the most significant bit is the sign bit so need to calculate commplement value[ if true
            AuxBattC = -1 * (((255 - AuxCurrByte1) * 256) + (256 - AuxCurrByte2)) * 0.01;
          } else {
            AuxBattC = ((AuxCurrByte1 * 256) + AuxCurrByte2) * 0.01;
          }
          AuxBattSoC = convertToInt(results.frames[3], 6, 1);
        }
        break;
  
      case 5:
        
        button();
        myELM327.sendCommand("AT SH 7C6");  // Set Header for CLU Cluster Module
        if (myELM327.queryPID("22B002")) {  // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          Odometer = convertToInt(results.frames[1], 4, 3);
        }
        break;
  
      case 6:
  
        button();
        myELM327.sendCommand("AT SH 7B3");  //Set Header Aircon
        if (myELM327.queryPID("220100")) {  // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          INDOORtemp = (((convertToInt(results.frames[1], 3, 1)) * 0.5) - 40);
          OUTDOORtemp = (((convertToInt(results.frames[1], 4, 1)) * 0.5) - 40);
        }
        break;
  
      case 7:
  
        button();
        myELM327.sendCommand("AT SH 7D4");  //Set Speed Header
        if (myELM327.queryPID("220101")) {  // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          Speed = (((convertToInt(results.frames[1], 7, 1)) << 8) + convertToInt(results.frames[2], 1, 1)) * 0.1;
        }
        Integrat_speed();
        break;
  
      case 8:
  
        button();
        myELM327.sendCommand("AT SH 7A0");  //Set BCM Header
        if (myELM327.queryPID("22C00B")) {  // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
  
          processPayload(payload, payloadLen, results);
          TireFL_P = convertToInt(results.frames[1], 2, 1) * 0.2;
          TireFL_T = convertToInt(results.frames[1], 3, 1) - 50;
          TireFR_P = convertToInt(results.frames[1], 6, 1) * 0.2;
          TireFR_T = convertToInt(results.frames[1], 7, 1) - 50;
          TireRL_P = convertToInt(results.frames[2], 7, 1) * 0.2;
          TireRL_T = convertToInt(results.frames[3], 1, 1) - 50;
          TireRR_P = convertToInt(results.frames[2], 3, 1) * 0.2;
          TireRR_T = convertToInt(results.frames[2], 4, 1) - 50;
        }
        pid_counter = 0;
        data_ready = true;  // after all PIDs have been read, turn on flag for valid value from OBD2
        break;
    }


  /////// Miscellaneous calculations /////////

  Power = (BATTv * BATTc) * 0.001;
  Integrat_power();
  Integrat_current();
  integrate_timer = millis();

    if (!ResetOn) {  // On power On, wait for current trip value to be re-initialized before executing the next lines of code
      TripOdo = Odometer - InitOdo;
  
      CurrTripOdo = Odometer - CurrInitOdo;
  
      CurrOPtime = OPtimemins - CurrTimeInit;
  
      TripOPtime = CurrOPtime + PrevOPtimemins;
  
      UsedSoC = InitSoC - SoC;
  
      CurrUsedSoC = CurrInitSoC - SoC;
  
      if (UsedSoC < 0.5){
        EstFull_Ah = 186;
      }
      else{
        EstFull_Ah = 100 * Net_Ah / UsedSoC;
      }
  
      CellVdiff = MAXcellv - MINcellv;
      
      if (PrevBmsSoC > BmsSoC) {  // perform a BmsSoC vs SoC ratio calculation when BmsSoC changes
        PrevBmsSoC = BmsSoC;
        SocRatioCalc();
      }
      else if (PrevBmsSoC < 0){
        PrevBmsSoC = BmsSoC;
      }
  
      if (PrevSoC != SoC) {  // perform "used_kWh" and "left_kWh" when SoC changes
        if (InitRst) {       // On Button Trip reset, initial kWh calculation
          Serial.print("1st Reset");
          initscan = true;
          record_code = 2;
          reset_trip();        
          kWh_corr = 0;
          PrevSoC = SoC;
          Prev_kWh = Net_kWh;
          used_kwh = calc_kwh(SoC, InitSoC);
          left_kwh = calc_kwh(0, SoC);
          InitRst = false;
        }
        if (!InitRst) {  // kWh calculation when the Initial reset is not active
          // After a Trip Reset, perform a new reset if SoC changed without a Net_kWh increase (in case SoC was just about to change when the reset was performed)
          if (((acc_energy < 0.25) && (PrevSoC > SoC)) || ((SoC > 98.5) && ((PrevSoC - SoC) > 0.5))) {
            //if(((Net_kWh < 0.3) && (PrevSoC > SoC)) || ((SoC > 98.5) && ((PrevSoC - SoC) > 0.5)) || (TrigRst && (PrevSoC > SoC))){          
            if ((acc_energy < 0.3) && (PrevSoC > SoC)) {
              initscan = true;
              mem_energy = acc_energy;
              mem_PrevSoC = PrevSoC;
              mem_SoC = SoC;
              record_code = 3;
              Serial.print("2nd Reset");
              reset_trip();           
              kWh_corr = 0;
              used_kwh = calc_kwh(SoC, InitSoC);
              left_kwh = calc_kwh(0, SoC);
              PrevSoC = SoC;
              Prev_kWh = Net_kWh;
              kWh_update = true;
            } else {
              record_code = 4;
            }
  
          } else if (((PrevSoC > SoC) && ((PrevSoC - SoC) < 1)) || ((PrevSoC < SoC) && (SpdSelect == 'P'))) {  // Normal kWh calculation when SoC decreases and exception if a 0 gitch in SoC data
            kWh_corr = 0;
            used_kwh = calc_kwh(SoC, InitSoC);
            left_kwh = calc_kwh(0, SoC);
            PrevSoC = SoC;
            Prev_kWh = Net_kWh;
            kWh_update = true;
  
            if ((used_kwh >= 2) && (SpdSelect == 'D')) {  // Wait till 2 kWh has been used to start calculating ratio to have a better accuracy
              degrad_ratio = Net_kWh / used_kwh;
              old_lost = degrad_ratio;
            } else {
              degrad_ratio = old_lost;
              if ((degrad_ratio > 1.2) || (degrad_ratio < 0.8)) {  // if a bad value[ got saved previously, initialize ratio to 1
                degrad_ratio = 1;
              }
            }
          }
        }
  
      } else if ((Prev_kWh < Net_kWh) && !kWh_update) {  // since the SoC has only 0.5 kWh resolution, when the Net_kWh increases, a 0.1 kWh is added to the kWh calculation to interpolate until next SoC change.
        kWh_corr += 0.1;
        used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
        left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
        Prev_kWh = Net_kWh;
        corr_update = true;
      } else if ((Prev_kWh > Net_kWh) && !kWh_update) {  // since the SoC has only 0.5 kWh resolution, when the Net_kWh decreases, a 0.1 kWh is substracted to the kWh calculation to interpolate until next SoC change.
        kWh_corr -= 0.1;
        used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
        left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
        Prev_kWh = Net_kWh;
        corr_update = true;
      }
  
      if (sendIntervalOn) {  // add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update" when wifi is not connected
        if (kWh_update) {
          Prev_kWh = Net_kWh;
          kWh_update = false;  // reset kWh_update so correction logic starts again
        }
        if (corr_update) {
          corr_update = false;  // reset corr_update since it's not being recorded
        }
        sendIntervalOn = false;
      }
  
      if ((LastSoC + 1) < SoC && (Power > 0) && (LastSoC != 0)) {  // reset trip after a battery recharge      
        mem_Power = Power;
        mem_LastSoC = LastSoC;
        mem_SoC = SoC;
        initscan = true;
        record_code = 1;
        reset_trip();      
      }
  
      EstFull_kWh = full_kwh * degrad_ratio;
      EstLeft_kWh = left_kwh * degrad_ratio;
  
      if ((millis() - RangeCalcTimer) > RangeCalcUpdate){
        RangeCalc();
        RangeCalcTimer = millis();
      }
      
      if (BMS_ign) {
        EnergyTOC();
      }
  
      if (Max_Pwr < 100 && (Max_Pwr < (Power + 20)) && !Power_page) {  //select the Max Power page if Power+20kW exceed Max_Pwr when Max_Pwr is lower then 100kW.
        DrawBackground = true;
        screenNbr = 2;
        Power_page = true;
        if (Btn1SetON){
          Btn1SetON = false;        
        }
        if (Btn2SetON){
          Btn2SetON = false;       
        }
      }
      if (Power < 0 && (SpdSelect == 'P') && !Charge_page) {
        DrawBackground = true;
        screenNbr = 2;
        Charge_page = true;
        if (Btn1SetON){
          Btn1SetON = false;        
        }
        if (Btn2SetON){
          Btn2SetON = false;       
        }
      }
    }
  
    save_lost(SpdSelect);
  }
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Calculation Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Discharge Energy Since last reset //////////*/

void UpdateNetEnergy() {

  if (InitCED == 0) {  //if discharge value[ have been reinitiate to 0 then
    InitCED = CED;     //initiate to current CED for initial CED value[ and
    InitSoC = SoC;     //initiate to current CED for initial SoC value[ and
    CurrInitCED = CED;
  }
  if (InitCDC == 0) {
    InitCDC = CDC;
  }
  if (InitCEC == 0) {  //if charge value[ have been reinitiate to 0 then
    InitCEC = CEC;     //initiate to current CEC for initial CEC value[
    CurrInitCEC = CEC;
  }
  if (InitCCC == 0) {
    InitCCC = CCC;
  }

  Discharg = CED - InitCED;
  Regen = CEC - InitCEC;
  Net_kWh = Discharg - Regen;

  DischAh = CDC - InitCDC;
  RegenAh = CCC - InitCCC;
  Net_Ah = DischAh - RegenAh;

  CurrAccEnergy = acc_energy - CurrInitAccEnergy;
  CurrTripDisc = CED - CurrInitCED;
  CurrTripReg = CEC - CurrInitCEC;
  CurrNet_kWh = CurrTripDisc - CurrTripReg;
}

//--------------------------------------------------------------------------------------------
//                   Net Energy based on Power integration Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Energy by power integration since last reset //////////*/

void Integrat_power() {
  float pwr_interval;
  float int_pwr;
  pwr_interval = (millis() - integrate_timer) / 1000;
  int_pwr = Power * pwr_interval / 3600;
  acc_energy += int_pwr;
  if (int_pwr < 0) {
    acc_regen += -(int_pwr);
  }
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Charge based on Power integration Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Energy Charge by current integration since last reset //////////*/

void Integrat_current() {
  float curr_interval;
  float int_curr;
  curr_interval = (millis() - integrate_timer) / 1000;
  int_curr = BATTc * curr_interval / 3600;
  acc_Ah += int_curr;
}

//--------------------------------------------------------------------------------------------
//                   Net Energy for last N km Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Energy over last N km //////////*/

void N_km_energy(float latest_energy) {
  energy_array[energy_array_index] = latest_energy;
  energy_array_index++;
  if (energy_array_index > N_km) {
    energy_array_index = 0;
  }
  span_energy = latest_energy - energy_array[energy_array_index];  
}

//--------------------------------------------------------------------------------------------
//                   Distance based on Speed integration Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate distance by speed integration over time //////////*/

void Integrat_speed() {
  speed_interval = (millis() - init_speed_timer) / 1000;
  int_speed = Speed * speed_interval / 3600;
  distance += (int_speed * 1.022);  // need to apply a 1.022 to get correct distance
  init_speed_timer = millis();
}

//--------------------------------------------------------------------------------------------
//                   Kilometer Range Calculation Function
//--------------------------------------------------------------------------------------------

void RangeCalc() {

  if ((prev_odo != CurrTripOdo) && (distance < 0.9)) {
    if (TripOdo < 2) {
      InitOdo = Odometer - distance;  // correct initial odometer value[ using speed integration if odometer changes within 0.9km
      TripOdo = Odometer - InitOdo;
    }
    CurrInitOdo = Odometer - distance;
    CurrTripOdo = Odometer - CurrInitOdo;
    prev_dist = distance;
    prev_odo = CurrTripOdo;
    N_km_energy(acc_energy);
  } else if (prev_odo != CurrTripOdo) {
    prev_dist = distance;
    prev_odo = CurrTripOdo;
    N_km_energy(acc_energy);
  }
  interval_dist = distance - prev_dist;
  Trip_dist = CurrTripOdo + interval_dist;

  if (Trip_dist >= 0.25 && !ResetOn) {
    kWh_100km = CurrAccEnergy * 100 / Trip_dist;
    PIDkWh_100 = CurrNet_kWh * 100 / Trip_dist;
  }  
  else {
    kWh_100km = old_kWh_100km;
    PIDkWh_100 = old_PIDkWh_100km;
  }

  if (Trip_dist >= (N_km + 1)) {  // wait 11km before calculating consommation for last 10km
    span_kWh_100km = span_energy * 100 / N_km;
  } else {
    span_kWh_100km = kWh_100km;
  }

  if (kWh_100km > 1) {
    Est_range = (EstLeft_kWh / kWh_100km) * 100;
    Est_range2 = (EstLeft_kWh / span_kWh_100km) * 100;
    Est_range3 = (EstLeft_kWh / PIDkWh_100) * 100;
  } else {
    Est_range = 999;
    Est_range2 = 999;
  }
}

//--------------------------------------------------------------------------------------------
//                   Ratio of Real Battery Capacity Used Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate the real  //////////*/

void SocRatioCalc() {
  SoCratio = (BmsSoC / SoC) * 100;
  
  Calc_kWh_corr = 1 - (0.97 - (SoCratio / 100));
}

//--------------------------------------------------------------------------------------------
//                   Energy TOC Function
//--------------------------------------------------------------------------------------------

/*//////Function to record time on condition  //////////*/

void EnergyTOC() {
  if (OUTDOORtemp >= 25) {
    acc_kWh_25 = acc_kWh_25 + (acc_energy - last_energy);
    acc_time_25 = acc_time_25 + (CurrOPtime - last_time);
    acc_dist_25 = acc_dist_25 + (distance - last_odo);
  } else if ((OUTDOORtemp < 25) && (OUTDOORtemp >= 10)) {
    acc_kWh_10 = acc_kWh_10 + (acc_energy - last_energy);
    acc_time_10 = acc_time_10 + (CurrOPtime - last_time);
    acc_dist_10 = acc_dist_10 + (distance - last_odo);
  } else if ((OUTDOORtemp < 10) && (OUTDOORtemp >= 0)) {
    acc_kWh_0 = acc_kWh_0 + (acc_energy - last_energy);
    acc_time_0 = acc_time_0 + (CurrOPtime - last_time);
    acc_dist_0 = acc_dist_0 + (distance - last_odo);
  } else if ((OUTDOORtemp < 0) && (OUTDOORtemp >= -10)) {
    acc_kWh_m10 = acc_kWh_m10 + (acc_energy - last_energy);
    acc_time_m10 = acc_time_m10 + (CurrOPtime - last_time);
    acc_dist_m10 = acc_dist_m10 + (distance - last_odo);
  } else if ((OUTDOORtemp < -10) && (OUTDOORtemp >= -20)) {
    acc_kWh_m20 = acc_kWh_m20 + (acc_energy - last_energy);
    acc_time_m20 = acc_time_m20 + (CurrOPtime - last_time);
    acc_dist_m20 = acc_dist_m20 + (distance - last_odo);
  } else if (OUTDOORtemp < -20) {
    acc_kWh_m20p = acc_kWh_m20p + (acc_energy - last_energy);
    acc_time_m20p = acc_time_m20p + (CurrOPtime - last_time);
    acc_dist_m20p = acc_dist_m20p + (distance - last_odo);
  }
  last_energy = acc_energy;
  last_time = CurrOPtime;
  last_odo = distance;
}

//--------------------------------------------------------------------------------------------
//                   Function to calculate energy between two SoC value
//--------------------------------------------------------------------------------------------

double Interpolate(double xvalue[], double yvalue[], int numvalue, double pointX, bool trim = true) {
  if (trim) {
    if (pointX <= xvalue[0]) return yvalue[0];
    if (pointX >= xvalue[numvalue - 1]) return yvalue[numvalue - 1];
  }

  auto i = 0;
  if (pointX <= xvalue[0]) i = 0;
  else if (pointX >= xvalue[numvalue - 1]) i = numvalue - 1;
  else
    while (pointX >= xvalue[i + 1]) i++;
  if (pointX == xvalue[i + 1]) return yvalue[i + 1];

  auto t = (pointX - xvalue[i]) / (xvalue[i + 1] - xvalue[i]);
  t = t * t * (3 - 2 * t);
  return yvalue[i] * (1 - t) + yvalue[i + 1] * t;
}

float calc_kwh(float min_SoC, float max_SoC) {
  /* variable for kWh/%SoC calculation: xvalue = %SoC and yvalue = kWh */
  const int numvalue = 21;
  double xvalue[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
  //double yvalue[] = { 0.5487, 0.5921, 0.5979, 0.6053, 0.6139, 0.6199, 0.6238, 0.6268, 0.6295, 0.6324, 0.6362, 0.6418, 0.6524, 0.6601, 0.6684, 0.6771, 0.6859, 0.6951, 0.7046, 0.7147, 0.7249};
  double yvalue[] = { 0.5432, 0.5867, 0.5931, 0.6011, 0.6102, 0.6168, 0.6213, 0.6249, 0.6282, 0.6317, 0.6362, 0.6424, 0.6537, 0.6621, 0.6711, 0.6805, 0.6900, 0.7000, 0.7102, 0.7211, 0.7321 };
  float integral;
  float interval;
  float return_kwh;
  static int N = 100;
  interval = (max_SoC - min_SoC) / N;
  integral = 0, 0;
  float x = 0;
  for (int i = 0; i < N; ++i) {
    x = min_SoC + interval * i;
    integral += Interpolate(xvalue, yvalue, numvalue, x);  //64kWh battery energy equation
  }
  //return_kwh = integral * interval;
  return_kwh = (integral * interval) * Calc_kWh_corr;
  return return_kwh;
}

void tokenStatusCallback(TokenInfo info){
    if (info.status == token_status_error){
        GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
        GSheet.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
    }
    else{
        GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
    }
}

//----------------------------------------------------------------------------------------
//        Task on core 0 to Send data to Google Sheet via IFTTT web service Function
//----------------------------------------------------------------------------------------

void sendGoogleSheet(void * pvParameters){
  for(;;){        
    if (send_enabled && send_data) {
      code_sent = false;
      
      FirebaseJson response;

      Serial.println("\nAppend spreadsheet values...");
      Serial.println("----------------------------");

      FirebaseJson valueRange;

      sprintf(EventTime, "%02d-%02d-%02d %02d:%02d:%02d", day(t), month(t), year(t), hour(t), minute(t), second(t));
          
      if(initscan || record_code != 0 || shutdown_esp){

        switch (record_code)
        {
        case 0:   // No reset only header required, ESP32 power reboot
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode0);          
          initscan = false;
          break;

        case 1:   // Write status for Reset after a battery was recharged
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode1);
          valueRange.set("values/[1]/[0]", Mess_SoC);
          valueRange.set("values/[2]/[0]", mem_SoC);
          valueRange.set("values/[3]/[0]", Mess_LastSoC); 
          valueRange.set("values/[4]/[0]", mem_LastSoC);             
          record_code = 0;
          initscan = true;
          break;

        case 2:   // Write status for Reset performed with reset button (right button)
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode2);                        
          record_code = 0;
          initscan = true;
          break;
          
        case 3:   // Write status for Reset when Acc_energy is less then 0.3kWh when SoC changes
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode3);
          valueRange.set("values/[1]/[0]", Mess_SoC);          
          valueRange.set("values/[2]/[0]", mem_SoC);
          valueRange.set("values/[3]/[0]", Mess_PrevSoC);          
          valueRange.set("values/[4]/[0]", mem_PrevSoC);
          valueRange.set("values/[5]/[0]", Mess_Energy);
          valueRange.set("values/[6]/[0]", mem_energy);              
          record_code = 0;
          initscan = true;
          break;
              
        case 4:   // Write status for Reset if SoC changes from 100 to 99% not going through 99.5%
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode4);
          valueRange.set("values/[1]/[0]", Mess_SoC);
          valueRange.set("values/[2]/[0]", mem_SoC);
          valueRange.set("values/[3]/[0]", Mess_PrevSoC); 
          valueRange.set("values/[4]/[0]", mem_PrevSoC); 
          record_code = 0;
          initscan = true;
          break;

        case 5:   // Write that esp is going normal shutdown
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode5);
          valueRange.set("values/[1]/[0]", Mess_SoC);
          valueRange.set("values/[2]/[0]", mem_SoC);
          valueRange.set("values/[3]/[0]", Mess_Power);
          valueRange.set("values/[4]/[0]", Power);                      
          code_received = true;
          record_code = 0;            
          Serial.println("Code Received");
          break;

        case 6:   // Write that esp is going timed shutdown
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode6);
          valueRange.set("values/[1]/[0]", Mess_Power);          
          valueRange.set("values/[2]/[0]", Power);
          valueRange.set("values/[3]/[0]", Mess_SD);          
          valueRange.set("values/[4]/[0]", shutdown_timer);                          
          code_received = true;
          record_code = 0;            
          break;

        case 7:   // Write that esp is going low 12V shutdown
          valueRange.add("majorDimension","COLUMNS");
          valueRange.set("values/[0]/[0]", EventCode7);
          valueRange.set("values/[0]/[0]", Mess_SD);
          valueRange.set("values/[3]/[0]", shutdown_timer);
          valueRange.set("values/[0]/[0]", Mess_12vSoC);
          valueRange.set("values/[7]/[0]", AuxBattSoC);                          
          code_received = true;
          record_code = 0;            
          break;
        }
      }  
      else{
        valueRange.add("majorDimension","COLUMNS");
        valueRange.set("values/[0]/[0]", EventTime);
        valueRange.set("values/[1]/[0]", SoC);
        valueRange.set("values/[2]/[0]", Power);
        valueRange.set("values/[3]/[0]", BattMinT);
        valueRange.set("values/[4]/[0]", Heater);
        valueRange.set("values/[5]/[0]", Net_Ah);
        valueRange.set("values/[6]/[0]", Net_kWh);
        valueRange.set("values/[7]/[0]", AuxBattSoC);
        valueRange.set("values/[8]/[0]", AuxBattV);
        valueRange.set("values/[9]/[0]", Max_Pwr);
        valueRange.set("values/[10]/[0]", Max_Reg);
        valueRange.set("values/[11]/[0]", BmsSoC);
        valueRange.set("values/[12]/[0]", MAXcellv);
        valueRange.set("values/[13]/[0]", MINcellv);
        valueRange.set("values/[14]/[0]", MAXcellvNb);
        valueRange.set("values/[15]/[0]", MINcellvNb);
        valueRange.set("values/[16]/[0]", BATTv);
        valueRange.set("values/[17]/[0]", BATTc);
        valueRange.set("values/[18]/[0]", Speed);
        valueRange.set("values/[19]/[0]", Odometer);
        valueRange.set("values/[20]/[0]", CEC);
        valueRange.set("values/[21]/[0]", CED);
        valueRange.set("values/[22]/[0]", CDC);
        valueRange.set("values/[23]/[0]", CCC);
        valueRange.set("values/[24]/[0]", SOH);
        valueRange.set("values/[25]/[0]", BMS_ign);
        valueRange.set("values/[26]/[0]", OPtimemins);
        valueRange.set("values/[27]/[0]", OUTDOORtemp);
        valueRange.set("values/[28]/[0]", INDOORtemp);
        valueRange.set("values/[29]/[0]", SpdSelected);
        valueRange.set("values/[30]/[0]", LastSoC);
        valueRange.set("values/[31]/[0]", used_kwh);
        valueRange.set("values/[32]/[0]", left_kwh);
        valueRange.set("values/[33]/[0]", TripOPtime);
        valueRange.set("values/[34]/[0]", CurrOPtime);
        valueRange.set("values/[35]/[0]", PIDkWh_100);
        valueRange.set("values/[36]/[0]", kWh_100km);
        valueRange.set("values/[37]/[0]", degrad_ratio);
        valueRange.set("values/[38]/[0]", EstLeft_kWh);
        valueRange.set("values/[39]/[0]", span_kWh_100km);
        valueRange.set("values/[40]/[0]", SoCratio);
        valueRange.set("values/[41]/[0]", nbr_powerOn);
        valueRange.set("values/[42]/[0]", TireFL_P);
        valueRange.set("values/[43]/[0]", TireFR_P);
        valueRange.set("values/[44]/[0]", TireRL_P);
        valueRange.set("values/[45]/[0]", TireRR_P);
        valueRange.set("values/[46]/[0]", TireFL_T);
        valueRange.set("values/[47]/[0]", TireFR_T);
        valueRange.set("values/[48]/[0]", TireRL_T);
        valueRange.set("values/[49]/[0]", TireRR_T);
        valueRange.set("values/[50]/[0]", acc_energy);
        valueRange.set("values/[51]/[0]", Trip_dist);
        valueRange.set("values/[52]/[0]", distance);
        valueRange.set("values/[53]/[0]", BattMaxT);
        valueRange.set("values/[54]/[0]", acc_Ah);
        valueRange.set("values/[55]/[0]", acc_kWh_25);
        valueRange.set("values/[56]/[0]", acc_kWh_10);
        valueRange.set("values/[57]/[0]", acc_kWh_0);
        valueRange.set("values/[58]/[0]", acc_kWh_m10);
        valueRange.set("values/[59]/[0]", acc_kWh_m20);
        valueRange.set("values/[60]/[0]", acc_kWh_m20p);
        valueRange.set("values/[61]/[0]", acc_time_25);
        valueRange.set("values/[62]/[0]", acc_time_10);
        valueRange.set("values/[63]/[0]", acc_time_0);
        valueRange.set("values/[64]/[0]", acc_time_m10);
        valueRange.set("values/[65]/[0]", acc_time_m20);
        valueRange.set("values/[66]/[0]", acc_time_m20p);
        valueRange.set("values/[67]/[0]", acc_dist_25);
        valueRange.set("values/[68]/[0]", acc_dist_10);
        valueRange.set("values/[69]/[0]", acc_dist_0);
        valueRange.set("values/[70]/[0]", acc_dist_m10);
        valueRange.set("values/[71]/[0]", acc_dist_m20);
        valueRange.set("values/[72]/[0]", acc_time_m20p);
        valueRange.set("values/[73]/[0]", acc_regen);
        valueRange.set("values/[74]/[0]", MaxDetNb);
        valueRange.set("values/[75]/[0]", MinDetNb);
        valueRange.set("values/[76]/[0]", Deter_Min);
      }                                   
            
      // Append values to the spreadsheet
      bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet1!A1" /* range to append */, &valueRange /* data range to append */);
      if (success){
          response.toString(Serial, true);
          valueRange.clear();
      }
      else{
          Serial.println(GSheet.errorReason());
      }
      Serial.println();
      Serial.println(ESP.getFreeHeap());

      datasent = true;    
      send_data = false;
      pwr_changed = 0;
      loop_count = 0;
      
      if(kWh_update){ //add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update"
        Prev_kWh = Net_kWh;        
        kWh_update = false;  // reset kWh_update after it has been recorded and so the correction logic start again       
      }            
      if(corr_update){  
        corr_update = false;  // reset corr_update after it has been recorded
      }
      if (code_received){
        Serial.println("Sending code sent");
        code_sent = true;
      }      
    
    }    
    vTaskDelay(10); // some delay is required to reset watchdog timer
  }
}

/*////////////// Full Trip Reset ///////////////// */

void reset_trip() {  //Overall trip reset. Automatic if the car has been recharged to the same level as previous charge or if left button is pressed for more then 3 secondes

  Serial.println("saving");
  InitOdo = Odometer;
  InitCED = CED;  
  InitSoC = SoC;  
  InitCEC = CEC;  
  InitCDC = CDC;
  InitCCC = CCC;
  Net_kWh = 0;
  acc_energy = 0;
  acc_Ah = 0;
  UsedSoC = 0;
  kWh_corr = 0;
  Discharg = 0;
  Regen = 0;
  Net_Ah = 0;
  DischAh = 0;
  RegenAh = 0;
  PrevOPtimemins = 0;
  LastSoC = SoC;
  PrevBmsSoC = BmsSoC;
  EEPROM.writeFloat(0, Net_kWh);  //save initial CED to Flash memory
  EEPROM.writeFloat(52, acc_energy);
  EEPROM.writeFloat(4, InitCED);   //save initial CED to Flash memory
  EEPROM.writeFloat(8, InitCEC);   //save initial CEC to Flash memory
  EEPROM.writeFloat(12, InitSoC);  //save initial SoC to Flash memory
  EEPROM.writeFloat(16, UsedSoC);  //save initial SoC to Flash memory
  EEPROM.writeFloat(20, InitOdo);  //save initial Odometer to Flash memory
  EEPROM.writeFloat(24, InitCDC);  //save initial CDC to Flash memory
  EEPROM.writeFloat(28, InitCCC);  //save initial CCC to Flash memory
  EEPROM.commit();
  Serial.println("value saved to EEPROM");
  CurrInitCED = CED;
  CurrInitCEC = CEC;
  CurrInitOdo = Odometer;
  CurrInitSoC = SoC;
  CurrTripReg = 0;
  CurrTripDisc = 0;
  CurrTimeInit = OPtimemins;
  integrate_timer = millis();
  distance = 0;
  CurrInitAccEnergy = 0;
  SocRatioCalc();
  last_energy = acc_energy;
  start_kwh = calc_kwh(0, InitSoC);
}

/*////////////// Current Trip Reset ///////////////// */

void ResetCurrTrip() {  // when the car is turned On, current trip value are resetted.

  if (ResetOn && (SoC > 1) && (Odometer > 1) && (CED > 1) && data_ready) {  // ResetOn condition might be enough, might need to update code...
    CurrInitAccEnergy = acc_energy;
    CurrInitCED = CED;
    CurrInitCEC = CEC;
    CurrInitOdo = Odometer;
    CurrInitSoC = SoC;
    CurrTripReg = 0;
    CurrTripDisc = 0;
    CurrTimeInit = OPtimemins;
    Serial.println("Trip Reset");
    Prev_kWh = Net_kWh;
    last_energy = acc_energy;
    SocRatioCalc();
    used_kwh = calc_kwh(SoC, InitSoC) + kWh_corr;
    left_kwh = calc_kwh(0, SoC) - kWh_corr;

    PrevSoC = SoC;
    PrevBmsSoC = BmsSoC;
    full_kwh = calc_kwh(0, 100);
    start_kwh = calc_kwh(0, InitSoC);
    ResetOn = false;
    DrawBackground = true;
    for (uint8_t i = 0; i < N_km; i++) {
      energy_array[i] = acc_energy;
    }
    degrad_ratio = old_lost;
    if ((degrad_ratio > 1.2) || (degrad_ratio < 0.7)) {  // if a bad values got saved previously, initial ratio to 1
      degrad_ratio = 1;
    }
  }
}

void initial_eeprom() {

  for (int i = 0; i < 148; i += 4) {
    if (isnan(EEPROM.readFloat(i))) {
      EEPROM.writeFloat(i, 0);
    }
  }
  EEPROM.commit();
}

/*//////Function to save some variable before turn off the car //////////*/

void save_lost(char selector) {
  if (selector == 'D' && !DriveOn) {
    DriveOn = true;
  }
  if (selector == 'P' && DriveOn && SoC > 0) {  // when the selector is set to Park, some value are saved to be used the next time the car is started
    DriveOn = false;

    EEPROM.writeFloat(32, degrad_ratio);
    Serial.println("new_lost saved to EEPROM");
    EEPROM.writeFloat(36, PIDkWh_100);  //save actual kWh/100 in Flash memory
    EEPROM.writeFloat(44, TripOPtime);  //save initial trip time to Flash memory
    EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory
    EEPROM.writeFloat(52, acc_energy);
    EEPROM.writeFloat(56, SoC);
    EEPROM.writeFloat(60, kWh_100km);
    EEPROM.writeFloat(64, acc_Ah);
    EEPROM.writeFloat(68, acc_kWh_25);
    EEPROM.writeFloat(72, acc_kWh_10);
    EEPROM.writeFloat(76, acc_kWh_0);
    EEPROM.writeFloat(80, acc_kWh_m10);
    EEPROM.writeFloat(84, acc_kWh_m20);
    EEPROM.writeFloat(88, acc_kWh_m20p);
    EEPROM.writeFloat(92, acc_time_25);
    EEPROM.writeFloat(96, acc_time_10);
    EEPROM.writeFloat(100, acc_time_0);
    EEPROM.writeFloat(104, acc_time_m10);
    EEPROM.writeFloat(108, acc_time_m20);
    EEPROM.writeFloat(112, acc_time_m20p);
    EEPROM.writeFloat(116, acc_dist_25);
    EEPROM.writeFloat(120, acc_dist_10);
    EEPROM.writeFloat(124, acc_dist_0);
    EEPROM.writeFloat(128, acc_dist_m10);
    EEPROM.writeFloat(132, acc_dist_m20);
    EEPROM.writeFloat(136, acc_dist_m20p);
    EEPROM.writeFloat(140, acc_regen);
    EEPROM.commit();
  }
}

void stop_esp() {
  ESP_on = false;
  if (DriveOn && (mem_SoC > 0)) {
    EEPROM.writeFloat(32, degrad_ratio);
    Serial.println("new_lost saved to EEPROM");
    EEPROM.writeFloat(36, PIDkWh_100);  //save actual kWh/100 in Flash memory
    EEPROM.writeFloat(44, TripOPtime);  //save initial trip time to Flash memory
    EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory
    EEPROM.writeFloat(52, acc_energy);
    EEPROM.writeFloat(56, mem_SoC);
    EEPROM.writeFloat(60, kWh_100km);
    EEPROM.writeFloat(64, acc_Ah);
    EEPROM.writeFloat(68, acc_kWh_25);
    EEPROM.writeFloat(72, acc_kWh_10);
    EEPROM.writeFloat(76, acc_kWh_0);
    EEPROM.writeFloat(80, acc_kWh_m10);
    EEPROM.writeFloat(84, acc_kWh_m20);
    EEPROM.writeFloat(88, acc_kWh_m20p);
    EEPROM.writeFloat(92, acc_time_25);
    EEPROM.writeFloat(96, acc_time_10);
    EEPROM.writeFloat(100, acc_time_0);
    EEPROM.writeFloat(104, acc_time_m10);
    EEPROM.writeFloat(108, acc_time_m20);
    EEPROM.writeFloat(112, acc_time_m20p);
    EEPROM.writeFloat(116, acc_dist_25);
    EEPROM.writeFloat(120, acc_dist_10);
    EEPROM.writeFloat(124, acc_dist_0);
    EEPROM.writeFloat(128, acc_dist_m10);
    EEPROM.writeFloat(132, acc_dist_m20);
    EEPROM.writeFloat(136, acc_dist_m20p);
    EEPROM.writeFloat(140, acc_regen);
    EEPROM.commit();
  }
  
  if (!sd_condition2){
    tft.setTextSize(1);
    tft.setFreeFont(&FreeSans18pt7b);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    if (WiFi.status() == WL_CONNECTED){
      tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 - 50);
      tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);
      WiFi.disconnect();
      Serial.println("Wifi Stopped");
    }
    else{
      tft.drawString("Going", tft.width() / 2, tft.height() / 2 - 50);
      tft.drawString("Stand", tft.width() / 2, tft.height() / 2);
      tft.drawString("By", tft.width() / 2, tft.height() / 2 + 50);
    }
    
    delay(1000);
    
    shutdown_esp = false;
    send_enabled = false;
    wifiReconn = false;
    DrawBackground = true;
  }
  else{
    tft.setTextSize(1);
    tft.setFreeFont(&FreeSans18pt7b);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.drawString("ESP", tft.width() / 2, tft.height() / 2 - 50);
    tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);    
    delay(1000);
    esp_deep_sleep_start();
  }  
}

//--------------------------------------------------------------------------------------------
//                   Touch Button Handling Function
//--------------------------------------------------------------------------------------------

void button(){
  if (ts.touched()) {
    p = ts.getPoint();
    x = p.x;
    y = p.y;
          
    //Button 1 test
    if ((x >= btnAon.xStart && x <= btnAon.xStart + btnAon.xWidth) && (y >= btnAon.yStart && y <= btnAon.yStart + btnAon.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;      
      if (TouchTime >= 3 & !TouchLatch){
        Serial.println("Button1 Long Press");
        TouchLatch = true;        
        InitRst = true;            
        PrevSoC = 0;
      }            
      if (!Btn1SetON)
      {  
          screenNbr = 0;
          Serial.println("Button1 Touched");        
          Serial.println("Button1 set to ON");          
          DrawBackground = true;                 
          Btn1SetON = true;
        if (Btn2SetON){
          Btn2SetON = false;        
        }
        if (Btn3SetON){
          Btn3SetON = false;        
        }
      }   
    } 
        
    //Button 2 test
    if ((x >= btnBon.xStart && x <= btnBon.xStart + btnBon.xWidth) && (y >= btnBon.yStart && y <= btnBon.yStart + btnBon.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 3 & !TouchLatch){
        TouchLatch = true;        
        Serial.println("Button2 Long Press");
        if (!send_enabled){
          //ledcWrite(pwmLedChannelTFT, 80);
          send_enabled = true;
          StartWifi = true;
        }
        else{
          //ledcWrite(pwmLedChannelTFT, 120);
          send_enabled = false;
          StartWifi = false;
        }
        
        Serial.println("DONE");        
        Btn2SetON = true;
      }
      if (!Btn2SetON)
      {            
        screenNbr = 1;
        Serial.println("Button2 Touched");        
        Serial.println("Button2 set to ON");        
        DrawBackground = true;        
        Btn2SetON = true; 
        if (Btn1SetON){ 
          Btn1SetON = false;        
        }
        if (Btn3SetON){ 
          Btn3SetON = false;        
        }       
      }           
    }

    //Button 3 test
    if ((x >= btnCon.xStart && x <= btnCon.xStart + btnCon.xWidth) && (y >= btnCon.yStart && y <= btnCon.yStart + btnCon.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 3 & !TouchLatch){
        TouchLatch = true;        
        Serial.println("Button3 Long Press");
        ledcWrite(pwmLedChannelTFT, 0);
        tft.writecommand(ST7789_DISPOFF); // Switch off the display
        tft.writecommand(ST7789_SLPIN); // Sleep the display driver
        display_off = true;
      }
      if (!Btn3SetON)
      {            
        screenNbr = 2;
        Serial.println("Button3 Touched");        
        Serial.println("Button3 set to ON");        
        DrawBackground = true;        
        Btn3SetON = true;              
        if (Btn1SetON){
          Btn1SetON = false;        
        }
        if (Btn2SetON){
          Btn2SetON = false;       
        }
      }      
    }

    //Button 4 test
    if (x >= 0 && x <= 320 && y >= 65 && y <= 400 && (screenNbr == 1 || screenNbr == 3)) {
      if (!TouchLatch && screenNbr == 1)
      {            
        screenNbr = 3;
        Serial.println("Screen Touched");
        TouchLatch = true;        
        DrawBackground = true;
      }
      else if (!TouchLatch && screenNbr == 3)
      {            
        screenNbr = 1;
        Serial.println("Screen Touched");
        TouchLatch = true;
        DrawBackground = true;
      }
    }

    //Button 5 test
    if (x >= 0 && x <= 320 && y >= 65 && y <= 400 && (screenNbr == 0 || screenNbr == 2)) {
      if (!TouchLatch && screenNbr == 0)
      {            
        screenNbr = 2;
        Serial.println("Screen Touched");
        TouchLatch = true;        
        DrawBackground = true;
        Btn1SetON = false;                
      }
      else if (!TouchLatch && screenNbr == 2)
      {            
        screenNbr = 0;
        Serial.println("Screen Touched");
        TouchLatch = true;
        DrawBackground = true;
        Btn3SetON = false;
      }
    }

    //Button 6 test
    if (x >= 0 && x <= 320 && y >= 0 && y <= 60 && screenNbr == 2) {
      TouchTime = (millis() - initTouchTime) / 1000;
      if (!TouchLatch && TouchTime >= 2)
      {            
        Serial.println("Screen Touched");
        TouchLatch = true;        
        reset_trip();                
      }      
    }
  }
  else{
    initTouchTime = millis();
    TouchLatch = false;
  }
}


//--------------------------------------------------------------------------------------------
//                        Button Draw function
//--------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------
//                   Format Displays in Pages
//--------------------------------------------------------------------------------------------

void DisplayPage() {
  if (DrawBackground) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextPadding(160);
    
    // Draw parameter names
    for (int i = 0; i < 10; i++) {  
      if (i < 5) { // update left colunm
        tft.drawString(titre[i], tft.width() / 4, textLvl[i], 1);
      }
      else { // update right colunm
        tft.drawString(titre[i], 3 * (tft.width() / 4), textLvl[i], 1);
      }
    }

    // Draw frame lines
    tft.drawLine(1,textLvl[0] - 10,319,textLvl[0] - 10,TFT_DARKGREY);
    tft.drawLine(tft.width() / 2,textLvl[0] - 10,tft.width() / 2,drawLvl[4] + 24,TFT_DARKGREY);
    tft.drawLine(1,drawLvl[4] + 24,319,drawLvl[4] + 24,TFT_DARKGREY);

    // Initialize previous values to empty value so value will be updated when background is redrawn
    for (int i = 0; i < 10; i++) {
      strcpy(prev_value[i], "");
    }
    
    // button state to display
    switch (screenNbr) {  
      
      case 0: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAon);      
        drawRoundedRect(btnBoff);
        drawRoundedRect(btnCoff);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(MainTitleColor, TFT_BLACK);                
        tft.drawString(Maintitre[screenNbr], tft.width() / 2, 22, 1); 
        break; 
         
      case 1: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAoff);      
        drawRoundedRect(btnBon);
        drawRoundedRect(btnCoff);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(MainTitleColor, TFT_BLACK);                
        tft.drawString(Maintitre[screenNbr], tft.width() / 2, 22, 1); 
        break;
        
      case 2: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAoff);      
        drawRoundedRect(btnBoff);
        drawRoundedRect(btnCon);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(MainTitleColor, TFT_BLACK);                
        tft.drawString(Maintitre[screenNbr], tft.width() / 2, 22, 1); 
        break;

      case 3: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAoff);      
        drawRoundedRect(btnBoff);
        drawRoundedRect(btnCoff);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(MainTitleColor, TFT_BLACK);                
        tft.drawString(Maintitre[screenNbr], tft.width() / 2, 22, 1); 
        break;
    }
    DrawBackground = false;    
  }
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans18pt7b);

  // test for negative values and set negative flag
  for (int i = 0; i < 10; i++) {  
    if (value_float[i] < 0) {
    //value_float[i] = abs(value_float[i]);
      negative_flag[i] = true;
    }
    else {
      negative_flag[i] = false;
    }
    
    dtostrf(value_float[i], 3, nbr_decimal[i], value[i]);    
    
    //if value changes update values    
    if ((value[i] != prev_value[i]) && (i < 5)) { // update left colunm
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(prev_value[i], tft.width() / 4, drawLvl[i], 1);
      if (negative_flag[i]) {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.drawString(value[i], tft.width() / 4, drawLvl[i], 1);
      } 
      else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString(value[i], tft.width() / 4, drawLvl[i], 1);
      }
      strcpy(prev_value[i], value[i]);
    }
    else if (value[i] != prev_value[i]) { // update right colunm
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(prev_value[i], 3 * (tft.width() / 4), drawLvl[i], 1);
      if (negative_flag[i]) {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.drawString(value[i], 3 * (tft.width() / 4), drawLvl[i], 1);
      } 
      else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString(value[i], 3 * (tft.width() / 4), drawLvl[i], 1);
      }
      strcpy(prev_value[i], value[i]);
    }  
  }
}

//-------------------------------------------------------------------------------------
//             Start of Pages content definition
//-------------------------------------------------------------------------------------

/*///////////////// Display Page 1 //////////////////////*/
void page1() {

  strcpy(titre[0],"SoC");
  strcpy(titre[1],"PID Cons");
  strcpy(titre[2],"PWR Int Cons");
  strcpy(titre[3],"Cons. 10Km");
  strcpy(titre[4],"CorrLeft_kWh");
  strcpy(titre[5],"TripOdo");  
  strcpy(titre[6],"Est. range");
  strcpy(titre[7],"Est. range");  
  strcpy(titre[8],"Est. range");
  strcpy(titre[9],"used kwh");
  value_float[0] = SoC;  
  value_float[1] = PIDkWh_100;
  value_float[2] = kWh_100km;
  value_float[3] = span_kWh_100km;
  value_float[4] = EstLeft_kWh;  
  value_float[5] = TripOdo;
  value_float[6] = Est_range3;
  value_float[7] = Est_range;
  value_float[8] = Est_range2;
  value_float[9] = used_kwh;
  
  // set number of decimals for each value to display
  for (int i = 0; i < 10; i++) {  
    if (value_float[i] >= 1000) {
      nbr_decimal[i] = 0;
    }
    else if ((value_float[i] < 10) && (value_float[i] > -10)) {
      nbr_decimal[i] = 2;
    } 
    else {
      nbr_decimal[i] = 1;
    }
  }
  
  DisplayPage();
}
/*///////////////// End of Display Page 1 //////////////////////*/

/*///////////////// Display Page 2 //////////////////////*/
void page2() {

  strcpy(titre[0], "SoC");
  strcpy(titre[1], "kWh Left");
  strcpy(titre[2], "MAXcellv");
  strcpy(titre[3], "SOH");
  strcpy(titre[4], "Full Ah");
  strcpy(titre[5], "BmsSoC");
  strcpy(titre[6], "BATTv");
  strcpy(titre[7], "Cell Vdiff");
  strcpy(titre[8], "Det. Total");
  strcpy(titre[9], "12V SoC");
  value_float[0] = SoC;
  value_float[1] = left_kwh;
  value_float[2] = MAXcellv;
  value_float[3] = SOH;
  value_float[4] = EstFull_Ah;
  value_float[5] = BmsSoC;
  value_float[6] = BATTv;
  value_float[7] = CellVdiff;
  value_float[8] = Deter_Min;
  value_float[9] = AuxBattSoC;
  
  // set number of decimals for each value to display
  for (int i = 0; i < 10; i++) {  
    if (value_float[i] >= 1000) {
      nbr_decimal[i] = 0;
    }
    else if ((value_float[i] < 10) && (value_float[i] > -10)) {
      nbr_decimal[i] = 2;
    }
    else {
      nbr_decimal[i] = 1;
    }
  }
  
  DisplayPage();
}
/*///////////////// End of Display Page 2 //////////////////////*/

/*///////////////// Display Page 3 //////////////////////*/
void page3() {

  strcpy(titre[0], "Power");
  strcpy(titre[1], "MIN Temp");
  strcpy(titre[2], "PID kWh");
  strcpy(titre[3], "kWh Used");
  strcpy(titre[4], "SoC");
  strcpy(titre[5], "Max Pwr");
  strcpy(titre[6], "MAX Temp");
  strcpy(titre[7], "Int. Energ");
  strcpy(titre[8], "kWh Left");
  strcpy(titre[9], "Chauf. Batt.");  
  value_float[0] = Power;
  value_float[1] = BattMinT;
  value_float[2] = Net_kWh;
  value_float[3] = used_kwh;
  value_float[4] = SoC;
  value_float[5] = Max_Pwr;
  value_float[6] = BattMaxT;
  value_float[7] = acc_energy;
  value_float[8] = left_kwh;
  value_float[9] = Heater;

  // set number of decimals for each value to display
  for (int i = 0; i < 10; i++) {  
    if (value_float[i] >= 1000) {
      nbr_decimal[i] = 0;
    }
    else if ((value_float[i] < 10) && (value_float[i] > -10)) {
      nbr_decimal[i] = 2;
    } 
    else {
      nbr_decimal[i] = 1;
    }
  }
  
  DisplayPage();
}
/*///////////////// End of Display Page 3 //////////////////////*/

/*///////////////// Display Page 4 //////////////////////*/
void page4() {

  strcpy(titre[0], "COOLtemp");
  strcpy(titre[1], "MINcellv");
  strcpy(titre[2], "MAXcellv");
  strcpy(titre[3], "Max_Reg");
  strcpy(titre[4], "SoCratio");
  strcpy(titre[5], "degrad_ratio");
  strcpy(titre[6], "Cell nbr");
  strcpy(titre[7], "Cell nbr");
  strcpy(titre[8], "OUT temp");
  strcpy(titre[9], "IN temp");
  value_float[0] = COOLtemp;
  value_float[1] = MINcellv;
  value_float[2] = MAXcellv;
  value_float[3] = Max_Reg;
  value_float[4] = SoCratio;
  value_float[5] = degrad_ratio;
  value_float[6] = MINcellvNb;
  value_float[7] = MAXcellvNb;
  value_float[8] = OUTDOORtemp;
  value_float[9] = INDOORtemp;
  
  // set number of decimals for each value to display
  for (int i = 0; i < 10; i++) {  
    if (value_float[i] >= 1000) {
      nbr_decimal[i] = 0;
    }
    else if ((value_float[i] < 10) && (value_float[i] > -10)) {
      nbr_decimal[i] = 2;
    }
    else {
      nbr_decimal[i] = 1;
    }
  }
  
  DisplayPage();
}
/*///////////////// End of Display Page 4 //////////////////////*/

/*///////////////////////////////////////////////////////////////////////*/
/*                     START OF LOOP                                     */
/*///////////////////////////////////////////////////////////////////////*/

void loop() { 

  /*/////// Read each OBDII PIDs /////////////////*/     
  if (BMS_relay || ResetOn){
    pid_counter++;
    read_data();    
  }
  else if ((millis() - read_timer) > read_data_interval){ // if BMS is not On, only scan OBD2 at some intervals
    read_data();
    read_timer = millis();
    pid_counter = 0;        
  }
  
  /*/////// Check if touch buttons are pressed /////////////////*/
  button();
    
  /*/////// This will trigger logic to send data to Google sheet /////////////////*/    
  if (millis() - GSheetTimer >= sendInterval){
    GSheetTimer = millis();
    if (send_enabled) {
      ready = GSheet.ready();
      
      if (ready) {
        // Get timestamp
        t = getTime();
        
        if (((nbrDays[month(t) - 1] + day(t)) >= 70) && ((nbrDays[month(t) - 1] + day(t)) <= 308)) {  //  summer time logic
          t = t - 14400;
        }      
        else{
          t = t - 18000;
        }                   
        send_data = true;  // This will trigger logic to send data to Google sheet 
      }
    }    
    if (!send_enabled) {
      sendIntervalOn = true;
    }       
  }

  //  To display a led status when values are sent to Google Sheet
  if (datasent){
    tft.fillCircle(20, 20, 6,TFT_GREEN);
    if (millis() - GSheetTimer >= 500){  // turn led off 500mS after it was turned On
      datasent = false;
    }
  }
  else{
    tft.fillCircle(20, 20, 6,TFT_BLACK);
  }  
  
  /*/////// Display Page Number /////////////////*/

  if ((ESP_on || (BMS_relay && Power < 0)) && SoC != 0 && !sd_condition1) {    
    
    if (display_off){      
      ledcWrite(pwmLedChannelTFT, 128);
      tft.writecommand(ST7789_SLPOUT);// Wakes up the display driver
      tft.writecommand(ST7789_DISPON); // Switch on the display      
      display_off = false;
      SoC_saved = false;
      if ((WiFi.status() != WL_CONNECTED) && !wifiReconn && StartWifi) {  // If esp32 is On when start the car, reconnect wifi if not connected
        ConnectWifi(tft);
        wifiReconn = true;
        if (WiFi.status() == WL_CONNECTED) {
          send_enabled = true;
          send_data = true;
          initscan = true;  // To write header name on Google Sheet
        }        
      }
      DrawBackground = true;
    }

    if (StartWifi) {  // If wifi is configured then display wifi status led
      if (WiFi.status() == WL_CONNECTED){
          tft.fillCircle(300, 20, 6,TFT_GREEN);
        }
      else{
        tft.fillCircle(300, 20, 6,TFT_RED);
      }
    }
        
    switch (screenNbr) {  // select page to display
      case 0: page1(); break;
      case 1: page2(); break;
      case 2: page3(); break;
      case 3: page4(); break;    
    }
  }
  /*/////// Turn off display when BMS is off /////////////////*/
  else {    
    ledcWrite(pwmLedChannelTFT, 0);
    tft.writecommand(ST7789_DISPOFF); // Switch off the display
    tft.writecommand(ST7789_SLPIN); // Sleep the display driver
    display_off = true;
    ESP_on = false;
  }

  /*/////// Stop ESP /////////////////*/
   
  if (!BMS_ign && ESP_on && (SpdSelect == 'P')) {  // When car is power off, call stop_esp which saves some data before powering ESP32 down
    record_code = 5;
    shutdown_esp = true;
    if (!SoC_saved) {
      mem_SoC = SoC;
      SoC_saved = true;
      stopESP_timer = millis();
    }
    
    if (!send_enabled) {
      Serial.println("No Code sent and Normal shutdown");
      stop_esp();
    }
    else if (code_sent || ((millis() - stopESP_timer) > (sendInterval + 2000))) {  // wait for code being sent or stop if code was not sent within 7 secondes
      Serial.println("Code sent and Normal shutdown");
      stop_esp();
    }
  }

  else if (!BMS_ign && BMS_relay && data_ready && ((Power >= 0) || (AuxBattSoC < 75))) {  // When the car is off but the BMS does some maintnance check, wait 20 mins before esp32 power down
    if (!SoC_saved) {
      ESPinitTimer = millis();
      mem_SoC = SoC;
      SoC_saved = true;
    }   
    shutdown_timer = (millis() - ESPinitTimer) / 1000;    
    if (shutdown_timer >= ESPTimerInterval){
      sd_condition1 = true;
    }
    sd_condition2 = ((AuxBattSoC > 0) && (AuxBattSoC < 75));
    if (sd_condition1 || sd_condition2) {

      if (sd_condition1) {
        record_code = 6;
        shutdown_esp = true;
        Serial.println("Code sent and Timer shutdown");
      } 
      else if (sd_condition2) {
        record_code = 7;
        shutdown_esp = true;
        Serial.println("Code sent and Low batt shutdown");
      }
      
      if (!send_enabled) {               
        stop_esp();
      }
      else if (code_sent || (shutdown_timer > (ESPTimerInterval + 2000))) {        
        stop_esp();
      }     
    }
  }
  else if (display_off && send_enabled){
    stop_esp();
  }

  ResetCurrTrip();  
}
