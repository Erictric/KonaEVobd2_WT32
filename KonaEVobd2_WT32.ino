
/*  KonaEvObd for Hyundai Kona EV + OBD Vgate iCar Pro BT4.0 + WT32-SC01 3.5" display
    Version: v3.01

    SafeString by Matthew Ford: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
    Elmduino by PowerBroker2: https://github.com/PowerBroker2/ELMduino
    https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/

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
#include "Button.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "BT_communication.h"
#include "Wifi_connection.h"
#include "FreeRTOSConfig.h"
#include "WiFi.h"
#include "Free_Fonts.h"
#include <Adafruit_FT6206.h>
//#include "Button_Select.h"

#define DEBUG_PORT Serial

TaskHandle_t Task1;
//TaskHandle_t Task2;

TFT_eSPI tft = TFT_eSPI();
Adafruit_FT6206 ts = Adafruit_FT6206();

//TFT y positions for texts and numbers
#define textLvl1 70  // y coordinates for text
#define textLvl2 140
#define textLvl3 210
#define textLvl4 280
#define textLvl5 350
#define drawLvl1 105  // and numbers
#define drawLvl2 175
#define drawLvl3 245
#define drawLvl4 315
#define drawLvl5 385  // TTGO 320x480 TFT display

//#define BUTTON_PIN 0
//#define BUTTON_2_PIN 35
#define Threshold 40 /* Greater the value, more the sensitivity */

RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

//Button bouton(BUTTON_PIN);
//Button bouton2(BUTTON_2_PIN);

#define pagenumbers 7  // number of pages to display
#define N_km 10        //variable for the calculating kWh/100km over a N_km

const int VESSoff = 12;
boolean SelectOn = true;
unsigned long StartMillis;
unsigned long ToggleDelay = 1000;
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

float BattMinT;
float BattMaxT;
float AuxBattV;
float AuxBattC;
float AuxBattSoC = 80;
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
int array_size = N_km + 1;
float energy_array[11];
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
float MeanSpeed;
float Time_100km;
float TripkWh_100km;
float kWh_100km;
float span_kWh_100km;
float PIDkWh_100;
float Est_range;
float Est_range2;
float Est_range3;
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
bool StayOn = false;
bool SetupOn = false;
bool StartWifi = true;
bool initscan = false;
bool InitRst = false;
bool TrigRst = false;
bool kWh_update = false;
bool corr_update = false;
bool ESP_on = false;
bool DrawBackground = true;
char title1[12];
char title2[12];
char title3[12];
char title4[12];
char title5[12];
char title6[12];
char title7[12];
char title8[12];
char title9[12];
char title10[12];
char value1[5];
char value2[5];
char value3[5];
char value4[5];
char value5[5];
char value6[5];
char value7[5];
char value8[5];
char value9[5];
char value10[5];
char prev_value1[5];
char prev_value2[5];
char prev_value3[5];
char prev_value4[5];
char prev_value5[5];
char prev_value6[5];
char prev_value7[5];
char prev_value8[5];
char prev_value9[5];
char prev_value10[5];
bool negative_flag1;
bool negative_flag2;
bool negative_flag3;
bool negative_flag4;
bool negative_flag5;
bool negative_flag6;
bool negative_flag7;
bool negative_flag8;
bool negative_flag9;
bool negative_flag10;
float value1_float;
float value2_float;
float value3_float;
float value4_float;
float value5_float;
float value6_float;
float value7_float;
float value8_float;
float value9_float;
float value10_float;
int nbr_decimal1;
int nbr_decimal2;
int nbr_decimal3;
int nbr_decimal4;
int nbr_decimal5;
int nbr_decimal6;
int nbr_decimal7;
int nbr_decimal8;
int nbr_decimal9;
int nbr_decimal10;
bool Charge_page = false;
bool Power_page = false;

bool datasent = false;

// Variables for touch x,y
static int32_t x, y;
TS_Point p;
static int xMargin = 20, yMargin = 420, margin = 20, btnWidth = 80, btnHeigth = 55;
char* BtnAtext = "MAIN";
char* BtnBtext = "BATT";
char* BtnCtext = "POWER";
char MainTitle[12];

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
  xMargin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_GREEN,
  BtnAtext
};

RoundedRect btnBgreen = {
  btnAgreen.xStart + btnAgreen.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_GREEN,
  BtnBtext
};

RoundedRect btnCgreen = {
  btnBgreen.xStart + btnBgreen.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_GREEN,
  BtnCtext
};

RoundedRect btnAred = {
  xMargin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_WHITE,
  BtnAtext
};

RoundedRect btnBred = {
  btnAred.xStart + btnAred.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_WHITE,
  BtnBtext
};

RoundedRect btnCred = {
  btnBred.xStart + btnBred.xWidth + margin,
  yMargin,
  btnWidth,
  btnHeigth,
  4,
  TFT_WHITE,
  BtnCtext
};

unsigned long ESPinitTimer = 0;
unsigned long ESPTimer = 0;
unsigned long ESPTimerInterval = 1200;  // time in seconds to turn off ESP when it power-up during 12V battery charge cycle.
unsigned long shutdown_timer = 0;

/*////// Variables for Google Sheet data transfer ////////////*/
bool send_enabled = false;
bool send_data = false;
bool data_sent = false;
int nbParam = 76;  //number of parameters to send to Google Sheet
unsigned long sendInterval = 5000;
unsigned long currentTimer = 0;
unsigned long previousTimer = 0;
bool sendIntervalOn = false;

const char* resource = "/trigger/SendData/with/key/bWQadqBNOh1P3PINCC1_Vr";  // Copy key from IFTTT applet

// Maker Webhooks IFTTT
const char* server = "maker.ifttt.com";

static bool flag = false;

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

void callback(){
  //placeholder callback function
}

/*////////////////////////////////////////////////////////////////////////*/
/*                         START OF SETUP                                 */
/*////////////////////////////////////////////////////////////////////////*/

void setup() {

  /*////////////////////////////////////////////////////////////////*/
  /*              Open serial monitor communications                */
  /*////////////////////////////////////////////////////////////////*/

  Serial.begin(115200);

  // Pins 18/19 are SDA/SCL for touch sensor on this device
  // 40 is a touch threshold
  if (!ts.begin(18, 19, 40)) {
    Serial.println("Couldn't start touchscreen controller");
    while (true);
  }
  tft.init();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 128);

  

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Serial Monitor - STARTED");


  //pinMode(VESSoff, OUTPUT); // enable output pin that activate a relay to temporary disable the VESS

  /*//////////////Initialise OLED display ////////////////*/

  tft.begin();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.setFreeFont(&FreeSans9pt7b);

  /*////// initialize EEPROM with predefined size ////////*/
  EEPROM.begin(148);

  /* uncomment if you need to display Safestring results on Serial Monitor */
  //SafeString::setOutput(Serial);
  
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Setup interrupt on Touch Pad 2 (GPIO2)
  touchAttachInterrupt(T2, callback, Threshold);

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup();
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_19,0);  // initialize ESP wakeup on button 1 activation

  /*////// Get the stored values from last re-initialisation /////*/

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
  }

  initscan = true;  // To write header name on Google Sheet on power up

  /*//////////////Initialise Task on core0 to send data on Google Sheet ////////////////*/

  xTaskCreatePinnedToCore(
    makeIFTTTRequest,   /* Function to implement the task */
    "makeIFTTTRequest", /* Name of the task */
    10000,              /* Stack size in words */
    NULL,               /* Task input parameter */
    5,                  /* Priority of the task */
    &Task1,             /* Task handle. */
    0);                 /* Core where the task should run */
  delay(500);

  tft.fillScreen(TFT_BLACK);

  integrate_timer = millis() / 1000;

  nbr_powerOn += 1;
  EEPROM.writeFloat(40, nbr_powerOn);
  EEPROM.commit();  
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
//         Data retreived from OBD2 and extract values of it
//------------------------------------------------------------------------------------------

void read_data() {

  pid_counter++;
  Serial.println(pid_counter);

  button();

  // read in rawData via ODBII
  
  //  Read PID 220101 each iteration to get faster battery power update
  myELM327.sendCommand("AT SH 7E4");  // Set Header for BMS

  if (myELM327.queryPID("220101")) {  // Service and Message PID = hex 22 0101 => dec 34, 257
    Serial.println("read PID101");

    char* payload = myELM327.payload;
    size_t payloadLen = myELM327.recBytes;

    processPayload(payload, payloadLen, results);

    int BattMinTraw = convertToInt(results.frames[2], 6, 1);  //specify frame#, starting Byte(o in TorquePro) and # of bytes required
    if (BattMinTraw > 127) {                                  //conversition for negative value
      BattMinT = -1 * (256 - BattMinTraw);
    } else {
      BattMinT = BattMinTraw;
    }
    int BattMaxTraw = convertToInt(results.frames[2], 5, 1);  //specify frame#, starting Byte(o in TorquePro) and # of bytes required
    if (BattMaxTraw > 127) {                                  //conversition for negative value
      BattMaxT = -1 * (256 - BattMaxTraw);
    } else {
      BattMaxT = BattMaxTraw;
    }
    AuxBattV = convertToInt(results.frames[4], 6, 1) * 0.1;
    BATTv = convertToInt(results.frames[2], 3, 2) * 0.1;
    int CurrentByte1 = convertToInt(results.frames[2], 1, 1);
    int CurrentByte2 = convertToInt(results.frames[2], 2, 1);
    if (CurrentByte1 > 127) {  // the most significant bit is the sign bit so need to calculate commplement value if true
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
  
  // Read remaining PIDs
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
        if (HeaterRaw > 127) {  //conversition for negative value
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
        if (COOLtempRaw > 127) {                                         //conversition for negative value
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
        if (AuxCurrByte1 > 127) {  // the most significant bit is the sign bit so need to calculate commplement value if true
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
      data_ready = true;  // after all PIDs have been read, turn on flag for valid values from OBD2
      break;
  }


  /////// Miscellaneous calculations /////////

  Power = (BATTv * BATTc) * 0.001;
  Integrat_power();
  Integrat_current();
  integrate_timer = millis();

  if (!ResetOn) {  // On power On, wait for current trip values to be re-initialized before executing the next lines of code
    TripOdo = Odometer - InitOdo;

    CurrTripOdo = Odometer - CurrInitOdo;

    CurrOPtime = OPtimemins - CurrTimeInit;

    TripOPtime = CurrOPtime + PrevOPtimemins;

    UsedSoC = InitSoC - SoC;

    CurrUsedSoC = CurrInitSoC - SoC;

    EstFull_Ah = 100 * Net_Ah / UsedSoC;

    CellVdiff = MAXcellv - MINcellv;

    if (PrevBmsSoC > BmsSoC) {  // perform a BmsSoC vs SoC ratio calculation when BmsSoC changes
      PrevBmsSoC = BmsSoC;
      SocRatioCalc();
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
        initscan = true;
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
            if ((degrad_ratio > 1.2) || (degrad_ratio < 0.8)) {  // if a bad value got saved previously, initialize ratio to 1
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

    RangeCalc();
    if (BMS_ign) {
      EnergyTOC();
    }

    if (Max_Pwr < 100 && (Max_Pwr < (Power + 20)) && !Power_page) {  //select the Max Power page if Power+20kW exceed Max_Pwr when Max_Pwr is lower then 100kW.
      DrawBackground = true;
      screenNbr = 2;
      Power_page = true;
    }
    if (Power < 0 && (SpdSelect == 'P') && !Charge_page) {
      DrawBackground = true;
      screenNbr = 2;
      Charge_page = true;
    }
  }

  save_lost(SpdSelect);
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Calculation Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Discharge Energy Since last reset //////////*/

void UpdateNetEnergy() {

  if (InitCED == 0) {  //if discharge value have been reinitiate to 0 then
    InitCED = CED;     //initiate to current CED for initial CED value and
    InitSoC = SoC;     //initiate to current CED for initial SoC value and
    CurrInitCED = CED;
  }
  if (InitCDC == 0) {
    InitCDC = CDC;
  }
  if (InitCEC == 0) {  //if charge value have been reinitiate to 0 then
    InitCEC = CEC;     //initiate to current CEC for initial CEC value
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
//                   Net Energy based on Power integration Function
//--------------------------------------------------------------------------------------------

/*//////Function to calculate Energy by power integration since last reset //////////*/

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
      InitOdo = Odometer - distance;  // correct initial odometer value using speed integration if odometer changes within 0.9km
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
//                   Function to calculate energy between two SoC values
//--------------------------------------------------------------------------------------------

double Interpolate(double xValues[], double yValues[], int numValues, double pointX, bool trim = true) {
  if (trim) {
    if (pointX <= xValues[0]) return yValues[0];
    if (pointX >= xValues[numValues - 1]) return yValues[numValues - 1];
  }

  auto i = 0;
  if (pointX <= xValues[0]) i = 0;
  else if (pointX >= xValues[numValues - 1]) i = numValues - 1;
  else
    while (pointX >= xValues[i + 1]) i++;
  if (pointX == xValues[i + 1]) return yValues[i + 1];

  auto t = (pointX - xValues[i]) / (xValues[i + 1] - xValues[i]);
  t = t * t * (3 - 2 * t);
  return yValues[i] * (1 - t) + yValues[i + 1] * t;
}

float calc_kwh(float min_SoC, float max_SoC) {
  /* variable for kWh/%SoC calculation: xValues = %SoC and yValues = kWh */
  const int numValues = 21;
  double xValues[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
  //double yValues[] = { 0.5487, 0.5921, 0.5979, 0.6053, 0.6139, 0.6199, 0.6238, 0.6268, 0.6295, 0.6324, 0.6362, 0.6418, 0.6524, 0.6601, 0.6684, 0.6771, 0.6859, 0.6951, 0.7046, 0.7147, 0.7249};
  double yValues[] = { 0.5432, 0.5867, 0.5931, 0.6011, 0.6102, 0.6168, 0.6213, 0.6249, 0.6282, 0.6317, 0.6362, 0.6424, 0.6537, 0.6621, 0.6711, 0.6805, 0.6900, 0.7000, 0.7102, 0.7211, 0.7321 };
  float integral;
  float interval;
  float return_kwh;
  static int N = 100;
  interval = (max_SoC - min_SoC) / N;
  integral = 0, 0;
  float x = 0;
  for (int i = 0; i < N; ++i) {
    x = min_SoC + interval * i;
    integral += Interpolate(xValues, yValues, numValues, x);  //64kWh battery energy equation
  }
  //return_kwh = integral * interval;
  return_kwh = (integral * interval) * Calc_kWh_corr;
  return return_kwh;
}

//----------------------------------------------------------------------------------------
//        Task on core 0 to Send data to Google Sheet via IFTTT web service Function
//----------------------------------------------------------------------------------------

void makeIFTTTRequest(void* pvParameters) {
  for (;;) {
    if (send_enabled && send_data) {      
      code_sent = false;
      Serial.print("Connecting to ");
      Serial.print(server);

      WiFiClient client;
      int retries = 5;
      while (!!!client.connect(server, 80) && (retries-- > 0)) {
        Serial.print(".");
      }
      Serial.println();
      if (!!!client.connected()) {
        Serial.println("Failed to connect...");
        code_sent = true;
      }

      Serial.print("Request resource: ");
      Serial.println(resource);

      float sensor_Values[nbParam];

      char column_name[][15] = { "SoC", "Power", "BattMinT", "Heater", "Net_Ah", "Net_kWh", "AuxBattSoC", "AuxBattV", "Max_Pwr", "Max_Reg", "BmsSoC", "MAXcellv", "MINcellv", "MAXcellvNb", "MINcellvNb", "BATTv", "BATTc", "Speed", "Odometer", "CEC", "CED", "CDC", "CCC", "SOH", "BMS_ign", "OPtimemins", "OUTDOORtemp", "INDOORtemp", "SpdSelect", "LastSoC", "Calc_Used", "Calc_Left", "TripOPtime", "CurrOPtime", "PIDkWh_100", "kWh_100km", "degrad_ratio", "EstLeft_kWh", "span_kWh_100km", "SoCratio", "nbr_powerOn", "TireFL_P", "TireFR_P", "TireRL_P", "TireRR_P", "TireFL_T", "TireFR_T", "TireRL_T", "TireRR_T", "acc_energy", "Trip_dist", "distance", "BattMaxT", "acc_Ah", "acc_kWh_25", "acc_kWh_10", "acc_kWh_0", "acc_kWh_m10", "acc_kWh_m20", "acc_kWh_m20p", "acc_time_25", "acc_time_10", "acc_time_0", "acc_time_m10", "acc_time_m20", "acc_time_m20p", "acc_dist_25", "acc_dist_10", "acc_dist_0", "acc_dist_m10", "acc_dist_m20", "acc_dist_m20p", "acc_regen", "MaxDetNb", "MinDetNb", "Deter_Min" };
      ;

      sensor_Values[0] = SoC;
      sensor_Values[1] = Power;
      sensor_Values[2] = BattMinT;
      sensor_Values[3] = Heater;
      sensor_Values[4] = Net_Ah;
      sensor_Values[5] = Net_kWh;
      sensor_Values[6] = AuxBattSoC;
      sensor_Values[7] = AuxBattV;
      sensor_Values[8] = Max_Pwr;
      sensor_Values[9] = Max_Reg;
      sensor_Values[10] = BmsSoC;
      sensor_Values[11] = MAXcellv;
      sensor_Values[12] = MINcellv;
      sensor_Values[13] = MAXcellvNb;
      sensor_Values[14] = MINcellvNb;
      sensor_Values[15] = BATTv;
      sensor_Values[16] = BATTc;
      sensor_Values[17] = Speed;
      sensor_Values[18] = Odometer;
      sensor_Values[19] = CEC;
      sensor_Values[20] = CED;
      sensor_Values[21] = CDC;
      sensor_Values[22] = CCC;
      sensor_Values[23] = SOH;
      sensor_Values[24] = BMS_ign;
      sensor_Values[25] = OPtimemins;
      sensor_Values[26] = OUTDOORtemp;
      sensor_Values[27] = INDOORtemp;
      sensor_Values[28] = SpdSelect;
      sensor_Values[29] = LastSoC;
      sensor_Values[30] = used_kwh;
      sensor_Values[31] = left_kwh;
      sensor_Values[32] = TripOPtime;
      sensor_Values[33] = CurrOPtime;
      sensor_Values[34] = PIDkWh_100;
      sensor_Values[35] = kWh_100km;
      sensor_Values[36] = degrad_ratio;
      sensor_Values[37] = EstLeft_kWh;
      sensor_Values[38] = span_kWh_100km;
      sensor_Values[39] = SoCratio;
      sensor_Values[40] = nbr_powerOn;
      sensor_Values[41] = TireFL_P;
      sensor_Values[42] = TireFR_P;
      sensor_Values[43] = TireRL_P;
      sensor_Values[44] = TireRR_P;
      sensor_Values[45] = TireFL_T;
      sensor_Values[46] = TireFR_T;
      sensor_Values[47] = TireRL_T;
      sensor_Values[48] = TireRR_T;
      sensor_Values[49] = acc_energy;
      sensor_Values[50] = Trip_dist;
      sensor_Values[51] = distance;
      sensor_Values[52] = BattMaxT;
      sensor_Values[53] = acc_Ah;
      sensor_Values[54] = acc_kWh_25;
      sensor_Values[55] = acc_kWh_10;
      sensor_Values[56] = acc_kWh_0;
      sensor_Values[57] = acc_kWh_m10;
      sensor_Values[58] = acc_kWh_m20;
      sensor_Values[59] = acc_kWh_m20p;
      sensor_Values[60] = acc_time_25;
      sensor_Values[61] = acc_time_10;
      sensor_Values[62] = acc_time_0;
      sensor_Values[63] = acc_time_m10;
      sensor_Values[64] = acc_time_m20;
      sensor_Values[65] = acc_time_m20p;
      sensor_Values[66] = acc_dist_25;
      sensor_Values[67] = acc_dist_10;
      sensor_Values[68] = acc_dist_0;
      sensor_Values[69] = acc_dist_m10;
      sensor_Values[70] = acc_dist_m20;
      sensor_Values[71] = acc_dist_m20p;
      sensor_Values[72] = acc_regen;
      sensor_Values[73] = MaxDetNb;
      sensor_Values[74] = MinDetNb;
      sensor_Values[75] = Deter_Min;

      String headerNames = "";
      String payload = "";

      int i = 0;

      if (initscan || record_code != 0 || shutdown_esp) {
        switch (record_code) {
          case 0:  // No reset only header required, ESP32 power reboot
            while (i != nbParam) {
              if (i == 0) {
                headerNames = String("{\"value1\":\"") + column_name[i];
                i++;
              }
              if (i == nbParam)
                break;
              headerNames = headerNames + "|||" + column_name[i];
              i++;
            }
            initscan = false;
            break;

          case 1:  // Write status for Reset after a battery was recharged
            headerNames = String("{\"value1\":\"") + "|||" + "Battery_Recharged" + "|||" + "LastSoc:" + "|||" + mem_LastSoC + "|||" + "Soc:" + "|||" + mem_SoC + "|||" + "Power:" + "|||" + mem_Power;
            record_code = 0;
            initscan = true;
            break;

          case 2:  // Write status for Reset performed with reset button (right button)
            headerNames = String("{\"value1\":\"") + "|||" + "Button_Reset";
            record_code = 0;
            initscan = true;
            break;

          case 3:  // Write status for Reset when Acc_energy is less then 0.3kWh when SoC changes
            headerNames = String("{\"value1\":\"") + "|||" + "ACC_energy <0.3" + "|||" + "acc_energy:" + "|||" + mem_energy + "|||" + "PreSoC:" + "|||" + mem_PrevSoC + "|||" + "SoC:" + "|||" + mem_SoC;
            record_code = 0;
            initscan = true;
            break;

          case 4:  // Write status for Reset if SoC changes from 100 to 99% not going through 99.5%
            headerNames = String("{\"value1\":\"") + "|||" + "100_to_99SoC_reset" + "|||" + "PreSoc:" + "|||" + mem_PrevSoC + "|||" + "SoC:" + "|||" + mem_SoC;
            record_code = 0;
            initscan = true;
            break;

          case 5:  // Write that esp is going normal shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Normal Shutdown" + "|||" + "Power:" + "|||" + Power + "|||" + "SoC:" + "|||" + mem_SoC;            
            code_received = true;            
            Serial.println("Code Received");
            break;

          case 6:  // Write that esp is going timed shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Timer Shutdown" + "|||" + "Power:" + "|||" + Power + "|||" + "Timer:" + "|||" + shutdown_timer;            
            code_received = true;            
            break;

          case 7:  // Write that esp is going low 12V shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Low 12V Shutdown" + "|||" + "12V Batt.:" + "|||" + AuxBattSoC + "|||" + "Timer:" + "|||" + shutdown_timer;            
            code_received = true;            
            break;
        }
        payload = headerNames;
      }

      else {
        while (i != nbParam) {
          if (i == 0) {
            payload = String("{\"value1\":\"") + sensor_Values[i];
            i++;
          }
          if (i == nbParam) {
            break;
          }
          payload = payload + "|||" + sensor_Values[i];
          i++;
        }
      }

      String jsonObject = payload + "\"}";

      client.println(String("POST ") + resource + " HTTP/1.1");
      client.println(String("Host: ") + server);
      client.println("Connection: close\r\nContent-Type: application/json");
      client.print("Content-Length: ");
      client.println(jsonObject.length());
      client.println();
      client.println(jsonObject);
      
      int timeout = 5;  // 50 * 100mS = 5 seconds
      while (!!!client.available() && (timeout-- > 0)) {
        delay(100);
      }
      if (!!!client.available()) {
        Serial.println("No response...");
        code_sent = true;
      }
      while (client.available()) {
        Serial.write(client.read());
      }

      Serial.println();
      Serial.println("closing connection");
      datasent = true;
      client.stop();

      send_data = false;
      pwr_changed = 0;
      loop_count = 0;

      if (kWh_update) {  //add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update"
        Prev_kWh = Net_kWh;
        kWh_update = false;  // reset kWh_update after it has been recorded and so the correction logic start again
      }
      if (corr_update) {
        corr_update = false;  // reset corr_update after it has been recorded
      }
      if (code_received) {
        Serial.println("Sending code sent");
        code_sent = true;
      }
    }    
    vTaskDelay(30);  // some delay is required to reset watchdog timer    
  }
}

//--------------------------------------------------------------------------------------------
//                        Button functions
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

void button(){
  if (ts.touched()) {
    p = ts.getPoint();
    x = p.x;
    y = p.y;      
    //Button 1 test
    if ((x >= btnAgreen.xStart && x <= btnAgreen.xStart + btnAgreen.xWidth) && (y >= btnAgreen.yStart && y <= btnAgreen.yStart + btnAgreen.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;      
      if (TouchTime > 4 & !TouchLatch){
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
    if ((x >= btnBgreen.xStart && x <= btnBgreen.xStart + btnBgreen.xWidth) && (y >= btnBgreen.yStart && y <= btnBgreen.yStart + btnBgreen.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 4 & !TouchLatch){
        TouchLatch = true;        
        Serial.println("Button2 Long Press");
        screenNbr = 3;
        DrawBackground = true;        
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
    if ((x >= btnCgreen.xStart && x <= btnCgreen.xStart + btnCgreen.xWidth) && (y >= btnCgreen.yStart && y <= btnCgreen.yStart + btnCgreen.yHeight)) {      
      TouchTime = (millis() - initTouchTime) / 1000;
      if (TouchTime >= 4 & !TouchLatch){
        TouchLatch = true;        
        Serial.println("Button3 Long Press");
        screenNbr = 3;
        DrawBackground = true;        
        Btn3SetON = true;
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
  }
  else{
    initTouchTime = millis();
    TouchLatch = false;    
    buttonReleased = true;      
  }
}

/*////////////// Full Trip Reset ///////////////// */

void reset_trip() {  //Overall trip reset. Automatic if the car has been recharged to the same level as previous charge or if left button is pressed for more then 3 secondes

  Serial.println("saving");
  InitOdo = Odometer;
  InitCED = CED;  //initiate to current CED for initial CED value and
  InitSoC = SoC;  //initiate to current CED for initial SoC value and
  InitCEC = CEC;  //initiate to current CEC for initial CEC value and
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
  EEPROM.writeFloat(24, InitCDC);  //save initial Calculated CED to Flash memory
  EEPROM.writeFloat(28, InitCCC);  //save initial Calculated CED to Flash memory
  EEPROM.commit();
  Serial.println("Values saved to EEPROM");
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

void ResetCurrTrip() {  // when the car is turned On, current trip values are resetted.

  if (
    ResetOn && (SoC > 1) && (Odometer > 1) && (CED > 1) && data_ready) {  // ResetOn condition might be enough, might need to update code...
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
    if ((degrad_ratio > 1.2) || (degrad_ratio < 0.8)) {  // if a bad value got saved previously, initial ratio to 1
      degrad_ratio = 1;
    }
  }
}

/*//////Function to disable the VESS //////////*/

void setVessOff(char selector) {
  if (selector == 'D' && SelectOn) {
    //digitalWrite(LedPin, HIGH);
    digitalWrite(VESSoff, HIGH);
    StartMillis = millis();
    SelectOn = false;
  }
  if ((millis() - StartMillis) >= ToggleDelay) {
    //digitalWrite(LedPin, LOW);
    digitalWrite(VESSoff, LOW);
  }
  if (selector == 'P') {
    SelectOn = true;
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
  if (selector == 'P' && DriveOn && SoC > 0) {  // when the selector is set to Park, some values are saved to be used the next time the car is started
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
  if (DriveOn && (SoC > 0)) {
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
  
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 - 16);
  tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);
  WiFi.disconnect();
  Serial.println("Wifi Stopped");
  delay(1500);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("OBD2", tft.width() / 2, tft.height() / 2 - 16);
  tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);
  ELM_PORT.end();  
  Serial.println("OBD2 Stopped");    
  delay(1500);
  Serial.println("Going to Sleep");
  esp_deep_sleep_start();  
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
    tft.drawString(title1, tft.width() / 4, textLvl1, 1);
    tft.drawString(title2, tft.width() / 4, textLvl2, 1);
    tft.drawString(title3, tft.width() / 4, textLvl3, 1);
    tft.drawString(title4, tft.width() / 4, textLvl4, 1);
    tft.drawString(title5, tft.width() / 4, textLvl5, 1);
    tft.drawString(title6, 3 * (tft.width() / 4), textLvl1, 1);
    tft.drawString(title7, 3 * (tft.width() / 4), textLvl2, 1);
    tft.drawString(title8, 3 * (tft.width() / 4), textLvl3, 1);
    tft.drawString(title9, 3 * (tft.width() / 4), textLvl4, 1);
    tft.drawString(title10, 3 * (tft.width() / 4), textLvl5, 1);

    tft.drawLine(1,textLvl1 - 10,319,textLvl1 - 10,TFT_DARKGREY);
    tft.drawLine(tft.width() / 2,textLvl1 - 10,tft.width() / 2,drawLvl5 + 24,TFT_DARKGREY);
    tft.drawLine(1,drawLvl5 + 24,319,drawLvl5 + 24,TFT_DARKGREY);

    strcpy(prev_value1, "");
    strcpy(prev_value2, "");
    strcpy(prev_value3, "");
    strcpy(prev_value4, "");
    strcpy(prev_value5, "");
    strcpy(prev_value6, "");
    strcpy(prev_value7, "");
    strcpy(prev_value8, "");
    strcpy(prev_value9, "");
    strcpy(prev_value10, "");
    
    switch (screenNbr) {  // button select to display
      
      case 0: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAgreen);      
        drawRoundedRect(btnBred);
        drawRoundedRect(btnCred);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        strcpy(MainTitle, "Consommation");        
        tft.drawString(MainTitle, tft.width() / 2, 22, 1); 
        break; 
         
      case 1: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAred);      
        drawRoundedRect(btnBgreen);
        drawRoundedRect(btnCred);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        strcpy(MainTitle, "Batt. Info");        
        tft.drawString(MainTitle, tft.width() / 2, 22, 1); 
        break;
        
      case 2: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAred);      
        drawRoundedRect(btnBred);
        drawRoundedRect(btnCgreen);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        strcpy(MainTitle, "Puissance");        
        tft.drawString(MainTitle, tft.width() / 2, 22, 1); 
        break;

      case 3: 
        tft.setFreeFont(&FreeSans9pt7b);        
        drawRoundedRect(btnAred);      
        drawRoundedRect(btnBred);
        drawRoundedRect(btnCred);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        strcpy(MainTitle, "Setup");        
        tft.drawString(MainTitle, tft.width() / 2, 22, 1); 
        break;
    }
    DrawBackground = false;    
  }
  tft.setFreeFont(&FreeSans18pt7b);

  if (value1_float < 0) {
    value1_float = abs(value1_float);
    negative_flag1 = true;
  } else {
    negative_flag1 = false;
  }
  if (value2_float < 0) {
    value2_float = abs(value2_float);
    negative_flag2 = true;
  } else {
    negative_flag2 = false;
  }
  if (value3_float < 0) {
    value3_float = abs(value3_float);
    negative_flag3 = true;
  } else {
    negative_flag3 = false;
  }
  if (value4_float < 0) {
    value4_float = abs(value4_float);
    negative_flag4 = true;
  } else {
    negative_flag4 = false;
  }
  if (value5_float < 0) {
    value5_float = abs(value5_float);
    negative_flag5 = true;
  } else {
    negative_flag5 = false;
  }
  if (value6_float < 0) {
    value6_float = abs(value6_float);
    negative_flag6 = true;
  } else {
    negative_flag6 = false;
  }
  if (value7_float < 0) {
    value7_float = abs(value7_float);
    negative_flag7 = true;
  } else {
    negative_flag7 = false;
  }
  if (value8_float < 0) {
    value8_float = abs(value8_float);
    negative_flag8 = true;
  } else {
    negative_flag8 = false;
  }
  if (value9_float < 0) {
    value9_float = abs(value9_float);
    negative_flag9 = true;
  } else {
    negative_flag9 = false;
  }
  if (value10_float < 0) {
    value10_float = abs(value10_float);
    negative_flag10 = true;
  } else {
    negative_flag10 = false;
  }

  dtostrf(value1_float, 3, nbr_decimal1, value1);
  dtostrf(value2_float, 3, nbr_decimal2, value2);
  dtostrf(value3_float, 3, nbr_decimal3, value3);
  dtostrf(value4_float, 3, nbr_decimal4, value4);
  dtostrf(value5_float, 3, nbr_decimal5, value5);
  dtostrf(value6_float, 3, nbr_decimal6, value6);
  dtostrf(value7_float, 3, nbr_decimal7, value7);
  dtostrf(value8_float, 3, nbr_decimal8, value8);
  dtostrf(value9_float, 3, nbr_decimal9, value9);
  dtostrf(value10_float, 3, nbr_decimal10, value10);

  if (value1 != prev_value1) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value1, tft.width() / 4, drawLvl1, 1);
    if (negative_flag1) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value1, tft.width() / 4, drawLvl1, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value1, tft.width() / 4, drawLvl1, 1);
    }
    strcpy(prev_value1, value1);
  }
  if (value2 != prev_value2) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value2, tft.width() / 4, drawLvl2, 1);
    if (negative_flag2) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value2, tft.width() / 4, drawLvl2, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value2, tft.width() / 4, drawLvl2, 1);
    }
    strcpy(prev_value2, value2);
  }
  if (value3 != prev_value3) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value3, tft.width() / 4, drawLvl3, 1);
    if (negative_flag3) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value3, tft.width() / 4, drawLvl3, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value3, tft.width() / 4, drawLvl3, 1);
    }
    strcpy(prev_value3, value3);
  }
  if (value4 != prev_value4) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value4, tft.width() / 4, drawLvl4, 1);
    if (negative_flag4) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value4, tft.width() / 4, drawLvl4, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value4, tft.width() / 4, drawLvl4, 1);
    }
    strcpy(prev_value4, value4);
  }
  if (value5 != prev_value5) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value5, tft.width() / 4, drawLvl5, 1);
    if (negative_flag5) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value5, tft.width() / 4, drawLvl5, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value5, tft.width() / 4, drawLvl5, 1);
    }
    strcpy(prev_value5, value5);
  }
  if (value6 != prev_value6) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value6, 3 * (tft.width() / 4), drawLvl1, 1);
    if (negative_flag6) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value6, 3 * (tft.width() / 4), drawLvl1, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value6, 3 * (tft.width() / 4), drawLvl1, 1);
    }
    strcpy(prev_value6, value6);
  }
  if (value7 != prev_value7) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value7, 3 * (tft.width() / 4), drawLvl2, 1);
    if (negative_flag7) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value7, 3 * (tft.width() / 4), drawLvl2, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value7, 3 * (tft.width() / 4), drawLvl2, 1);
    }
    strcpy(prev_value7, value7);
  }
  if (value8 != prev_value8) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value8, 3 * (tft.width() / 4), drawLvl3, 1);
    if (negative_flag8) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value8, 3 * (tft.width() / 4), drawLvl3, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value8, 3 * (tft.width() / 4), drawLvl3, 1);
    }
    strcpy(prev_value8, value8);
  }
  if (value9 != prev_value9) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value9, 3 * (tft.width() / 4), drawLvl4, 1);
    if (negative_flag9) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value9, 3 * (tft.width() / 4), drawLvl4, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value9, 3 * (tft.width() / 4), drawLvl4, 1);
    }
    strcpy(prev_value9, value9);
  }
  if (value10 != prev_value10) {
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(prev_value10, 3 * (tft.width() / 4), drawLvl5, 1);
    if (negative_flag10) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(value10, 3 * (tft.width() / 4), drawLvl5, 1);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(value10, 3 * (tft.width() / 4), drawLvl5, 1);
    }
    strcpy(prev_value10, value10);
  }
}
//-------------------------------------------------------------------------------------
//             Start of Pages content definition
//-------------------------------------------------------------------------------------

/*///////////////// Display Page 1 //////////////////////*/
void page1() {

  strcpy(title1,"SoC");
  strcpy(title2,"PIDkWh_100");
  strcpy(title3,"kWh/100km");
  strcpy(title4,"10Km_kWh");
  strcpy(title5,"EstLeft_kWh");
  strcpy(title6,"TripOdo");  
  strcpy(title7,"Est_range");
  strcpy(title8,"Est_range");  
  strcpy(title9,"Est_range");
  strcpy(title10,"used_kwh");
  value1_float = SoC;  
  value2_float = PIDkWh_100;
  value3_float = kWh_100km;
  value4_float = span_kWh_100km;
  value5_float = EstLeft_kWh;  
  value6_float = TripOdo;
  value7_float = Est_range3;
  value8_float = Est_range;
  value9_float = Est_range2;
  value10_float = used_kwh;
  
  if (value1_float >= 100) {
    nbr_decimal1 = 0;
  } else {
    nbr_decimal1 = 1;
  }
  if (value2_float >= 100) {
    nbr_decimal2 = 0;
  } else {
    nbr_decimal2 = 1;
  }
  if (value3_float >= 100) {
    nbr_decimal3 = 0;
  } else {
    nbr_decimal3 = 1;
  }
  if (value4_float >= 100) {
    nbr_decimal4 = 0;
  } else {
    nbr_decimal4 = 1;
  }
  if (value5_float >= 100) {
    nbr_decimal5 = 0;
  } else {
    nbr_decimal5 = 1;
  }
  if (value6_float >= 100) {
    nbr_decimal6 = 0;
  } else {
    nbr_decimal6 = 1;
  }
  if (value7_float >= 100) {
    nbr_decimal7 = 0;
  } else {
    nbr_decimal7 = 1;
  }
  if (value8_float >= 100) {
    nbr_decimal8 = 0;
  } else {
    nbr_decimal8 = 1;
  }
  if (value9_float >= 100) {
    nbr_decimal9 = 0;
  } else {
    nbr_decimal9 = 1;
  }
  if (value10_float >= 100) {
    nbr_decimal10 = 0;
  } else {
    nbr_decimal10 = 1;
  }

  DisplayPage();
}
/*///////////////// End of Display Page 1 //////////////////////*/

/*///////////////// Display Page 2 //////////////////////*/
void page2() {

  strcpy(title1, "SoC");
  strcpy(title2, "MIN_Temp");
  strcpy(title3, "MAXcellv");
  strcpy(title4, "SOH");
  strcpy(title5, "Full_Ah");
  strcpy(title6, "BmsSoC");
  strcpy(title7, "MAX_Temp");
  strcpy(title8, "CellVdiff");
  strcpy(title9, "Det_Total");
  strcpy(title10, "12V_SoC");
  value1_float = SoC;
  value2_float = BattMinT;
  value3_float = MAXcellv;
  value4_float = SOH;
  value5_float = EstFull_Ah;
  value6_float = BmsSoC;
  value7_float = BattMaxT;
  value8_float = CellVdiff;
  value9_float = Deter_Min;
  value10_float = AuxBattSoC;
  if (value1_float >= 100) {
    nbr_decimal1 = 0;
  } else {
    nbr_decimal1 = 1;
  }
  if (value2_float >= 100) {
    nbr_decimal2 = 0;
  } else {
    nbr_decimal2 = 1;
  }
  if (value3_float >= 100) {
    nbr_decimal3 = 0;
  } else {
    nbr_decimal3 = 1;
  }
  if (value4_float >= 100) {
    nbr_decimal4 = 0;
  } else {
    nbr_decimal4 = 1;
  }
  if (value5_float >= 100) {
    nbr_decimal5 = 0;
  } else {
    nbr_decimal5 = 1;
  }
  if (value6_float >= 100) {
    nbr_decimal6 = 0;
  } else {
    nbr_decimal6 = 1;
  }
  if (value7_float >= 100) {
    nbr_decimal7 = 0;
  } else {
    nbr_decimal7 = 1;
  }
  if (value8_float >= 100) {
    nbr_decimal8 = 0;
  } else {
    nbr_decimal8 = 2;
  }
  if (value9_float >= 100) {
    nbr_decimal9 = 0;
  } else {
    nbr_decimal9 = 1;
  }
  if (value10_float >= 100) {
    nbr_decimal10 = 0;
  } else {
    nbr_decimal10 = 1;
  }

  DisplayPage();
}
/*///////////////// End of Display Page 2 //////////////////////*/

/*///////////////// Display Page 3 //////////////////////*/
void page3() {

  strcpy(title1, "Power");
  strcpy(title2, "MIN_Temp");
  strcpy(title3, "PID_kWh");
  strcpy(title4, "Calc_Used");
  strcpy(title5, "SoC");
  strcpy(title6, "Max_Pwr");
  strcpy(title7, "MAX_Temp");
  strcpy(title8, "Int_Energ");
  strcpy(title9, "Calc_Left");
  strcpy(title10, "Heater");  
  value1_float = Power;
  value2_float = BattMinT;
  value3_float = Net_kWh;
  value4_float = used_kwh;
  value5_float = SoC;
  value6_float = Max_Pwr;
  value7_float = BattMaxT;
  value8_float = acc_energy;
  value9_float = left_kwh;
  value10_float = Heater;
  
  if (value1_float >= 100) {
    nbr_decimal1 = 0;
  } else {
    nbr_decimal1 = 1;
  }
  if (value2_float >= 100) {
    nbr_decimal2 = 0;
  } else {
    nbr_decimal2 = 1;
  }
  if (value3_float >= 100) {
    nbr_decimal3 = 0;
  } else {
    nbr_decimal3 = 1;
  }
  if (value4_float >= 100) {
    nbr_decimal4 = 0;
  } else {
    nbr_decimal4 = 1;
  }
  if (value5_float >= 100) {
    nbr_decimal5 = 0;
  } else {
    nbr_decimal5 = 1;
  }
  if (value6_float >= 100) {
    nbr_decimal6 = 0;
  } else {
    nbr_decimal6 = 1;
  }
  if (value7_float >= 100) {
    nbr_decimal7 = 0;
  } else {
    nbr_decimal7 = 1;
  }
  if (value8_float >= 100) {
    nbr_decimal8 = 0;
  } else {
    nbr_decimal8 = 1;
  }
  if (value9_float >= 100) {
    nbr_decimal9 = 0;
  } else {
    nbr_decimal9 = 1;
  }
  if (value10_float >= 100) {
    nbr_decimal10 = 0;
  } else {
    nbr_decimal10 = 1;
  }

  DisplayPage();
}
/*///////////////// End of Display Page 3 //////////////////////*/

/*///////////////// Display Page 4 //////////////////////*/
void page4() {

  strcpy(title1, "MIN_Temp");
  strcpy(title2, "Heater");
  strcpy(title3, "Power");
  strcpy(title4, "MAX_Temp");
  value1_float = BattMinT;
  value2_float = Heater;
  value3_float = Power;
  value4_float = BattMaxT;
  nbr_decimal1 = 1;
  nbr_decimal2 = 1;
  if (value1_float >= 100) {
    nbr_decimal1 = 0;
  } else {
    nbr_decimal1 = 1;
  }
  if (value2_float >= 100) {
    nbr_decimal2 = 0;
  } else {
    nbr_decimal2 = 1;
  }
  if (value3_float >= 100) {
    nbr_decimal3 = 0;
  } else {
    nbr_decimal3 = 1;
  }
  if (value4_float >= 100) {
    nbr_decimal4 = 0;
  } else {
    nbr_decimal4 = 1;
  }

  DisplayPage();
}
/*///////////////// End of Display Page 4 //////////////////////*/

/*///////////////////////////////////////////////////////////////////////*/
/*                     START OF LOOP                                     */
/*///////////////////////////////////////////////////////////////////////*/

void loop() {

  loop_count += 1;
  //ButtonLoop();

  currentTimer = millis();
  if (currentTimer - previousTimer >= sendInterval) {    
    send_data = true;  // This will trigger logic to send data to Google sheet
    previousTimer = currentTimer;
    if (!send_enabled) {
      sendIntervalOn = true;
    }
  }

  if (datasent){
    tft.fillCircle(20, 20, 6,TFT_GREEN);
    datasent = false;
  }
  else{
    tft.fillCircle(20, 20, 6,TFT_BLACK);
  }


  /*/////// Read each OBDII PIDs /////////////////*/

  button();
    
  read_data();

  /*/////// Display Page Number /////////////////*/

  if (!SetupOn && (ESP_on || (Power < 0))) {

    if (WiFi.status() == WL_CONNECTED){
        tft.fillCircle(300, 20, 6,TFT_GREEN);
      }
    else{
      tft.fillCircle(300, 20, 6,TFT_RED);
    }
        
    switch (screenNbr) {  // select page to display
      case 0: page1(); break;
      case 1: page2(); break;
      case 2: page3(); break;
      case 3: page4(); break;    
    }
  }
  /*/////// Display Setup Page/////////////////*/
  else {
    page1();
  }

  /*/////// Stop ESP /////////////////*/
  if (!BMS_ign && ESP_on && !StayOn && (SpdSelect == 'P')) {  // When car is power off, call stop_esp which saves some data before powering ESP32 down
    record_code = 5;
    shutdown_esp = true;
    if (!SoC_saved) {
      mem_SoC = SoC;
      SoC_saved = true;
    }
    if (code_sent) {
      Serial.println("Code sent and Normal shutdown");
      stop_esp();
    } else if (!send_enabled) {
      Serial.println("No Code sent and Normal shutdown");
      stop_esp();
    }

  }

  else if (!BMS_ign && (Power >= 0) && !StayOn && data_ready) {  // When the car is off but the BMS does some maintnance check, wait 20 mins before esp32 power down
    if (!SoC_saved) {
      ESPinitTimer = millis();
      mem_SoC = SoC;
      SoC_saved = true;
    }
    ESPTimer = millis();
    shutdown_timer = (ESPTimer - ESPinitTimer) / 1000;
    sd_condition1 = shutdown_timer >= ESPTimerInterval;
    sd_condition2 = ((AuxBattSoC > 0) && (AuxBattSoC < 75));
    if (sd_condition1 || sd_condition2) {

      if (sd_condition1) {
        record_code = 6;
        shutdown_esp = true;
      } else if (sd_condition2) {
        record_code = 7;
        shutdown_esp = true;
      }
      if (code_sent) {
        Serial.println("Code sent and Timer shutdown");
        stop_esp();
      } else if (!send_enabled) {
        Serial.println("No Code sent and Timer shutdown");
        stop_esp();
      }
    }
  }

  if ((WiFi.status() != WL_CONNECTED) && BMS_ign && !wifiReconn && StartWifi) {  // If esp32 is On when start the car, reconnect wifi if not connected    
    tft.setFreeFont(&FreeSans9pt7b);
    ConnectWifi(tft);
    wifiReconn = true;
    if (WiFi.status() == WL_CONNECTED) {
      send_enabled = true;
      send_data = true;
    }
    DrawBackground = true;
  }

  ResetCurrTrip();

  //setVessOff(SpdSelect);  //This will momentarely set an output ON to turned off the VESS //
}
