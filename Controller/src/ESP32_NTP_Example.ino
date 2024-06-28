////////////////////////// CREATED BY NORBERT LIPOWICZ ///////////////////////////////

#include "SunPos.h"
#include <Wire.h> 
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <Timers.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <Preferences.h>


#define PotElevation 35
#define PotAzimuth 34
#define ButtonLeft 19
#define ButtonRight 18
#define ButtonMenuLeft 32
#define ButtonMenuRight 33
#define SwitchAuto 14
#define SwitchManual 27
#define OpenAzimuth 13
#define CloseAzimuth 12
#define OpenElevation 25
#define CloseElevation 26
#define RXD2 16
#define TXD2 17

#define LCDsunrise 0
#define LCDsunset 1
#define LCDdegree 2
#define LCDchoose 3
#define LCDcursor 4
#define LCDnext 5
#define LCDnow 6
#define LCDprev 7

#define EEPROMsize 50
#define EEPROMaziRes180 16 //4
#define EEPROMaziRes225 20 //4
#define EEPROMelevRes45 24 //4
#define EEPROMelevRes90 28 //4
#define EEPROMminAziResistance 32 //4
#define EEPROMmaxAziResistance 36 //4
#define EEPROMminElevResistance 40 //4
#define EEPROMUTC 44 //1
#define EEPROMdowntimeMin 45 //1
#define EEPROMidController 46 //1

#define ANALOGpinRange 4095

LiquidCrystal_I2C lcd(0x3F,20,4); // 0x3F 0x27 
RTC_DS1307 rtc;
Timer timer;

enum WifiStatus 
{
  WaitToSetup = 0,
  Connected,
  LostConnect
};
//############################# VARIABLES ##############################
//
//----------------------- User-defined variables ----------------------|
double nLongitude = 17.060841, nLatitude = 51.109219;
int UTC = 1;
double downtimeMin = 8;
int minAziResistance = 0, maxAziResistance = 4095, minElevResistance = 0;
int elevRes90 = 3900, elevRes45 = 1000, aziRes180 = 2000, aziRes225 = 3700;
//
//------------------------- Variables from RTC ------------------------|
int nDay, nMonth, nYear;
double nHour, nMinute, nSecond;
//
//------------------------ Sun position variables ---------------------|
double nZenithAngle, nAzimuthAngle, nElevationAngle;
//
//-------------------------- Internal variables -----------------------|
const int numberOfSamples = 1000, buttonDelay = 20, tableSize = 1000, nrOfMenuPositions = 12;
int actualMenuPosition = 1;
int intLongitude[8], intLatitude[8];
int valPotElevation, valPotAzimuth;
int sunPosPotValChoose;
int backCounter = 0, timeToBack = 20;
int checkDay;
double sunriseHour, sunriseMin, sunsetHour, sunsetMin;
double switchOnTable[tableSize]; //{H, MIN, POT}
double valPotAziDeg, valPotElevDeg;
double oneDegElev, oneDegAzi;
double elevMinDeg = 0, elevMaxDeg = 90, aziMinDeg = 0, aziMaxDeg = 180;
double accuracy = 2.5;
double aziDegWork, elevDegWork;
bool menuState = true;
bool buttonTimerCheckLeft = false, buttonTimerCheckRight = false;
bool work = false;
bool manualMode = false;
bool manualModeUpdate = false;
bool autoMode = false;
float setElevationManual = 90, setAzimuthManual = 180;
IPAddress IP;
WifiStatus wifiStatus = WifiStatus::WaitToSetup;
WifiStatus newWifiStatus = WifiStatus::WaitToSetup;
bool dataStatus = false;
bool dataStatusShow = false;
bool dataUpdated = false;
bool updated_by_app = false;

SemaphoreHandle_t mutex;
//
//-------------------------- Wifi variables -------------------------|
const char* PARAM_INPUT_2 = "server_ip";
const char* PARAM_INPUT_3 = "wifi_ssid";
const char* PARAM_INPUT_4 = "wifi_password";
//
const int intervalWifi = 15000; //ms
Timer timerWifi;
//
String serverIp = "unaffiliated-rumble.000webhostapp.com"; // IP-адрес сервера
String wifiSSID = "AndroidAPE261";
String wifiPassword = "11111111";
//
WebServer server(80);
HTTPClient http;
//
const char* ap_ssid = "Solar_Tracker";
const char* ap_password = "11111111";

bool wifiReady = false;


Preferences preferences;

const char* PARAM_SSID = "ssid";
const char* PARAM_PASSWORD = "password";
const char* PARAM_SERVER = "server";

int idController = 1;
int maxIdsController = 2;
//
//-------------------- Charge Controller variables -------------------|
ModbusMaster node;
float batteryCapacity = 0.0;
float batteryVoltage = 0.0;
float batteryChargeCurrent = 0.0;
float solarPanelVoltage = 0.0;
float solarPanelPower = 0.0;
const int intervalModBus = 2000; //ms
//########################### END VARIABLES ############################




//############################# LCD CHARS ##############################
byte sunriseChar[8] = {
  0b00000,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b11111,
  0b11111,
  0b00000
};
//
byte sunsetChar[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b10101,
  0b01110,
  0b11111,
  0b11111,
  0b00000
};

byte degreeChar[8] = {
  0b01100,
  0b10010,
  0b10010,
  0b01100,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte chooseChar[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte cursorChar[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte nextChar[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};

byte nowChar[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b01110,
  0b01110,
  0b00100,
  0b00000,
  0b00000
};

byte prevChar[8] = {
  0b00000,
  0b00100,
  0b00100,
  0b00100,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

//########################### END LCD CHARS ############################




//############################# FUNCTIONS ##############################
void readModbusData() {

  uint8_t result;
  result = node.readHoldingRegisters(0x100, 1);
  if (result == node.ku8MBSuccess) {
    batteryCapacity = node.getResponseBuffer(0x00);
  } else {
    batteryCapacity = 0;
  }
  result = node.readHoldingRegisters(0x101, 1);
  if (result == node.ku8MBSuccess) {
    batteryVoltage = node.getResponseBuffer(0x00) / 10.0;
  } else {
    batteryVoltage = 0;
  }
  result = node.readHoldingRegisters(0x102, 1);
  if (result == node.ku8MBSuccess) {
    batteryChargeCurrent = node.getResponseBuffer(0x00) / 100.0;
  } else {
    batteryChargeCurrent = 0;
  }
  result = node.readHoldingRegisters(0x107, 1);
  if (result == node.ku8MBSuccess) {
    solarPanelVoltage = node.getResponseBuffer(0x00) / 10.0;
  } else {
    solarPanelVoltage = 0;
  }
  result = node.readHoldingRegisters(0x109, 1);
  if (result == node.ku8MBSuccess) {
    solarPanelPower = node.getResponseBuffer(0x00);
  } else {
    solarPanelPower = 0;
  }
}

void initModbus()
{
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(255, Serial2); // 255 is the Modbus slave ID
}

void checkPosForCursorForInfoFromChargeController(float val, int row)
{
    lcd.setCursor(13, row);
    if(val < 10)
      lcd.print("  ");
    else
    {
      if(val < 100)
        lcd.print(" ");
    }
}

void updateDisplayInfoFromChargeController() {
    // Battery Capacity
    checkPosForCursorForInfoFromChargeController(batteryCapacity, 0);
    lcd.print(batteryCapacity);
    lcd.print("%");

    // Battery Voltage
    checkPosForCursorForInfoFromChargeController(batteryVoltage, 1);
    lcd.print(batteryVoltage);
    lcd.print("V");

    // Battery Charge Current
    checkPosForCursorForInfoFromChargeController(batteryChargeCurrent, 2);
    lcd.print(batteryChargeCurrent);
    lcd.print("A");

    // Solar Panel Voltage
    checkPosForCursorForInfoFromChargeController(solarPanelVoltage, 3);
    lcd.print(solarPanelVoltage);
    lcd.print("V");
}

//---------------------- Calculate Sun Position -----------------
void sunPosCalc(double dHour, double dMinute)
{
  struct cTime nTime;
  struct cLocation nLocation;
  struct cSunCoordinates nSunCoordinates;
//  nHour = nHour - UTC;
  
  nTime.iYear = nYear;
  nTime.iMonth = nMonth;
  nTime.iDay = nDay;
  nTime.dHours = dHour;
  nTime.dMinutes = dMinute;
  nTime.dSeconds = nSecond;
  
  nLocation.dLongitude = nLongitude;
  nLocation.dLatitude = nLatitude;

  sunpos(nTime, nLocation, &nSunCoordinates);
  
  nZenithAngle = nSunCoordinates.dZenithAngle;
  nElevationAngle = 90 - nZenithAngle;
  nAzimuthAngle = nSunCoordinates.dAzimuth;
}
//
//-------------- Average from "n" samples for analog "pin" --------------
int analogValAverage(int n, int pin)
{
  int sum = 0;
  float average;
  for(int i=1; i<=n; i++)
  {
    sum += analogRead(pin);
  }
  average = (float)(sum/n);
  average = round(average);
  average = (int)average;
  return average;
}
//
//--------------------- Calculate Sunrise Time ----------------------
void sunriseTimeCalc()
{
  double h, m;
  for(h = 0; h <= 23; h++)
  {
    for(m = 0; m <= 59; m++)
    {
      sunPosCalc(h, m);
      sunriseHour = h;
      sunriseMin = m;
      
      if(abs(nElevationAngle) < 1 )
      {
        if(nElevationAngle >= 0)
          break;
      }
    }
    if(nElevationAngle >= 0)
      break;
  }
  sunriseHour += UTC;
}
//
//------------------------ Calculate Sunset Time ------------------
void sunsetTimeCalc()
{
  double h, m;
  for(h = 23; h >= 0; h--)
  {
    for(m = 59; m >= 0; m--)
    {
      sunPosCalc(h, m);
      sunsetHour = h; 
      sunsetMin = m;
      //Serial.print(h,0);
      //Serial.print(":");
      //Serial.print(m,0);
      //Serial.print("\t");
      //Serial.print(nAzimuthAngle);
      //Serial.print("\t");
      //Serial.println(nElevationAngle);
      
      if(abs(nElevationAngle) < 1 )
      {
        if(nElevationAngle >= 0)
          break;
      }
    }
    if(nElevationAngle >= 0)
      break;
  }
  sunsetHour += UTC;
}
//
//----------------- Calculate Switch-On Actuators Table ----------------
void switchOnTableCalc()
{
//  switchOnElevTable{H, MIN, POT}
//  switchOnAziTable
  sunriseTimeCalc();
  sunsetTimeCalc();
  double h=sunriseHour-UTC, m=sunriseMin;
  double dh, dm;
  sunPosCalc(h, m);
  
  int i;
  for(i=0; i<tableSize; i+=4)
  {
    dh = h;
    dm = m + downtimeMin/2;
    if(dm >= 60)
    {
      dm -= 60;
      dh++;
    }
    sunPosCalc(dh, dm); //Calculate forward SunPos
//    if(nElevationAngle < 0)
//    {
//      nElevationAngle = 0;
//    }
    if(nElevationAngle < elevMinDeg)
      nElevationAngle = elevMinDeg;
      
    if(nElevationAngle > elevMaxDeg)
      nElevationAngle = elevMaxDeg;
      
    if(nAzimuthAngle > aziMaxDeg)
      nAzimuthAngle = aziMaxDeg;

    if(nAzimuthAngle < aziMinDeg)
      nAzimuthAngle = aziMinDeg;
    
    switchOnTable[i] = h+UTC;
    switchOnTable[i+1] = m;
    switchOnTable[i+2] = nAzimuthAngle;
    switchOnTable[i+3] = nElevationAngle;
    
    m += downtimeMin;
    if(m >= 60)
    {
      m -= 60;
      h++;
    }
    
    if((h == sunsetHour-UTC && m >= sunsetMin) || h > sunsetHour - UTC)
    {
      break;
    }
  }
//  sunPosCalc(sunsetHour-UTC, sunsetMin);
//  
//  switchOnTable[i] = sunsetHour;
//  switchOnTable[i+1] = sunsetMin;
//  switchOnTable[i+2] = nElevationAngle;
//  switchOnTable[i+3] = nAzimuthAngle;
//  

  for(i+=4; i<tableSize; i++)
  {
    switchOnTable[i] = -1;
  }
}
//
//----------------- Checking which button is pressed ----------------
bool buttonLeftPressed()
{
  int n = 0;
  if(digitalRead(ButtonLeft))
  {
    delay(buttonDelay);
    while(digitalRead(ButtonLeft))
    {
      if(timerAvailable())
      {
        n++;
        if(n == 2)
          buttonTimerCheckLeft = true;
      }
      if(buttonTimerCheckLeft)
      {
        while(digitalRead(ButtonLeft))
        {
          //Serial.print("buttonLeftPressed()");
          return true;
        }
      }
    }
    delay(buttonDelay);
    backCounter = 0;
    //Serial.print("buttonLeftPressed()");
    return true;
  }
  else
  {
    buttonTimerCheckLeft = false;
    return false;
  }
}

bool buttonRightPressed()
{
  int n = 0;
  if(digitalRead(ButtonRight))
  {
    delay(buttonDelay);
    while(digitalRead(ButtonRight))
    {
      if(timerAvailable())
      {
        n++;
        if(n == 2)
          buttonTimerCheckRight = true;
      }
      if(buttonTimerCheckRight)
      {
        while(digitalRead(ButtonRight))
        {
        //Serial.print("buttonRightPressed()");
          return true;
        }
      }
    }
    delay(buttonDelay);
    backCounter = 0;
    //Serial.print("buttonRightPressed()");
    return true;
  }
  else
  {
    buttonTimerCheckRight = false;
    return false;
  }
}

bool buttonMenuRightPressed()
{
  if(digitalRead(ButtonMenuRight))
  {
    delay(buttonDelay);
    while(digitalRead(ButtonMenuRight));
    delay(buttonDelay);
    backCounter = 0;
    //Serial.print("buttonMenuRightPressed()");
    return true;
  }
  else
    return false;
}

bool buttonMenuLeftPressed()
{
  if(digitalRead(ButtonMenuLeft))
  {
    delay(buttonDelay);
    while(digitalRead(ButtonMenuLeft));
    delay(buttonDelay);
    backCounter = 0;
    //Serial.println("buttonMenuLeftPressed()");
    return true;
  }
  else
    return false;
}

bool switchAutoOn()
{
  if(digitalRead(SwitchAuto))
  {
    delay(buttonDelay);
    return true;
  }
  else
    return false;
}

bool switchManualOn()
{
  if(digitalRead(SwitchManual))
  {
    delay(buttonDelay);
    return true;
  }
  else
    return false;
}
//
//----------------- Print two digits when number is < 10 ----------------
void DLCDprint2digits(double number)
{
  if (number >= 0 && number <10)
    lcd.write('0');
  lcd.print(number,0);
}

void LCDprint2digits(int number)
{
  if (number >= 0 && number <10)
    lcd.write('0');
  lcd.print(number);
}
//
//-------------- Print space when number is in specific range ----------------
void LCDprintSpace(double number, int comma)
{
  if (number >= 100 && number < 1000)
  {
    lcd.print(number, comma);
  }
  if (number >= 10 && number < 100)
  {
    lcd.print(" ");
    lcd.print(number, comma);
  }
  else if (number >= 0 && number < 10)
  {
    lcd.print("  ");
    lcd.print(number, comma);
  }
  else if (number > -10 && number < 0)
  {
    lcd.print(" ");
    lcd.print(number, comma);
  }
  else if (number > -100 && number <= -10)
  {
    lcd.print(number, comma);
  }
}
//
//------------ Print space when int number is in specific range ----------------
void LCDprintSpaceInt(int number)
{
  if (number >= 1000 && number < 10000)
  {
    lcd.print(number);
  }
  if (number >= 100 && number < 1000)
  {
    lcd.print(" ");
    lcd.print(number);
  }
  if (number >= 10 && number < 100)
  {
    lcd.print("  ");
    lcd.print(number);
  }
  else if (number >= 0 && number < 10)
  {
    lcd.print("   ");
    lcd.print(number);
  }
  else if (number > -10 && number < 0)
  {
    lcd.print("  ");
    lcd.print(number);
  }
  else if (number > -100 && number <= -10)
  {
    lcd.print(" ");
    lcd.print(number);
  }
}
//----------------------- Print first page of menu ----------------------
void printMenuFirstPage()
{
  

  lcd.setCursor(0,0);
  lcd.print("      ");
  DLCDprint2digits(nHour);
  lcd.print(":");
  DLCDprint2digits(nMinute);
  lcd.print(":");
  DLCDprint2digits(nSecond);
  lcd.print("     ");

  lcd.setCursor(0,1);
  lcd.print("     ");
  LCDprint2digits(nDay);
  lcd.print("/");
  LCDprint2digits(nMonth);
  lcd.print("/");
  lcd.print(nYear);
  lcd.print("    ");
  
  lcd.setCursor(0,2);
  lcd.write(LCDsunrise);
  lcd.print(" ");
  DLCDprint2digits(sunriseHour);
  lcd.print(":");
  DLCDprint2digits(sunriseMin);
  lcd.setCursor(0,3);
  lcd.write(LCDsunset);
  lcd.print(" ");
  DLCDprint2digits(sunsetHour);
  lcd.print(":");
  DLCDprint2digits(sunsetMin);

  sunPosCalc(nHour-UTC, nMinute);
  lcd.setCursor(10,2);
  lcd.print("SA ");
  LCDprintSpace(nAzimuthAngle,2);
  lcd.write(LCDdegree);
  lcd.setCursor(10,3);
  lcd.print("SE ");
  LCDprintSpace(nElevationAngle,2);
  lcd.write(LCDdegree);
}
//
//----------------------------- Update time ----------------------
void updateTime()
{
  DateTime now = rtc.now();

  nHour = now.hour();
  nMinute = now.minute();
  nSecond = now.second();
  nDay = now.day();
  nMonth = now.month();
  nYear = now.year();
}
//
//------------------------ Case Initial Conditions ----------------------
void caseInitialConditions()
{
  lcd.clear();
  menuState = true;
  backCounter = 0;
}
//
//------------------------------ Menu Change ----------------------
int menuChange(int actualPos, int nrOfPos, bool isLeftExit)
{
  if(buttonMenuRightPressed())
  {
    actualPos++;
    if(actualPos > nrOfPos)
      actualPos = 1;
    menuState = !menuState;
    return actualPos;
  }
  if(!isLeftExit)
  {
    if(buttonMenuLeftPressed())
    {
      actualPos--;
      if(actualPos < 1)
      actualPos = nrOfPos;
      menuState = !menuState;
      return actualPos;
    }
  }
  return actualPos;
}
//
//--------------------------- Actuators Control ----------------------
void openAzimuth()
{
  if(digitalRead(CloseAzimuth) == LOW)
  {
    digitalWrite(CloseAzimuth, HIGH);
    delay(200);
  }
  digitalWrite(OpenAzimuth, LOW);
}

void closeAzimuth()
{
  if(digitalRead(OpenAzimuth) == LOW)
  {
    digitalWrite(OpenAzimuth, HIGH);
    delay(200);
  }
  digitalWrite(CloseAzimuth, LOW);
}

void openElevation()
{
  if(digitalRead(CloseElevation) == LOW)
  {
    digitalWrite(CloseElevation, HIGH);
    delay(200);
  }
  digitalWrite(OpenElevation, LOW);
}

void closeElevation()
{
  if(digitalRead(OpenElevation) == LOW)
  {
    digitalWrite(OpenElevation, HIGH);
    delay(200);
  }
  digitalWrite(CloseElevation, LOW);
}

void stopAzimuth()
{
  digitalWrite(OpenAzimuth, HIGH);
  digitalWrite(CloseAzimuth, HIGH);
}

void stopElevation()
{
  digitalWrite(OpenElevation, HIGH);
  digitalWrite(CloseElevation, HIGH);
}
//
//------------------------ Manual Actuators Control ----------------------
void manualActuatorsControl()
{
  if(buttonRightPressed())
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Azimuth");
    lcd.setCursor(11,0);
    lcd.print("Elevation");
    
    int pos = 1;
    bool stateA = false;
    bool stateE = false;
    while(!buttonMenuLeftPressed())
    {
      readPotValues();
      resistanceToDegrees(valPotAzimuth, valPotElevation);
      lcd.setCursor(0,1);
      LCDprintSpace(valPotAziDeg,2);
      lcd.write(LCDdegree);
      lcd.setCursor(13,1);
      LCDprintSpace(valPotElevDeg,2);
      lcd.write(LCDdegree);
      
      switch(pos)
      {
        case 1:
        {
          lcd.setCursor(10,0);
          lcd.print(" ");
          lcd.setCursor(7,0);
          lcd.write(LCDchoose);   
          
          if(digitalRead(ButtonRight) && stateA == false)
          {
            while(digitalRead(ButtonRight));
            lcd.setCursor(0, 3);
            lcd.print("Opening");
            openAzimuth();
            stateA = true;
          }
          else if(digitalRead(ButtonLeft) && stateA == false)
          {
            while(digitalRead(ButtonLeft));
            lcd.setCursor(0, 3);
            lcd.print("Closing");
            closeAzimuth();
            stateA = true;
          }
          else if((digitalRead(ButtonRight) || digitalRead(ButtonLeft)) && stateA == true)
          {
            while(digitalRead(ButtonRight));
            while(digitalRead(ButtonLeft));
            stopAzimuth();
            lcd.setCursor(0, 3);
            lcd.print("       ");
            stateA = false;
          }
          pos = menuChange(pos, 2, 1);
        }
        break;

        case 2:
        {
          lcd.setCursor(7,0);
          lcd.print(" ");
          lcd.setCursor(10,0);
          lcd.write(LCDchoose);
          
          if(digitalRead(ButtonRight) && stateE == false)
          {
            while(digitalRead(ButtonRight));
            lcd.setCursor(13, 3);
            lcd.print("Opening");
            openElevation();
            stateE = true;
          }
          else if(digitalRead(ButtonLeft) && stateE == false)
          {
            while(digitalRead(ButtonLeft));
            lcd.setCursor(13, 3);
            lcd.print("Closing");
            closeElevation();
            stateE = true;
          }
          else if((digitalRead(ButtonRight) || digitalRead(ButtonLeft)) && stateE == true)
          {
            while(digitalRead(ButtonRight));
            while(digitalRead(ButtonLeft));
            stopElevation();
            lcd.setCursor(13, 3);
            lcd.print("       ");
            stateE = false;
          }
          pos = menuChange(pos, 2, 1);
        }
        break;
      }
    }
    stopElevation();
    stopAzimuth();
    menuState = false;
  }
}
//
//------------------------ EEPROM write functions ----------------------
void saveLongitude()
{
  xSemaphoreTake(mutex,portMAX_DELAY);
   int x;
   x = nLongitude*1000000;
  
  for(int i=7; i>-1; i--)
  {
    intLongitude[i] = x % 10;
    x /= 10;
  }
  for(int i=0; i<8; i++)
  {
    EEPROM.write(i, intLongitude[i]);
    EEPROM.commit();
  }
  xSemaphoreGive(mutex);
  delay(200); // Short delay is needed!
}

void saveLatitude()
{
  xSemaphoreTake(mutex,portMAX_DELAY);
   int x;
   x = nLatitude*1000000;
  
  for(int i=7; i>-1; i--)
  {
    intLatitude[i] = x % 10;
    x /= 10;
  }
  for(int i=0; i<8; i++)
  {
    EEPROM.write(i+8, intLatitude[i]);
    EEPROM.commit();
  }
  xSemaphoreGive(mutex);
  delay(200); // Short delay is needed!
}

void save4DigitNumber(int x, int EEPROMstartPos)
{
  int s=0, val = x, power;
  for(int i=EEPROMstartPos; i<EEPROMstartPos+4; i++)
  {
    power = 3-(i-EEPROMstartPos);
    x = val/(pow(10, power))-s;
    s=s*10+x*10;
    EEPROM.write(i, x);
    EEPROM.commit();
  }
}

void saveNumber0_255(int x, int EEPROMpos)
{
  EEPROM.write(EEPROMpos, x);
  EEPROM.commit();
}
//
//------------------------- EEPROM read functions ----------------------
void readLongitude()
{
  int x=0;
  
  for(int i=0; i<8; i++)
  {
    intLongitude[i] = EEPROM.read(i);
    x = x*10 + intLongitude[i];
  }
  nLongitude = x/1000000.;
}

void readLatitude()
{
  int x = 0;
  
  for(int i=0; i<8; i++)
  {
    intLatitude[i] = EEPROM.read(i+8);
    x = x*10 + intLatitude[i];
  }
  nLatitude = x/1000000.;
}

int read4DigitNumber(int EEPROMstartPos)
{
  int val=0, x;
  for(int i=EEPROMstartPos; i<EEPROMstartPos+4; i++)
  {
    x = EEPROM.read(i);
    val = val*10 + x;
  }
  return val;
}

int readNumber0_255(int EEPROMpos)
{
  int val;
  val = EEPROM.read(EEPROMpos);
  return val;
}
//
//---------------------------- Location settings ----------------------
void locationSettings()
{
  if(buttonRightPressed())
  {
    int pos1 = 1;
    int choise = 0;
    while(!buttonMenuLeftPressed())
    {

      switch(pos1)
      {
        case 1:
        {
          lcd.setCursor(9,3);
          lcd.print(" ");
          lcd.setCursor(9,2);
          lcd.write(LCDchoose);
          pos1 = menuChange(pos1, 2, 1);
          choise = 1;
        }
        break;
  
        case 2:
        {
          lcd.setCursor(9,2);
          lcd.print(" ");
          lcd.setCursor(9,3);
          lcd.write(LCDchoose);
          pos1 = menuChange(pos1, 2, 1);
          choise = 2;
        }
        break;
      }

      if(buttonRightPressed())
      {
        int pos = 1;
        while(pos != 9)
        {
          //Serial.println(pos);
          switch(pos)
          {
            
            case 1:
            {
              lcd.setCursor(11,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(11,2);
                  lcd.print(intLongitude[0]);
                  intLongitude[0] = valueSettings(intLongitude[0], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(11,3);
                  lcd.print(intLatitude[0]);
                  intLatitude[0] = valueSettings(intLatitude[0], 0, 9);
                }
                
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(11,1);
              lcd.print(" ");
            }
            break;
      
            case 2:
            {
              lcd.setCursor(12,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(12,2);
                  lcd.print(intLongitude[1]);
                  intLongitude[1] = valueSettings(intLongitude[1], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(12,3);
                  lcd.print(intLatitude[1]);
                  intLatitude[1] = valueSettings(intLatitude[1], 0, 9);
                }
                  
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(12,1);
              lcd.print(" ");
            }
            break;
      
            case 3:
            {
              lcd.setCursor(14,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(14,2);
                  lcd.print(intLongitude[2]);
                  intLongitude[2] = valueSettings(intLongitude[2], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(14,3);
                  lcd.print(intLatitude[2]);
                  intLatitude[2] = valueSettings(intLatitude[2], 0, 9);
                }
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(14,1);
              lcd.print(" ");
            }
            break;
      
            case 4:
            {
              lcd.setCursor(15,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(15,2);
                  lcd.print(intLongitude[3]);
                  intLongitude[3] = valueSettings(intLongitude[3], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(15,3);
                  lcd.print(intLatitude[3]);
                  intLatitude[3] = valueSettings(intLatitude[3], 0, 9);
                }
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(15,1);
              lcd.print(" ");
            }
            break;
      
            case 5:
            {
              lcd.setCursor(16,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(16,2);
                  lcd.print(intLongitude[4]);
                  intLongitude[4] = valueSettings(intLongitude[4], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(16,3);
                  lcd.print(intLatitude[4]);
                  intLatitude[4] = valueSettings(intLatitude[4], 0, 9);
                }
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(16,1);
              lcd.print(" ");
            }
            break;
      
            case 6:
            {
              lcd.setCursor(17,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(17,2);
                  lcd.print(intLongitude[5]);
                  intLongitude[5] = valueSettings(intLongitude[5], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(17,3);
                  lcd.print(intLatitude[5]);
                  intLatitude[5] = valueSettings(intLatitude[5], 0, 9);
                }
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(17,1);
              lcd.print(" ");
            }
            break;
      
            case 7:
            {
              lcd.setCursor(18,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(18,2);
                  lcd.print(intLongitude[6]);
                  intLongitude[6] = valueSettings(intLongitude[6], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(18,3);
                  lcd.print(intLatitude[6]);
                  intLatitude[6] = valueSettings(intLatitude[6], 0, 9);
                }
                pos = menuChange(pos, 8, 0);
                
              }
              menuState = true;
              lcd.setCursor(18,1);
              lcd.print(" ");
            }
            break;
      
            case 8:
            {
              lcd.setCursor(19,1);
              lcd.write(LCDcursor);
              while(menuState)
              {
                if(choise == 1)
                {
                  lcd.setCursor(19,2);
                  lcd.print(intLongitude[7]);
                  intLongitude[7] = valueSettings(intLongitude[7], 0, 9);
                }
                else if(choise == 2)
                {
                  lcd.setCursor(19,3);
                  lcd.print(intLatitude[7]);
                  intLatitude[7] = valueSettings(intLatitude[7], 0, 9);
                }
                pos = menuChange(pos, 9, 0);
                
              }
              menuState = true;
              lcd.setCursor(19,1);
              lcd.print(" ");
            }
            break;
          }
          
        }
      }
    }
    lcd.setCursor(9,3);
    lcd.print(" ");
    lcd.setCursor(9,2);
    lcd.print(" ");
    saveLongitude();
    saveLatitude();
    readLongitude();
    readLatitude();
    calcBasicValues();
  }
}
//
//------------------------------ Value settings ----------------------
int valueSettings(int val, int minVal, int maxVal)
{
  if(buttonRightPressed())
  {
    val++;
    if(val > maxVal)
      val = minVal;
    return val;
  }
  if(buttonLeftPressed())
  {
    val--;
    if(val < minVal)
      val = maxVal;
    return val;
  }
  return val;
}
//
//------------------------------ Limits check ----------------------
int limitsCheck(int checkedVal, int assignedVal, int minVal, int maxVal)
{
  if(assignedVal >= minVal && assignedVal <= maxVal)
    return assignedVal;
  else
    return checkedVal;
}
//
//--------------------------- Mapping value ----------------------
double mapF(double x, double in_min, double in_max, double out_min, double out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//--------------------------------- Timer ----------------------
bool timerAvailable()
{
  if(timer.available())
  {
    timer.restart();
    return true;
  }
  else
  {
    return false;
  }
}

bool timerWifiAvailable()
{
  if(timerWifi.available())
  {
    timerWifi.restart();
    return true;
  }
  else
  {
    return false;
  }
}
//
//--------------------------- Potentiometers readings ----------------------
void readPotValues()
{
  valPotElevation = analogValAverage(numberOfSamples, PotElevation);
  valPotAzimuth = analogValAverage(numberOfSamples, PotAzimuth);
}
//
//------------------- Calculate max/min resistance as degrees ----------------------
void limitResistancesToDegrees()
{
  //valPotAziDeg, valPotElevDeg;
  //elevMinDeg, elevMaxDeg, aziMinDeg, aziMaxDeg;
  //minAziResistance = 1000, maxAziResistance = 3000, minElevResistance = 500;
  //elevRes90 = 3900, elevRes45 = 2300, aziRes180 = 2000, aziRes225 = 2700;

  oneDegElev = (elevRes90 - elevRes45)/45.;
  oneDegAzi = (aziRes225 - aziRes180)/45.;
  elevMinDeg = 90-(elevRes90 - minElevResistance)/oneDegElev;
  elevMaxDeg = 90;
  aziMinDeg = 180 - ((aziRes180-minAziResistance)/oneDegAzi);
  aziMaxDeg = 180 + ((maxAziResistance - aziRes180)/oneDegAzi);
  
  //Serial.print(oneDegElev);
  //Serial.print("\t");
  //Serial.print(oneDegAzi);
  //Serial.print("\t");
  //Serial.print(elevMinDeg);
  //Serial.print("\t");
  //Serial.print(elevMaxDeg);
  //Serial.print("\t");
  //Serial.print(aziMinDeg);
  //Serial.print("\t");
  //Serial.print(aziMaxDeg);
  //Serial.print("\t");
}
//
//------------------------ Calculate resistance as degrees ----------------------
void resistanceToDegrees(int aziR, int elevR)
{
  valPotAziDeg = mapF((double)aziR, (double)minAziResistance, (double)maxAziResistance, aziMinDeg, aziMaxDeg);
  valPotElevDeg = mapF((double)elevR, (double)minElevResistance, (double)elevRes90, elevMinDeg, elevMaxDeg);

//  Serial.print(valPotAziDeg);
//  Serial.print("\t");
//  Serial.println(valPotElevDeg);
}
//
//-------------------------- Positioning with resistance ----------------------
void positioningWithResistance()
{
  if(buttonRightPressed())
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("A180");
    lcd.write(LCDdegree);
    lcd.setCursor(7,0);
    lcd.print("A225");
    lcd.write(LCDdegree);
    lcd.setCursor(0,2);
    lcd.print("E45");
    lcd.write(LCDdegree);
    lcd.setCursor(7,2);
    lcd.print("E90");
    lcd.write(LCDdegree);

    lcd.setCursor(1,1);
    LCDprintSpaceInt(aziRes180);
    lcd.setCursor(8,1);
    LCDprintSpaceInt(aziRes225);
    lcd.setCursor(1,3);
    LCDprintSpaceInt(elevRes45);
    lcd.setCursor(8,3);
    LCDprintSpaceInt(elevRes90);

   int pos = 1;
    while(pos != 5)
    {
      lcd.setCursor(15,0);
      lcd.print("A-Res");
      lcd.setCursor(15,2);
      lcd.print("E-Res");

      readPotValues();
      lcd.setCursor(16,1);
      LCDprintSpaceInt(valPotAzimuth);
      lcd.setCursor(16,3);
      LCDprintSpaceInt(valPotElevation);
      
      switch(pos)
      {
        case 1:
        {
          lcd.setCursor(7,1);
          lcd.print(" ");
          lcd.setCursor(0,1);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 4, 0);
          if(buttonRightPressed())
          {
            //aziRes180 = valPotAzimuth;
            aziRes180 = limitsCheck(aziRes180, valPotAzimuth, 0, aziRes225);
          }
//          aziRes180 = valueSettings(aziRes180, 0, ANALOGpinRange);
          lcd.setCursor(1,1);
          LCDprintSpaceInt(aziRes180);
        }
        break;

        case 2:
        {
          lcd.setCursor(0,1);
          lcd.print(" ");
          lcd.setCursor(0,3);
          lcd.print(" ");
          lcd.setCursor(7,1);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 4, 0);
          if(buttonRightPressed())
          {
            aziRes225 = limitsCheck(aziRes225, valPotAzimuth, aziRes180, ANALOGpinRange);
          }
//          aziRes225 = valueSettings(aziRes225, 0, ANALOGpinRange);
          lcd.setCursor(8,1);
          LCDprintSpaceInt(aziRes225);
        }
        break;

        case 3:
        {
          lcd.setCursor(7,1);
          lcd.print(" ");
          lcd.setCursor(7,3);
          lcd.print(" ");
          lcd.setCursor(0,3);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 4, 0);
          if(buttonRightPressed())
          {
            elevRes45 = limitsCheck(elevRes45, valPotElevation, 0, elevRes90);
          }
//          elevRes45 = valueSettings(elevRes45, 0, ANALOGpinRange);
          LCDprintSpaceInt(elevRes45);
          lcd.setCursor(8,3);
        }
        break;

        case 4:
        {
          lcd.setCursor(0,1);
          lcd.print(" ");
          lcd.setCursor(0,3);
          lcd.print(" ");
          lcd.setCursor(7,3);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 5, 0);
          if(buttonRightPressed())
          {
            elevRes90 = limitsCheck(elevRes90, valPotElevation, elevRes45, ANALOGpinRange);
          }
//          elevRes90 = valueSettings(elevRes90, 0, ANALOGpinRange);
          lcd.setCursor(8,3);
          LCDprintSpaceInt(elevRes90);          
        }
        break;
      }
    }
    save4DigitNumber(aziRes180, EEPROMaziRes180);
    save4DigitNumber(aziRes225, EEPROMaziRes225);
    save4DigitNumber(elevRes45, EEPROMelevRes45);
    save4DigitNumber(elevRes90, EEPROMelevRes90);
    calcBasicValues();
    
    menuState = false;
  }
}
//
//--------------------------- Setting limit resistances ----------------------
void settingLimitResistances()
{
  if(buttonRightPressed())
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("A-Min");
    lcd.setCursor(7,0);
    lcd.print("A-Max");
    lcd.setCursor(0,2);
    lcd.print("E-Min");
    lcd.setCursor(7,2);

    lcd.setCursor(1,1);
    LCDprintSpaceInt(minAziResistance);
    lcd.setCursor(8,1);
    LCDprintSpaceInt(maxAziResistance);
    lcd.setCursor(1,3);
    LCDprintSpaceInt(minElevResistance);

   int pos = 1;
    while(pos != 4)
    {
      lcd.setCursor(15,0);
      lcd.print("A-Res");
      lcd.setCursor(15,2);
      lcd.print("E-Res");

      readPotValues();
      lcd.setCursor(16,1);
      LCDprintSpaceInt(valPotAzimuth);
      lcd.setCursor(16,3);
      LCDprintSpaceInt(valPotElevation);
      
      switch(pos)
      {
        case 1:
        {
          lcd.setCursor(7,1);
          lcd.print(" ");
          lcd.setCursor(0,1);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 3, 0);
          if(buttonRightPressed())
          {
            minAziResistance = limitsCheck(minAziResistance, valPotAzimuth, 0, maxAziResistance);
          }
          lcd.setCursor(1,1);
          LCDprintSpaceInt(minAziResistance);
        }
        break;

        case 2:
        {
          lcd.setCursor(0,1);
          lcd.print(" ");
          lcd.setCursor(0,3);
          lcd.print(" ");
          lcd.setCursor(7,1);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 3, 0);
          if(buttonRightPressed())
          {
            maxAziResistance = limitsCheck(maxAziResistance, valPotAzimuth, minAziResistance, ANALOGpinRange);
          }
          lcd.setCursor(8,1);
          LCDprintSpaceInt(maxAziResistance);
        }
        break;

        case 3:
        {
          lcd.setCursor(7,1);
          lcd.print(" ");
          lcd.setCursor(0,1);
          lcd.print(" ");
          lcd.setCursor(0,3);
          lcd.write(LCDchoose);
          pos = menuChange(pos, 4, 0);
          if(buttonRightPressed())
          {
            minElevResistance = limitsCheck(minElevResistance, valPotElevation, 0, elevRes90);
          }
          LCDprintSpaceInt(minElevResistance);
          lcd.setCursor(8,3);
        }
        break;
      }
    }
    save4DigitNumber(minAziResistance, EEPROMminAziResistance);
    save4DigitNumber(maxAziResistance, EEPROMmaxAziResistance);
    save4DigitNumber(minElevResistance, EEPROMminElevResistance);
    calcBasicValues();
    
    menuState = false;
  }
}
//
//------------------------------ Set actuator angle ----------------------
void setActuatorAngle()
{
  if(buttonRightPressed())
  {
    readPotValues();
    resistanceToDegrees(valPotAzimuth, valPotElevation);
    int setAziAngle = round(valPotAziDeg), setElevAngle = round(valPotElevDeg);
    int pos = 1;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Azimuth");
    lcd.setCursor(11,0);
    lcd.print("Elevation");
    lcd.setCursor(6, 2);
    lcd.print("Current");

    while(!buttonMenuLeftPressed())
    {
      readPotValues();
      resistanceToDegrees(valPotAzimuth, valPotElevation);
      lcd.setCursor(0,1);
      LCDprintSpace(setAziAngle,0);
      lcd.write(LCDdegree);
      lcd.setCursor(16,1);
      LCDprintSpace(setElevAngle,0);
      lcd.write(LCDdegree);
      lcd.setCursor(0,3);
      LCDprintSpace(valPotAziDeg,2);
      lcd.write(LCDdegree);
      lcd.setCursor(13,3);
      LCDprintSpace(valPotElevDeg,2);
      lcd.write(LCDdegree);

      if(buttonRightPressed())
      {
        while(!buttonMenuLeftPressed())
        {
          readPotValues();
          resistanceToDegrees(valPotAzimuth, valPotElevation);
          lcd.setCursor(0,1);
          LCDprintSpace(setAziAngle,0);
          lcd.write(LCDdegree);
          lcd.setCursor(16,1);
          LCDprintSpace(setElevAngle,0);
          lcd.write(LCDdegree);
          lcd.setCursor(0,3);
          LCDprintSpace(valPotAziDeg,2);
          lcd.write(LCDdegree);
          lcd.setCursor(13,3);
          LCDprintSpace(valPotElevDeg,2);
          lcd.write(LCDdegree);
          switch(pos)
          {
            case 1:
            {
              lcd.setCursor(10,0);
              lcd.print(" ");
              lcd.setCursor(7,0);
              lcd.write(LCDchoose);
              setAziAngle = valueSettings(setAziAngle, aziMinDeg, aziMaxDeg);
              pos = menuChange(pos, 2, 1);
            }
            break;
    
            case 2:
            {
              lcd.setCursor(7,0);
              lcd.print(" ");
              lcd.setCursor(10,0);
              lcd.write(LCDchoose);             
              setElevAngle = valueSettings(setElevAngle, elevMinDeg, elevMaxDeg);
              pos = menuChange(pos, 2, 1);
            }
            break;
          }
        }
        lcd.setCursor(10,0);
        lcd.print(" ");
        lcd.setCursor(7,0);
        lcd.print(" ");
      }
      settingActuatorsPosition(valPotAziDeg, setAziAngle, valPotElevDeg, setElevAngle);
    }
    stopAzimuth();
    stopElevation();
    menuState = false;
    
  }
}
//
//-------------------- Setting the position of the actuators ----------------------
void settingActuatorsPosition(double actualDegAzi, double degAzi, double actualDegElev, double degElev)
{
  if(actualDegAzi < degAzi-accuracy)
  {
    closeAzimuth();
  }
  else if(actualDegAzi > degAzi+accuracy)
  {
    openAzimuth();
  }
  else
  {
    stopAzimuth();
  }

  if(actualDegElev < degElev-accuracy)
  {
    closeElevation();
  }
  else if(actualDegElev > degElev+accuracy)
  {
    openElevation();
  }
  else
  {
    stopElevation();
  }
  
}
//
//--------------------------- Print actual parameters ----------------------
void printActualParameters()
{
  updateTime();
  sunPosCalc(nHour-UTC, nMinute);

  lcd.setCursor(2,0);
  lcd.print("Time");
  lcd.setCursor(8,0);
  lcd.print("SA[");
  lcd.write(LCDdegree);
  lcd.print("]");
  lcd.setCursor(15,0);
  lcd.print("SE[");
  lcd.write(LCDdegree);
  lcd.print("]");

  lcd.setCursor(0,2);
  lcd.write(LCDnow);
  DLCDprint2digits(nHour);
  lcd.print(":");
  DLCDprint2digits(nMinute);
  lcd.print(" ");

  readPotValues();
  resistanceToDegrees(valPotAzimuth, valPotElevation);

  if(sunPosPotValChoose == 0)
  {
    LCDprintSpace(valPotAziDeg, 2);
    lcd.print(" ");
    LCDprintSpace(valPotElevDeg, 2);
    if(buttonRightPressed())
      sunPosPotValChoose = 1;
  }
  else if(sunPosPotValChoose == 1)
  {
    LCDprintSpace(nAzimuthAngle,2);
    lcd.print(" ");
    LCDprintSpace(nElevationAngle,2);
    if(buttonRightPressed())
      sunPosPotValChoose = 0;
  }

  lcd.setCursor(0,1);
  lcd.write(LCDnext);

  lcd.setCursor(0,3);
  lcd.write(LCDprev);
  
  int n;
  if(checkIfItsSwitchOnTime(nHour, nMinute) != -2)
  {
    n = checkIfItsSwitchOnTime(nHour, nMinute);
    if(switchOnTable[n+4] == -1)
    {
      lcd.setCursor(1,1);
      lcd.print("                   ");
      printActualParametersLCD(3, switchOnTable[n-4], switchOnTable[n-3], switchOnTable[n-2], switchOnTable[n-1]);
    }
    else if(n == 0)
    {
      printActualParametersLCD(1, switchOnTable[n+4], switchOnTable[n+5], switchOnTable[n+6], switchOnTable[n+7]);
      lcd.setCursor(1,3);
      lcd.print("                   ");
    }
    else
    {
      printActualParametersLCD(1, switchOnTable[n+4], switchOnTable[n+5], switchOnTable[n+6], switchOnTable[n+7]);
      printActualParametersLCD(3, switchOnTable[n-4], switchOnTable[n-3], switchOnTable[n-2], switchOnTable[n-1]);
    }
  }
  else if((nHour == sunsetHour && nMinute > sunsetMin) || (nHour > sunsetHour))
  {
    lcd.setCursor(1,1);
    lcd.print("                   ");
    lcd.setCursor(1,3);
    lcd.print("                   ");
  }
  else if((nHour < sunriseHour) || (nHour == sunriseHour && nMinute < sunriseMin))
  {
    printActualParametersLCD(1, switchOnTable[0], switchOnTable[1], switchOnTable[2], switchOnTable[3]);
    lcd.setCursor(1,3);
    lcd.print("                   ");
  }
  else
  {
    double hr = nHour, mn = nMinute;
    while(checkIfItsSwitchOnTime(hr, mn) == -2)
    {
      mn++;
      if(mn > 59)
      { 
        mn = 0;
        hr++;
      }

      if(hr == sunsetHour && mn > sunsetMin)
      {
        mn = mn - downtimeMin;
        if(mn < 0)
        {
          hr--;
          mn = mn + 60;
        }
      }
      else if(hr > sunsetHour)
      {
        hr--;
        mn = mn - downtimeMin;
        if(mn < 0)
        {
          hr--;
          mn = mn + 60;
        }
      }
      n = checkIfItsSwitchOnTime(hr, mn);
    }
    if(switchOnTable[n+4] == -1 && ((nHour == hr && nMinute > mn) || nHour > hr))
    {
      lcd.setCursor(1,1);
      lcd.print("                   ");
      printActualParametersLCD(3, switchOnTable[n], switchOnTable[n+1], switchOnTable[n+2], switchOnTable[n+3]);
    }
    else if(n == 0)
    {
      printActualParametersLCD(1, switchOnTable[0], switchOnTable[1], switchOnTable[2], switchOnTable[3]);
      lcd.setCursor(1,3);
      lcd.print("                   ");
    }
    else
    {
      printActualParametersLCD(1, switchOnTable[n], switchOnTable[n+1], switchOnTable[n+2], switchOnTable[n+3]);
      printActualParametersLCD(3, switchOnTable[n-4], switchOnTable[n-3], switchOnTable[n-2], switchOnTable[n-1]);
    }
  }
}
//
//--------------------- Print actual parameters LCD ----------------------
void printActualParametersLCD(int verse, double h, double m, double a, double e)
{
  lcd.setCursor(1,verse);
  DLCDprint2digits(h);
  lcd.print(":");
  DLCDprint2digits(m);
  lcd.print(" ");
  LCDprintSpace(a, 2);
  lcd.print(" ");
  LCDprintSpace(e, 2);
}
//
//------------------ Checking if it's time to switch on ----------------------
int checkIfItsSwitchOnTime(double h, double m)
{
  for(int i=0; i<tableSize; i=i+4)
  {
    if(switchOnTable[i] == h && switchOnTable[i+1] == m)
      return i;
  }
  return -2;
}
//
//----------------------------- Set UTC ----------------------
void setUTC()
{
  lcd.setCursor(18,2);
  UTC = valueSettings(UTC, 0, 9);
  if(UTC > 0)
    lcd.print("+");
  else
    lcd.print(" ");
  lcd.print(UTC);
}
//
//--- Calculate sunrise, sunset and switch on table after settings change ----------------------
void calcBasicValues()
{
  limitResistancesToDegrees();
  sunriseTimeCalc();
  sunsetTimeCalc();
  switchOnTableCalc();
}
//
//----------------------- Set Downtime in mins ----------------------
void setDowntimeMin()
{
  int val = downtimeMin/4;
  
  val = valueSettings(val, 1, 7);
  downtimeMin = val*4;
  lcd.setCursor(7,3);
  if(downtimeMin < 10)
  {
    lcd.print(" ");
  }
  lcd.print(downtimeMin,0);
  lcd.print(" min");
}
//
//------------------------- Actuators work ----------------------
void actuatorsWork()
{
  if(digitalRead(SwitchAuto) && manualMode == false)
  {
    int check;
    updateTime();
    readPotValues();
    resistanceToDegrees(valPotAzimuth, valPotElevation);
    check = checkIfItsSwitchOnTime(nHour, nMinute);
    if(autoMode == manualMode && manualMode == false)
    {
      work = true;
      autoMode = true;
    }
    if(check != -2)
    {
      aziDegWork = switchOnTable[check+2];
      elevDegWork = switchOnTable[check+3];
      work = true;
    }
    if((nHour == sunsetHour && nMinute > sunsetMin) || nHour > sunsetHour)
    {
      work = true;
      aziDegWork = 180;
      elevDegWork = 90;
    }
    if(work == true)
    {
      settingActuatorsPosition(valPotAziDeg, aziDegWork, valPotElevDeg, elevDegWork);
      if(valPotAziDeg > aziDegWork - accuracy && valPotAziDeg < aziDegWork + accuracy && valPotElevDeg > elevDegWork - accuracy && valPotElevDeg < elevDegWork + accuracy)
        work = false;
    }
  }
  if(digitalRead(SwitchAuto) && manualMode)
  {
    readPotValues();
    resistanceToDegrees(valPotAzimuth, valPotElevation);
    work = true;
    autoMode = false;
    if(work == true)
    {
      if(valPotAziDeg > setAzimuthManual - accuracy && valPotAziDeg < setAzimuthManual + accuracy && setElevationManual > elevDegWork - accuracy && valPotElevDeg < setElevationManual + accuracy)
        work = false;
      settingActuatorsPosition(valPotAziDeg, setAzimuthManual, valPotElevDeg, setElevationManual);
            if(valPotAziDeg > setAzimuthManual - accuracy && valPotAziDeg < setAzimuthManual + accuracy && setElevationManual > elevDegWork - accuracy && valPotElevDeg < setElevationManual + accuracy)
        work = false;
    }
  }
}
//
//--------------------- Hour and Date Settings ----------------------
void timeAndDateSetting()
{
  if(buttonRightPressed())
  {
    xSemaphoreTake(mutex,portMAX_DELAY);
    int pos = 1;
    updateTime();
    int h = nHour, m= nMinute, s = nSecond, d = nDay, mth = nMonth, y = nYear;
    while(pos != 7)
    {
      DateTime now = rtc.now();
      lcd.setCursor(6,0);
      DLCDprint2digits(nHour);
      lcd.print(":");
      DLCDprint2digits(nMinute);
      lcd.print(":");
      LCDprint2digits(now.second());
      lcd.setCursor(5,1);
      LCDprint2digits(nDay);
      lcd.print("/");
      LCDprint2digits(nMonth);
      lcd.print("/");
      LCDprint2digits(nYear);
      
      switch(pos)
      {
        case 1:
        {
          lcd.setCursor(19,0);
          lcd.print("H");
          nHour = valueSettings(nHour, 0, 23);
          pos = menuChange(pos, 6, 0);
        }
        break;

        case 2:
        {
          lcd.setCursor(19,0);
          lcd.print("M");
          nMinute = valueSettings(nMinute, 0, 59);
          pos = menuChange(pos, 6, 0);
        }
        break;
        
        case 3:
        {
          lcd.setCursor(19,1);
          lcd.print(" ");
          lcd.setCursor(19,0);
          lcd.print("S");
          if(buttonRightPressed())
            rtc.adjust(DateTime(nYear, nMonth, nDay, nHour, nMinute, 0));
          pos = menuChange(pos, 6, 0);
        }
        break;

        case 4:
        {
          lcd.setCursor(19,0);
          lcd.print(" ");
          lcd.setCursor(19,1);
          lcd.print("D");
          nDay = valueSettings(nDay, 1, 31);
          pos = menuChange(pos, 6, 0);
        }
        break;

        case 5:
        {
          lcd.setCursor(19,1);
          lcd.print("M");
          nMonth = valueSettings(nMonth, 1, 12);
          pos = menuChange(pos, 6, 0);
        }
        break;

        case 6:
        {
          lcd.setCursor(19,0);
          lcd.print(" ");
          lcd.setCursor(19,1);
          lcd.print("Y");
          nYear = valueSettings(nYear, 1900, 3000);
          pos = menuChange(pos, 7, 0);
        }
        break;

        
      }
    }
    DateTime now = rtc.now();
    rtc.adjust(DateTime(nYear, nMonth, nDay, nHour, nMinute, now.second()));
    lcd.setCursor(19,1);
    lcd.print(" ");
    calcBasicValues();
    checkDay = nDay;
    xSemaphoreGive(mutex);
    delay(200); // Short delay is needed!
  }
}
//
//------------------- Back to first page of menu ----------------------
void backToFirstPage()
{
  if(timerAvailable())
  {
    backCounter++;
    if(backCounter >= timeToBack)
    {
      menuState = false;
      actualMenuPosition = 1;
    }
  }
}
//
//------------------- Check if the day is changed ----------------------
void checkDayChange()
{
  updateTime();
  if(checkDay != nDay)
  {
    calcBasicValues();
    checkDay = nDay;
  }
}
//
void setupAccessPoint() {

  WiFi.softAP(ap_ssid, ap_password);
  //Serial.println("Access Point Created");
  //Serial.print("IP Address: ");
  //Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>ESP32 Setup</title>";
    html += "<style>body { font-family: Arial, sans-serif; background-color: #f4f4f9; color: #333; text-align: center; margin: 0; padding: 0; }";
    html += "form { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); display: inline-block; margin-top: 50px; }";
    html += "input[type='text'], input[type='password'], input[type='submit'] { display: block; width: calc(100% - 24px); padding: 10px; margin: 10px auto; font-size: 16px; }";
    html += "label { display: block; margin: 20px 0 10px; font-weight: bold; }";
    html += "input[type='submit'] { background-color: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; }";
    html += "input[type='submit']:hover { background-color: #45a049; }";
    html += "</style></head><body><form action='/get' method='get'>";
    html += "<label for='server_ip'>Server IP:</label><input type='text' name='server_ip' value='" + serverIp + "'>";
    html += "<label for='wifi_ssid'>WiFi SSID:</label><input type='text' name='wifi_ssid' value='" + wifiSSID + "'>";
    html += "<label for='wifi_password'>WiFi Password:</label><input type='password' name='wifi_password' value='" + wifiPassword + "'>";
    html += "<input type='submit' value='Submit'></form></body></html>";

    server.send(200, "text/html", html);
  });

  server.on("/get", HTTP_GET, []() {
    if (server.hasArg(PARAM_INPUT_2) && server.hasArg(PARAM_INPUT_3) && server.hasArg(PARAM_INPUT_4)) {
      serverIp = server.arg(PARAM_INPUT_2);
      wifiSSID = server.arg(PARAM_INPUT_3);
      wifiPassword = server.arg(PARAM_INPUT_4);

      preferences.begin("wifi-creds", false);
      preferences.putString(PARAM_SSID, wifiSSID);
      preferences.putString(PARAM_PASSWORD, wifiPassword);
      preferences.putString(PARAM_SERVER, serverIp);
      preferences.end();

      server.send(200, "text/html", "Settings saved. Server IP: " + serverIp + " WiFi SSID: " + wifiSSID);

      WiFi.softAPdisconnect(true);
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
        attempts++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        newWifiStatus = Connected;
        wifiReady = true;
      } else {
        newWifiStatus = LostConnect;
      }
    } else {
      server.send(200, "text/html", "Invalid input");
    }
  });

  server.begin();
  IP = WiFi.softAPIP();
  newWifiStatus = WifiStatus::WaitToSetup;
}

void checkAndSyncData() {
  http.begin("https://" + serverIp + "/get_data_esp32.php?id=" + idController);
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    dataStatus = true;
    String payload = http.getString();
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("Failed to parse JSON");
      return;
    }

      if (doc["updated_by_app"].as<int>() > 0)
        updated_by_app = true;
      else
        updated_by_app = false;

    if (updated_by_app) {
      updateLocalVariables(payload);
      clearUpdatedByAppOnServer();
      clearDateUpdatedOnServer();
    } else {
      sendDataHTTP();
    }
  } else {
    Serial.println("Error on fetching data: " + String(httpCode));
    dataStatus = false;
  }
  http.end();
}

void sendDataHTTP() {
  HTTPClient http;
  http.begin("https://" + serverIp + "/update_data_esp32.php");
  http.addHeader("Content-Type", "application/json");

  String jsonData = createJsonData();

  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  http.end();
}

String createJsonData() {
  DynamicJsonDocument doc(2048);

  doc["data_time"] = getFormattedTime(nYear, nMonth, nDay, nHour, nMinute, nSecond);
  doc["timezone_offset"] = UTC;
  doc["longitude"] = nLongitude;
  doc["latitude"] = nLatitude;
  doc["azimuth"] = valPotAziDeg;
  doc["elevation"] = valPotElevDeg;
  doc["target_azimuth"] = setAzimuthManual;
  doc["target_elevation"] = setElevationManual;
  doc["battery_capacity"] = batteryCapacity;
  doc["battery_voltage"] = batteryVoltage;
  doc["battery_charge_current"] = batteryChargeCurrent;
  doc["pv_voltage"] = solarPanelVoltage;
  doc["pv_power"] = solarPanelPower;
  doc["zenith"] = nZenithAngle;
  doc["sunrise"] = getFormattedTime(nYear, nMonth, nDay, sunriseHour, sunriseMin, nSecond);
  doc["sunset"] = getFormattedTime(nYear, nMonth, nDay, sunsetHour, sunsetMin, nSecond);
  doc["is_manual_mode"] = manualMode ? 1 : 0;
  doc["min_azimuth"] = aziMinDeg;
  doc["max_azimuth"] = aziMaxDeg;
  doc["min_elevation"] = elevMinDeg;
  doc["max_elevation"] = elevMaxDeg;
  doc["auto_azimuth"] = nAzimuthAngle;
  doc["auto_elevation"] = nElevationAngle;
  doc["actuator_frequency"] = downtimeMin;
  doc["id"] = idController; 

  String jsonData;
  serializeJson(doc, jsonData);
  return jsonData;
}

String getFormattedTime(int year, int month, int day, int hour, int minute, int second) {
  char formattedTime[20];
  sprintf(formattedTime, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
  return String(formattedTime);
}

void clearUpdatedByAppOnServer() {
  http.begin("https://" + serverIp + "/clear_updated_by_app.php");
  http.addHeader("Content-Type", "application/json");
  StaticJsonDocument<128> doc;
  doc["updated_by_app"] = false;
  doc["id"] = idController;
  String jsonData;
  serializeJson(doc, jsonData);
  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    //Serial.println(response);
  } else {
    //Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  http.end();
}

void clearDateUpdatedOnServer() {
  http.begin("https://" + serverIp + "/clear_data_updated.php");
  http.addHeader("Content-Type", "application/json");
  StaticJsonDocument<128> doc;
  doc["date_updated"] = false;
  doc["id"] = idController;
  String jsonData;
  serializeJson(doc, jsonData);
  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    //Serial.println(response);
  } else {
    //Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  http.end();
}



void updateLocalVariables(String json) {
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    //Serial.println("Failed to parse JSON");
    return;
  }

  xSemaphoreTake(mutex,portMAX_DELAY);

  UTC = doc["timezone_offset"];
  nLongitude = doc["longitude"];
  nLatitude = doc["latitude"];
  setAzimuthManual = doc["target_azimuth"];
  setElevationManual = doc["target_elevation"];

  if (doc["is_manual_mode"].as<int>() > 0)
    manualMode = true;
  else
    manualMode = false;

  downtimeMin = doc["actuator_frequency"];

  if (doc["updated_by_app"].as<int>() > 0)
    updated_by_app = true;
  else
    updated_by_app = false;


  if (doc["date_updated"].as<int>() > 0)
    dataUpdated = true;
  else
    dataUpdated = false;
  //Serial.println(doc["date_updated"].as<int>());

  if (dataUpdated)
  {
    String dataTime = doc["data_time"].as<String>();

    nYear = dataTime.substring(0, 4).toInt();
    nMonth = dataTime.substring(5, 7).toInt();
    nDay = dataTime.substring(8, 10).toInt();
    nHour = dataTime.substring(11, 13).toInt();
    nMinute = dataTime.substring(14, 16).toInt();
    nSecond = dataTime.substring(17, 19).toInt();

    rtc.adjust(DateTime(nYear, nMonth, nDay, nHour, nMinute, nSecond));
  }
  saveNumber0_255(UTC, EEPROMUTC);
  xSemaphoreGive(mutex);
  delay(200); // Short delay is needed!

  saveLongitude();
  saveLatitude();
  calcBasicValues();

}

void loopEthernet()
{
  server.handleClient();
  if(wifiReady)
  {
    if (timerWifiAvailable()) {
        if (WiFi.status() == WL_CONNECTED) {
          newWifiStatus = Connected;
          checkAndSyncData();
        } else {
          newWifiStatus = LostConnect;
          dataStatus = false;
          reconnectWiFi();
        }
    }
  }
}

void reconnectWiFi() {
  WiFi.disconnect();
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  int attempts = 0;
  //Serial.print("WL_CONNECTED..");
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    //Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    newWifiStatus = Connected;
    //Serial.println("Reconnected to WiFi");
  } else {
    //Serial.println("Failed to reconnect to WiFi");
  }
}

void pageEnthernet()
{
  wifiStatus = newWifiStatus;
  printPageEnthernet();

}

void printPageEnthernet()
{
switch(wifiStatus)
  {
  case WaitToSetup:
      lcd.setCursor(0, 0);
      lcd.print("AP IP: ");
      lcd.print(IP);
      lcd.setCursor(0, 1);
      lcd.print("Connect and set WiFi");
      break;
  case Connected:
      lcd.setCursor(0, 0);
      lcd.print("WiFi Connected!");
      lcd.setCursor(0, 1);
          lcd.print("Data Status: ");
      if (dataStatus) {
        lcd.print("Success");
      } else {
        lcd.print("Failed");
      }
      break;
  case LostConnect:
      lcd.setCursor(0, 0);
      lcd.print("Failed to connect");
      lcd.setCursor(0, 1);
      lcd.print("to WiFi");
      break;
  }
  
  if(manualMode)
  {
    lcd.setCursor(0, 2);
    lcd.print("Mode by app: Manual");
  }
  else
  {
    lcd.setCursor(0, 2);
    lcd.print("Mode by app: Auto");
  }
}

void updatePageEnthernet()
{
  if (buttonRightPressed()) {
        WiFi.disconnect(true);
        delay(1000);
        setupAccessPoint();
        wifiReady = false;
  }
  if (buttonLeftPressed()) {
      if(manualMode)
      {
        manualMode = false;
        caseInitialConditions();
        printPageEnthernet();
      }
  }
  if(newWifiStatus != wifiStatus || dataStatus != dataStatusShow || manualModeUpdate != manualMode)
  {
    manualModeUpdate = manualMode;
    caseInitialConditions();
    wifiStatus = newWifiStatus;
    dataStatusShow = dataStatus;
    printPageEnthernet();
  }
}

void wifiTask(void *parameter) {
    while(1){
        loopEthernet();
    }
}

void modbusTask(void *parameter) {
    while(1){
        readModbusData();
        delay(intervalModBus);
    }
}

void startupWifi()
{
  preferences.begin("wifi-creds", false);

  wifiSSID = preferences.getString(PARAM_SSID, "");
  wifiPassword = preferences.getString(PARAM_PASSWORD, "");
  serverIp = preferences.getString(PARAM_SERVER, "");
  preferences.end();
  wifiReady = true;
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  if (WiFi.status() == WL_CONNECTED) {
    newWifiStatus = Connected;
  } else {
    newWifiStatus = LostConnect;
  }
}

void setIdController()
{
  idController = valueSettings(idController, 1, 2);
  lcd.setCursor(9,3);
  if(idController < 10)
  {
    lcd.print(" ");
  }
  lcd.print(idController,0);
}
//############################ END FUNCTIONS #############################




//################################ SETUP #################################
//
void setup() {
  lcd.init();
  lcd.backlight();
  delay(200);
  
  startupWifi();
  initModbus();

  rtc.begin();

  EEPROM.begin(EEPROMsize);

  
  pinMode(ButtonLeft, INPUT);
  pinMode(ButtonRight, INPUT);
  pinMode(ButtonMenuRight, INPUT);
  pinMode(ButtonMenuLeft, INPUT);
  pinMode(SwitchAuto, INPUT);
  pinMode(SwitchManual, INPUT);
  pinMode(OpenAzimuth,OUTPUT);
  pinMode(CloseAzimuth,OUTPUT);
  pinMode(OpenElevation,OUTPUT);
  pinMode(CloseElevation,OUTPUT);
  digitalWrite(OpenAzimuth, HIGH);
  digitalWrite(CloseAzimuth, HIGH);
  digitalWrite(OpenElevation, HIGH);
  digitalWrite(CloseElevation, HIGH);
  Serial.begin(115000);
  delay(500);

  aziRes180 = read4DigitNumber(EEPROMaziRes180);
  aziRes225 = read4DigitNumber(EEPROMaziRes225);
  elevRes45 = read4DigitNumber(EEPROMelevRes45);
  elevRes90 = read4DigitNumber(EEPROMelevRes90);
  minAziResistance = read4DigitNumber(EEPROMminAziResistance);
  maxAziResistance = read4DigitNumber(EEPROMmaxAziResistance);
  minElevResistance = read4DigitNumber(EEPROMminElevResistance);
  UTC = readNumber0_255(EEPROMUTC);
  downtimeMin = readNumber0_255(EEPROMdowntimeMin);
  idController = readNumber0_255(EEPROMidController);
  updateTime();
  readLongitude();
  readLatitude();
  limitResistancesToDegrees();
  
  sunriseTimeCalc();
//  Serial.print(sunriseHour,0);
//  Serial.print(":");
//  Serial.println(sunriseMin,0);

  sunsetTimeCalc();
//  Serial.print(sunsetHour,0);
//  Serial.print(":");
//  Serial.println(sunsetMin,0);

  switchOnTableCalc();
//for(int i=0; i<tableSize; i+=4)
//{
//  Serial.print(switchOnTable[i],0);
//  Serial.print(":");
//  Serial.print(switchOnTable[i+1],0);
//  Serial.print("   ");
//  Serial.print(switchOnTable[i+2]);
//  Serial.print("   ");
//  Serial.println(switchOnTable[i+3]);
//}

  checkDay = nDay;
  
  lcd.createChar(LCDsunrise, sunriseChar);
  lcd.createChar(LCDsunset, sunsetChar);
  lcd.createChar(LCDdegree, degreeChar);
  lcd.createChar(LCDchoose, chooseChar);
  lcd.createChar(LCDcursor, cursorChar);
  lcd.createChar(LCDnext, nextChar);
  lcd.createChar(LCDnow, nowChar);
  lcd.createChar(LCDprev, prevChar);

  timer.begin(1000);
  timerWifi.begin(intervalWifi);

  mutex = xSemaphoreCreateMutex();
  //scanModbusIDs();
  xTaskCreate(
      wifiTask,      
      "wifi",   
      16384,        
      NULL,        
      1,           
      NULL         
    );
  xTaskCreate(
      modbusTask,      
      "modbusTask",   
      4096,        
      NULL,        
      1,           
      NULL         
    );
}
//
//############################### END SETUP ###############################




//################################# LOOP ##################################
//
void loop() {
  switch(actualMenuPosition)
  {    
    case 1:
    {
      caseInitialConditions();
      while(menuState)
      {
        updateTime();
        checkDayChange();
        actuatorsWork();
        printMenuFirstPage();
        timeAndDateSetting();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
    }
    break;

    case 2:
    {
      caseInitialConditions();
      sunPosPotValChoose = 0;
      while(menuState)
      {
        actuatorsWork();
        checkDayChange();
        printActualParameters();
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
    }
    break;

    case 3:
    {
      caseInitialConditions();
      int u = UTC;
      lcd.setCursor(5,0);
      lcd.print("Time zone");
      lcd.setCursor(0,2);
      lcd.print("UTC");
      while(menuState)
      {
        setUTC();
        checkDayChange();
        actuatorsWork();;
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
      if(u != UTC)
      {
        xSemaphoreTake(mutex,portMAX_DELAY);
        saveNumber0_255(UTC, EEPROMUTC);
        xSemaphoreGive(mutex);
        delay(200); // Short delay is needed!
        calcBasicValues();
      }
    }
    break;

    case 4:
    {
      caseInitialConditions();
      int d = downtimeMin;
      lcd.setCursor(1,0);
      lcd.print("Freq. of actuators");
      lcd.setCursor(5,1);
      lcd.print("activation");
      while(menuState)
      {
        setDowntimeMin();
        checkDayChange();
        actuatorsWork();
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
      if(d != downtimeMin)
      {
        saveNumber0_255(downtimeMin, EEPROMdowntimeMin);
        calcBasicValues();
      }
    }
    break;
    
    case 5:
    {
      caseInitialConditions();
      lcd.setCursor(6,0);
      lcd.print("Location");
      lcd.setCursor(0,2);
      lcd.print("Longitude");
      lcd.setCursor(11,2);
      lcd.print(nLongitude,6);
      lcd.setCursor(0,3);
      lcd.print("Latitude");
      lcd.setCursor(11,3);
      lcd.print(nLatitude,6);
      while(menuState)
      {
        backToFirstPage();
        checkDayChange();
        actuatorsWork();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
        locationSettings();
      }
    }
    break;
    
    case 6:
    {
      caseInitialConditions();
      lcd.setCursor(3,0);
      lcd.print("Setting limit");
      lcd.setCursor(4,1);
      lcd.print("resistances");
      lcd.setCursor(1,3);
      lcd.print("Press ENTER to set");
      while(menuState)
      {
        backToFirstPage();
        checkDayChange();
        actuatorsWork();
        settingLimitResistances();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
      
    }
    break;
    
    case 7:
    {
      caseInitialConditions();
      lcd.setCursor(4,0);
      lcd.print("Positioning");
      lcd.setCursor(2,1);
      lcd.print("with resistances");
      lcd.setCursor(1,3);
      lcd.print("Press ENTER to set");
      while(menuState)
      {
        backToFirstPage();
        checkDayChange();
        actuatorsWork();
        positioningWithResistance();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
    }
    break;

    case 8:
    {
      caseInitialConditions();
      lcd.setCursor(0,0);
      lcd.print("[M] Set actuator");
      lcd.setCursor(7,1);
      lcd.print("angle");
      lcd.setCursor(1,3);
      lcd.print("Press ENTER to set");
      while(menuState)
      {
        backToFirstPage();
        checkDayChange();
        actuatorsWork();
        setActuatorAngle();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
      
    }
    break;

    case 9:
    {
      caseInitialConditions();
      lcd.setCursor(0,0);
      lcd.print("[M]  Actuators");
      lcd.setCursor(6,1);
      lcd.print("control");
      lcd.setCursor(1,3);
      lcd.print("Press ENTER to set");
      while(menuState)
      {
        backToFirstPage();
        checkDayChange();
        actuatorsWork();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
        manualActuatorsControl();
      }
      
    }
    break;
    case 10:
    {
      caseInitialConditions();
      pageEnthernet();
      while(menuState)
      {   
        updatePageEnthernet();
        checkDayChange();
        actuatorsWork();
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
    }
    break;
    case 11:
    {
      caseInitialConditions();
      int id = idController;
      lcd.setCursor(1,0);
      lcd.print("Unit id for control");
      lcd.setCursor(1,1);
      lcd.print("from application");
      while(menuState)
      {
        setIdController();
        checkDayChange();
        actuatorsWork();
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
      if(id != idController)
      {
        saveNumber0_255(idController, EEPROMidController);
      }
    }
    break;
    case 12:
    {
      caseInitialConditions();
      lcd.setCursor(0, 0);
      lcd.print("Battery Cap: ");
      lcd.setCursor(0, 1);
      lcd.print("Battery Volt: ");
      lcd.setCursor(0, 2);
      lcd.print("Charge Curr: ");
      lcd.setCursor(0, 3);
      lcd.print("PV Volt: ");

      while(menuState)
      {   
        updateDisplayInfoFromChargeController();
        checkDayChange();
        actuatorsWork();
        backToFirstPage();
        actualMenuPosition = menuChange(actualMenuPosition, nrOfMenuPositions, 0);
      }
    }
    break;
  }
}
//
//################################ END LOOP ###############################