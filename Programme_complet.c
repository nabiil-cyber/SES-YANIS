#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <MsTimer2.h>
#include <SD.h>
#include <stdlib.h>
#include <stdio.h>
#include <SdFat.h>
#include <Wire.h>
#include "DS1307.h"
#include <SoftwareSerial.h>
#include "SparkFunBME280.h"
#include <ChainableLED.h>
#include <String.h>
#include <avr/pgmspace.h>
#include <RTClib.h>



#define GPS_1 8
#define GPS_2 9
#define SD_CHIP 4
#define MAX_SD 7000000
#define LUM_SENSOR A0
#define TMP_CONF 10000
#define TMP_BTN 5000
#define CONFIG_MODE_TIME 30000
#define GPS_HEADER "$GPGGA"
#define BTN_STATE LOW

#define RBUTTON        2  // Button 1
#define GBUTTON       3  // Button 2
#define DEFAULT_LONGPRESS_LEN 5000
#define LED1 4
#define LED2 5
#define SD_CS_PIN 4

int LUMIN, LUMIN_LOW, LUMIN_HIGH, TEMP_AIR, MIN_TEMP_AIR, MAX_TEMP_AIR,
    HYGR, HYGR_MINT, HYGR_MAXT, PRESSURE, PRESSURE_MIN, PRESSURE_MAX,
    LOG_INTERVAL, FILE_MAX_SIZE, TIMEOUT, jour;


RTC_DS1307 RTC;
File myFile;
SoftwareSerial SoftSerial(GPS_1, GPS_2);
DS1307 clock;
ChainableLED leds(LED1, LED2, 1);
BME280 THPsensor;
volatile unsigned long rStart, gStart;
volatile int redTimer, greenTimer;
boolean rtc, gps, lum, airTemperature, pressure, hygro, sim, maintenanceMode = 0, economyMode = 0;
short logInterval, fileMaxSize, lumMin, lumMax, pressureMin, pressureMax;
unsigned int captureCounter = 0;
char fileCounter = 0, airTemperatureMin, airTemperatureMax, hygroMin, hygroMax;

typedef struct donnees donnees;
struct donnees {
  int mesure;
  char chaine[];
};

#define SET_THP_SENSOR  THPsensor.settings.commInterface = I2C_MODE;\
  THPsensor.settings.I2CAddress = 0x76;\
  THPsensor.settings.runMode = 3;\
  THPsensor.settings.tStandby = 0;\
  THPsensor.settings.filter = 0;\
  THPsensor.settings.tempOverSample = 1;\
  THPsensor.settings.pressOverSample = 1;\
  THPsensor.settings.humidOverSample = 1;\
  delay(10);\
  THPsensor.begin();





#define WRITE_IN_SD char fileName[15] = "";\
  sprintf(fileName, "%d%d%d_%d.LOG", clock.year, clock.month, clock.dayOfMonth, fileCounter);\
  Serial.begin(9600);\
  myFile = SD.open(fileName, FILE_WRITE);\
  if (myFile) {\
    Serial.print("Writing...");\
    if (LUMIN == 1){\
      myFile.print(analogRead(LUM_SENSOR));\
      myFile.print(" LUX|");\
    }\
    if (TEMP_AIR == 1) { \
      myFile.print(random(-20,50));\
      myFile.print(" °C|"); \
    }\
    if (PRESSURE == 1) {  \
      myFile.print(random(1,2));\
      myFile.print(" HPa|");\
    }\
    if (HYGR == 1) {  \
      myFile.print(THPsensor.readFloatHumidity());\
      myFile.print(" %|");\
    }\
    myFile.print(random(-40,126));\
    myFile.print(" °C|");\
    myFile.print(random(3,61)/10);\
    myFile.print(" L/min|");\
    myFile.print(random(1,131));\
    myFile.println(" km/h|");\
    myFile.close();\
    Serial.println("done.");\
  } else {\
    Serial.println("error opening");\
  }


#define SET_WEATHER_STATION RESET();\
  Serial.begin(9600);\
  SoftSerial.begin(9600);\
  Wire.begin();\
  RTC.begin();\
  clock.begin();\
  pinMode(RBUTTON, INPUT);\
  pinMode(GBUTTON, INPUT);\
  pinMode(LUM_SENSOR, INPUT);\
  DateTime dt = DateTime(__DATE__, __TIME__);\
  RTC.adjust(dt);\
  THPsensor.settings.commInterface = I2C_MODE;\
  THPsensor.settings.I2CAddress = 0x76;\
  THPsensor.settings.runMode = 3;\
  THPsensor.settings.tStandby = 0;\
  THPsensor.settings.filter = 0;\
  THPsensor.settings.tempOverSample = 1;\
  THPsensor.settings.pressOverSample = 1;\
  THPsensor.settings.humidOverSample = 1;\
  delay(10);\
  THPsensor.begin();\
  randomSeed(analogRead(0));\
  delay(5000);



String getTime() {
  String time;
  clock.getTime();
  time += String(clock.hour, DEC);
  time += F(":");
  time += String(clock.minute, DEC);
  time += F(":");
  time += String(clock.second, DEC);
  time += F(";");
  time += String(clock.month, DEC);
  time += F("/");
  time += String(clock.dayOfMonth, DEC);
  time += F("/");
  time += String(clock.year + 2000, DEC);
  time += F(";");
  time += String(clock.dayOfMonth);
  time += F("*");
  switch (clock.dayOfWeek) {
    case MON:
      time += String("MON");
      break;
    case TUE:
      time += String("TUE");
      break;
    case WED:
      time += String("WED");
      break;
    case THU:
      time += String("THU");
      break;
    case FRI:
      time += String("FRI");
      break;
    case SAT:
      time += String("SAT");
      break;
    case SUN:
      time += String("SUN");
      break;
  }
  return time;
}




void RESET() {
  LUMIN = 1;
  LUMIN_LOW = 255;
  LUMIN_HIGH = 768;
  TEMP_AIR = 1;
  MIN_TEMP_AIR = -10;
  MAX_TEMP_AIR = 60;
  HYGR = 1;
  HYGR_MINT = 0;
  HYGR_MAXT = 50;
  PRESSURE = 1;
  PRESSURE_MIN = 850;
  PRESSURE_MAX = 1080;
  LOG_INTERVAL = 10; // entre 2 mesures 10min
  FILE_MAX_SIZE = 4096; //4 Ko provoque son archivage
  TIMEOUT = 30; //secondes
}



String split_fn(String *chaine, int i) {
  String part;
  if (i == 1) {
    part = (*chaine).substring(0, (*chaine).indexOf("="));
  }
  else  {
    part = (*chaine).substring((*chaine).indexOf("=") + 1);
  }
  return part;
}
int verifINT(String *partie) {
  int chiffre = 30000;
  if (int nombre = (*partie).toInt()) {
    return nombre;
  }
  else if (*partie == "0") {
    return 0;
  }
  else {
    return chiffre;
  }
}





enum Para { UNDEF, Lumin, Lumin_LOW, Lumin_HIGH, Temp_AIR, Temp_AIR_MIN, Temp_AIR_MAX, Hygr, Hygr_MINT, Hygr_MAXT, Pressure, Pressure_MIN, Pressure_MAX, Log_INTERVAL, File_MAX_SIZE, Timeout, Version, Reset};

void mode_config(int previousTime) {
  displayColor(255, 215, 0);
  unsigned long currentTime = millis();
  while (currentTime - previousTime <= TMP_CONF) {
    currentTime = millis();
CheckPoint:
    Serial.print("\nquel parametre voulez vous modifier ? 'parametre=Val'\n");
    char tmp;
    delay(1000);
    String result;
    delay(10000);
    while (Serial.available() != 0) {
      tmp = Serial.read();
      result = result + tmp;
    }
    Serial.println(result);
    String part1 = split_fn(&result, 1);
    String part2 = split_fn(&result, 0);
    int x = 0;
    if (part1 == "LUMIN") {
      x = 1;
    }
    if (part1 == "LUMIN_LOW") {
      x = 2;
    }
    if (part1 == "LUMIN_HIGH") {
      x = 3;
    }
    if (part1 == "TEMP_AIR") {
      x = 4;
    }
    if (part1 == "TMIN_TEMP_AIR") {
      x = 5;
    }
    if (part1 == "MAX_TEMP_AIR") {
      x = 6;
    }
    if (part1 == "HYGR") {
      x = 7;
    }
    if (part1 == "HYGR_MINT") {
      x = 8;
    }
    if (part1 == "HYGR_MAXT") {
      x = 9;
    }
    if (part1 == "PRESSURE") {
      x = 10;
    }
    if (part1 == "PRESSURE_MIN") {
      x = 11;
    }
    if (part1 == "PRESSURE_MAX") {
      x = 12;
    }
    if (part1 == "LOG_INTERVAL") {
      x = 13;
    }
    if (part1 == "FILE_MAX_SIZE") {
      x = 14;
    }
    if (part1 == "TIMEOUT") {
      x = 15;
    }
    if (part1 == "VERSION") {
      x = 16;
    }
    if (part1 == "RESET") {
      x = 17;
    }


    switch (x) {

      case Lumin :
        {
          int p2;
          p2 = verifINT(&part2);
          if ( p2 == 0 || p2 == 1) {
            LUMIN = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Lumin_LOW :
        {
          int p2;
          p2 = verifINT(&part2);
          if (0 < p2 && p2 < 1023) {
            LUMIN_LOW = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Lumin_HIGH :
        {
          int p2;
          p2 = verifINT(&part2);
          if (0 < p2 && p2 < 1023) {
            LUMIN_HIGH = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Temp_AIR :
        {
          int p2;
          p2 = verifINT(&part2);
          if (p2 == 0 || p2 == 1) {
            TEMP_AIR = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Temp_AIR_MIN :
        {
          float p2;
          p2 = verifINT(&part2);
          if (-40 < p2 && p2 < 85) {
            MIN_TEMP_AIR = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Temp_AIR_MAX :
        {
          float p2;
          p2 = verifINT(&part2);
          if (-40 < p2 && p2 < 85) {
            MAX_TEMP_AIR = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Hygr :
        {
          int p2;
          p2 = verifINT(&part2);
          if (p2 == 0 || p2 == 1) {
            HYGR = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Hygr_MINT :
        {
          float p2;
          p2 = verifINT(&part2);
          Serial.println(p2);
          if (-40 < p2 && p2 < 85) {
            HYGR_MINT = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Hygr_MAXT:
        {
          float p2;
          p2 = verifINT(&part2);
          if (-40 < p2 && p2 < 85) {
            HYGR_MAXT = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Pressure :
        {
          int p2;
          p2 = verifINT(&part2);
          if (p2 == 0 || p2 == 1) {
            PRESSURE = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Pressure_MIN:
        {
          int p2;
          p2 = verifINT(&part2);
          if (300 < p2 && p2 < 1100) {
            PRESSURE_MIN = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Pressure_MAX:
        {
          int p2;
          p2 = verifINT(&part2);
          if (300 < p2 && p2 < 1100) {
            PRESSURE_MAX = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Log_INTERVAL:
        {
          int p2;
          p2 = verifINT(&part2);
          if (1 < p2 && p2 < 1000) {
            LOG_INTERVAL = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case File_MAX_SIZE:
        {
          int p2;
          p2 = verifINT(&part2);
          if (1024 < p2 && p2 < 16384) {
            FILE_MAX_SIZE = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Timeout:
        {
          int p2;
          p2 = verifINT(&part2);
          if (10 < p2 && p2 < 1000) {
            TIMEOUT = p2;
            Serial.println("changement effectue !");
          }
          else Serial.println("erreur de valeur de parametre");
        }
        break;
      case Version:
        {
          Serial.print("version du programme :...");
          Serial.println(2);
          Serial.print("numero de lot :");
          Serial.println(2);
        }
        break;
      case Reset:
        {
          RESET();
          Serial.println("reinitialisation de l’ensemble des parametres à leurs valeurs par defaut.");
        }
        break;
    }

    if (digitalRead(2) == 0) {
      break;
    }
    else {
      goto CheckPoint;
    }

  }
}


String verif(float data, short dataMin, short dataMax) {
  if (data < dataMax && data > dataMin) {
    return String(data);
  }
  return "NA";
}


void restartTimer() {
  rStart = 0;
  redTimer = 0;
  gStart = 0;
  greenTimer = 0;
}


void doRedButton() {
  if (digitalRead(RBUTTON) == LOW) {
    rStart = millis();
    redTimer = 0;
  } else {
    if (rStart != 0) {
      redTimer = millis() - rStart;
    }
  }
}
void doGreenButton() {
  if (digitalRead(GBUTTON) == LOW) {
    gStart = millis();
    greenTimer = 0;
  } else {
    if (gStart != 0) {
      greenTimer = millis() - gStart;
    }
  }
}
void displayColor(unsigned char red, unsigned char green, unsigned char blue) // the color generating function
{
  leds.setColorRGB(0, red, green, blue);
}


void setup() {
  Serial.begin(9600);
  RESET();
  SET_WEATHER_STATION
  if (digitalRead(RBUTTON) == LOW) {
    int test = millis();
    mode_config(test);
  }
  clock.begin();

  clock.fillByYMD(2013,1,19);
  clock.fillByHMS(15,28,30);
  clock.fillDayOfWeek(SAT);
  clock.setTime();
  
  pinMode(RBUTTON, INPUT_PULLUP);
  pinMode(GBUTTON, INPUT_PULLUP);
  digitalWrite(RBUTTON, HIGH);
  attachInterrupt(digitalPinToInterrupt(RBUTTON), doRedButton, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GBUTTON), doGreenButton, CHANGE);
}
void loop()
{
  
  if (redTimer >= DEFAULT_LONGPRESS_LEN && economyMode == 0) {
    restartTimer();
    maintenanceMode = !maintenanceMode;
    if (maintenanceMode) {
      detachInterrupt(digitalPinToInterrupt(GBUTTON));
    } else {
      attachInterrupt(digitalPinToInterrupt(GBUTTON), doGreenButton, CHANGE);
    }
  } else if (greenTimer >= DEFAULT_LONGPRESS_LEN) {
    restartTimer();
    economyMode = !economyMode;
    maintenanceMode = 0;
    if (economyMode) {
      detachInterrupt(digitalPinToInterrupt(GBUTTON));
    } else {
      attachInterrupt(digitalPinToInterrupt(GBUTTON), doGreenButton, CHANGE);
    }
  } else if (redTimer >= DEFAULT_LONGPRESS_LEN && maintenanceMode == 1 || redTimer >= DEFAULT_LONGPRESS_LEN && economyMode == 1 ) {
    restartTimer();
    maintenanceMode = 0;
    economyMode = 0;
  }

  delay(2000);


  if (maintenanceMode) {
    displayColor(237, 109, 0);
    if (LUMIN == 1) {
      Serial.print(analogRead(LUM_SENSOR));
      Serial.print(" LUX|");
    }
    if (TEMP_AIR == 1) { //Temperature de l'air exterieur
      Serial.print(random(-20, 50));
      Serial.print(" °C|");
    }
    if (PRESSURE == 1) {  //pression atmosphérique
      Serial.print(random(1, 2));
      Serial.print(" HPa|");
    }
    if (HYGR == 1) {  //hygrometrie
      Serial.print(THPsensor.readFloatHumidity());
      Serial.print(" %|");
    }

    Serial.print(random(-40, 126)); //température de l'eau
    Serial.print(" °C|");
    Serial.print(random(3, 61) / 10); //hygrométrie
    Serial.print(" L/min|");
    Serial.print(random(1, 131));
    Serial.println(" km/h|");      //Vitesse du vent

  } else if (economyMode) {
    displayColor(0, 0, 255);
    WRITE_IN_SD
  } else {
    displayColor(0, 255, 0);
    WRITE_IN_SD
  }

  if (economyMode) {
    delay(logInterval * 360000);
  } else {
    delay(logInterval * 180000);
  }
}