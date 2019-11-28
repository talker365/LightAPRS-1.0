#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <SoftwareSerial.h>
//#include <TinyGPS++.h>      //https://github.com/mikalhart/TinyGPSPlus
#include <TinyGPS.h>
#include <LowPower.h>       //https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <Adafruit_BMP085.h>//https://github.com/adafruit/Adafruit-BMP085-Library
#include <avr/wdt.h>

#define GpsRx       2
#define GpsTx       3
#define RfPDPin     4
#define GpsVccPin   7
#define RfPwrHLPin  6
#define RfPttPin    5
#define BattPin     A2
#define PIN_DRA_RX  8
#define PIN_DRA_TX  9

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

#define GpsON         digitalWrite(GpsVccPin, LOW)//PNP
#define GpsOFF        digitalWrite(GpsVccPin, HIGH)
#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
//#define RfPttON       digitalWrite(RfPttPin, HIGH)//NPN
//#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define RfPttON       pinMode(RfPttPin, OUTPUT);digitalWrite(RfPttPin, HIGH)//NPN
#define RfPttOFF      pinMode(RfPttPin, OUTPUT);digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(10,INPUT);pinMode(11,INPUT);pinMode(12,INPUT);pinMode(13,INPUT)
#define AprsPinOutput pinMode(10,OUTPUT);pinMode(11,OUTPUT);pinMode(12,OUTPUT);pinMode(13,OUTPUT)

#define DEVMODE // Development mode. Uncomment to enable for debugging.

//****************************************************************************
char  CallSign[7]="WD4VA"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
uint8_t   CallNumber=9; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char  Symbol='>'; // '/>' for car, '/k' for truck, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool alternateSymbolTable = false ; //false = '/' , true = '\'

char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

char comment[50] = "http://www.lightaprs.com"; // Max 50 char
char StatusMessage[50] = "LightAPRS"; 
//*****************************************************************************


unsigned int   BeaconWait=60;  //seconds sleep for next beacon (TX).
//unsigned int   BattWait=60;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
unsigned int   BattWait=2;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
//float BattMin=4.5;        // min Volts to wake up.
float BattMin=4.0;        // min Volts to wake up.
float DraHighVolt=8.0;    // max Volts for radio module (DRA818V) to transmit (TX) 1 Watt, above this transmit 0.5 Watt. Do not increase this value to avoid overheating.
float GpsMinVolt=4.0; //min Volts for GPS to wake up. (important if power source is solar panel) 

boolean aliveStatus = true; //for tx status message on first wake-up just once.

//do not change WIDE path settings below if you don't know what you are doing :) 
byte  Wide1=1; // 1 for WIDE1-1 path
byte  Wide2=1; // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.  
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.  
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.  
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles. 
 */
int pathSize=2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = true; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet)) 

boolean GpsFirstFix=false;

static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1;

//TinyGPSPlus gps;
TinyGPS gps;
Adafruit_BMP085 bmp;
String serialCommand;

SoftwareSerial gpsSerial(GpsRx, GpsTx); // RX, TX

void setup() {
  wdt_enable(WDTO_8S);
  //analogReference(INTERNAL2V56);
  analogReference(DEFAULT);
  pinMode(RfPDPin, OUTPUT);
  pinMode(GpsVccPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX,INPUT);

  RfOFF;
  GpsON;
  RfPwrLow;
  RfPttOFF;
  delay(1000);
  
  Serial.begin(115200);
  gpsSerial.begin(9600);
  #if defined(DEVMODE)
    Serial.println(F("Start"));
  #endif
      
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign,CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setMessageDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_useAlternateSymbolTable(alternateSymbolTable); 
  APRS_setSymbol(Symbol);
  APRS_setPathSize(pathSize);
  AprsPinInput;
  
  configDra818(Frequency);
  
  bmp.begin();
 
  #if defined(DEVMODE)
    Serial.println(F("()setup"));
  #endif

}

void loop() {
  #if defined(DEVMODE)
    Serial.println(F("loop()"));
  #endif

  wdt_reset();

  Serial.println(readBatt());
  
  if (readBatt() > BattMin) {
  
  
  if(aliveStatus){

      //send status tx on startup once (before gps fix)
      
      #if defined(DEVMODE)
        Serial.println(F("Sending"));
      #endif
      sendStatus();
      #if defined(DEVMODE)
        Serial.println(F("Sent"));
      #endif
      
      aliveStatus = false;
      
   } else {
      #if defined(DEVMODE)
        Serial.println(F("aliveStatus = false"));
      #endif
   }

    Serial.println(F("updateGpsData(5000);"));
    updateGpsData(5000);
//    updateGpsData(1000);
    gpsDebug();

  
    updatePosition();
    updateTelemetry();
    
    GpsFirstFix=true;

    APRS_setPathSize(pathSize);
    
    //send status message every 60 minutes
    sendStatus();       
    sendLocation();

    freeMem();
    Serial.flush();
    sleepSeconds(BeaconWait);

  } else {

    sleepSeconds(BattWait);
    
  }
  
}

void aprs_msg_callback(struct AX25Msg *msg) {
  //do not remove this function, necessary for LibAPRS
}

void sleepSeconds(int sec) {  
  RfOFF;
  RfPttOFF;
  Serial.flush();
  wdt_disable();
  for (int i = 0; i < sec; i++) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);   
  }
   wdt_enable(WDTO_8S);
}



byte configDra818(char *freq)
{
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  RfON;
  char ack[3];
  int n;

  
  
  delay(500);
  char cmd[50];

  
  Serial.print(F("Clearing Serial Buffer: "));
  ack[0] = 0;
  ack[1] = 0;
  ack[2] = 0;
  while (Serial_dra.available() > 0) {
    ack[0] = ack[1];
    ack[1] = ack[2];
    ack[2] = Serial_dra.read();
    Serial.print(ack[2]);
  }
  Serial.println();
  
  delay(500);
  Serial_dra.println("AT+DMOCONNECT");
  Serial.println(F("AT+DMOCONNECT"));
  delay(500);
  Serial.print(F("Response: "));
  ack[0] = 0;
  ack[1] = 0;
  ack[2] = 0;
  while (Serial_dra.available() > 0) {
    ack[0] = ack[1];
    ack[1] = ack[2];
    ack[2] = Serial_dra.read();
    Serial.print(ack[2]);
  }
  Serial.println();
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial.println(cmd);
  Serial_dra.println(cmd);
  delay(500);
  Serial.print("Response: ");
  ack[0] = 0;
  ack[1] = 0;
  ack[2] = 0;
  while (ack[2] != 0x0a && ack[1] != 0x0d && ack[0] != 0x30)
  {
    if (Serial_dra.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial_dra.read();
      Serial.print(ack[2]);
      //Serial.print(ack[2], HEX);
      //Serial.print(",");
    }
  }
  Serial.println();
  Serial_dra.end();
  RfOFF;
  pinMode(PIN_DRA_TX,INPUT);
#if defined(DEVMODE)
  if (ack[0] == 0x30) {
    Serial.println(F("Frequency updated...")); 
  } else {
    Serial.println(F("Frequency update error!"));
    Serial.print("ack[0]: ");
    Serial.println(ack[0]);
    Serial.print("ack[1]: ");
    Serial.println(ack[1]);
    Serial.print("ack[2]: ");
    Serial.println(ack[2]);
  }
#endif
  return (ack[0] == 0x30) ? 1 : 0;
}

void updatePosition() {

  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);

  
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;

//  double d_lat = gps.location.lat();
  double d_lat = flat;
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
//  double d_lon = gps.location.lng();
  double d_lon = flon;
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
}


void updateTelemetry() {
 
  telemetry_buff[0] = ' ';
  telemetry_buff[3] += '/';
  telemetry_buff[7] = '/';
  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'x';
  telemetry_buff[22] = 'C';
  telemetry_buff[23] = ' '; float tempC = bmp.readTemperature();//-21.4;//
  dtostrf(tempC, 6, 2, telemetry_buff + 24);
  telemetry_buff[30] = 'C';
  telemetry_buff[31] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 32);
  telemetry_buff[39] = 'h';
  telemetry_buff[40] = 'P';
  telemetry_buff[41] = 'a';
  telemetry_buff[42] = ' ';
  dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
  telemetry_buff[48] = 'V';
  telemetry_buff[49] = ' ';
  telemetry_buff[50] = '0';
  telemetry_buff[51] = '0';
  telemetry_buff[52] = 'S';
  telemetry_buff[53] = ' ';
  sprintf(telemetry_buff + 54, "%s", comment);
  

#if defined(DEVMODE)
  Serial.println(telemetry_buff);
#endif

}

void sendLocation() {

  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): Begin - setting RF Power level"));
  #endif
  if (readBatt() < DraHighVolt) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt

  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): Getting time..."));
  #endif

  char timestamp_buff[7];

  timestamp_buff[6] = 'h';


  
  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): Setting pins"));
  #endif
  
  AprsPinOutput;
  RfON;
  delay(500);
  Serial.println(F("sendLocation(): RfPttON;"));
  RfPttON;
  delay(2000);

  
  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): APRS_sendLocWtTmStmp()"));
  #endif

  APRS_sendLocWtTmStmp(telemetry_buff, strlen(telemetry_buff), timestamp_buff); //beacon with timestamp
  delay(50);
  while(digitalRead(1)){;}//LibAprs TX Led pin PB1
  delay(50);

  
  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): Disabling pins"));
  #endif

  Serial.println(F("sendLocation(): RfPttOFF;"));
  RfPttOFF;
  RfOFF;
  AprsPinInput;
  #if defined(DEVMODE)
    Serial.println(F("sendLocation(): Sent with comment"));
  #endif

  TxCount++;
}





void sendStatus() {
  wdt_reset();
  if (readBatt() < DraHighVolt) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt

  AprsPinOutput;
  RfON;
  delay(1000);
  Serial.println(F("sendStatus(): RfPttON;"));
  RfPttON;
  delay(2000);
  Serial.println(F("-1"));
 // APRS_sendStatus(StatusMessage, strlen(StatusMessage));
  delay(50);
//  while(digitalRead(1)){;}//LibAprs TX Led pin PB1
  delay(50);
  Serial.println(F("sendStatus(): RfPttOFF;"));
  RfPttOFF;
  RfOFF;
  AprsPinInput; 
  TxCount++;

}






static void updateGpsData(int ms)
{
  while (!gpsSerial) {
    Serial.println(F("Waiting for GPS Serial"));
    delayMicroseconds(1000); // wait for serial port to connect.
  }
    unsigned long start = millis();
    unsigned long bekle=0;
    do
    {
      while (gpsSerial.available()>0) {
        char c;
        c=gpsSerial.read();
        gps.encode(c);
        bekle= millis();
        Serial.println(F("reading GPS"));
      }
      if (bekle!=0 && bekle+10<millis())break;
    } while (millis() - start < ms);

}

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do { 
    value =analogRead(BattPin);
    delay(5);
    value =analogRead(BattPin);
    value=value-8;
    value = (value * 2.56) / 1024.0;
    value = value / (R2/(R1+R2));
  } while (value > 16.0);
  return value ;
}

void freeMem() {
#if defined(DEVMODE)
  Serial.print(F("Free RAM: ")); Serial.print(freeMemory()); Serial.println(F(" byte"));
#endif
}

void gpsDebug() {
#if defined(DEVMODE)
  Serial.println();
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  Serial.print("LAT=");
  Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial.print(" LON=");
  Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  Serial.print(" SAT=");
  Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  Serial.print(" PREC=");
  Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  Serial.println();

#endif
}


static void printFloat(float val, bool valid, int len, int prec)
{
#if defined(DEVMODE)
  if (!valid)
  {
    while (len-- > 1)
      Serial.print(F("*"));
    Serial.print(F(" "));
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(F(" "));
  }
#endif
}

static void printInt(unsigned long val, bool valid, int len)
{
#if defined(DEVMODE)
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
#endif
}


static void printStr(const char *str, int len)
{
#if defined(DEVMODE)
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
#endif
}


