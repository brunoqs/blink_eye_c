/*
   CODE:
   Connect to a public network, wait for a HIGH pulse from DE10-nano on an interrupt pin and send it to a extern server
   The system will check if some data wasn't sent since last startup and try to resend


   ISSUES
   1. ESP12E error encountered: the board needs more than 100mA to use SPIFFS, otherwise the watchdog timer will reset it
   2. STACK was overflowing because of many functions calls

   IDEAS:
   https://gist.github.com/dogrocker/f998dde4dbac923c47c1 (Webserver running on AP mode to set ssid and pass to STA mode)

*/
#include "ESP8266WiFi.h"
#include "FS.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#define inp 5
#define id 1

#ifdef ESP8266
extern "C" {
  bool system_update_cpu_freq(int);
}
#define _DEBUG(x) Serial.println(x)
#endif



void interrupt();

void _createLog() {
  File wFile;
  if (SPIFFS.exists("/log.dat")) {
    _DEBUG("File already exists!");
  } else {
    _DEBUG("Creating file...");
    wFile = SPIFFS.open("/log.dat", "w+");
    if (!wFile) {
      _DEBUG("Error! Failed to create file!");
    } else {
      _DEBUG("File has sucessfully created!");
    }
  }
  wFile.close();
}

void _writeLog(String time, bool isConnected = true) {
  //StaticJsonBuffer<1> jsonBuffer;
  //DynamicJsonBuffer jsonBuffer(150);
  //JsonObject& root = jsonBuffer.createObject();
  //root["time"] = time;
  //root["isConnected"] = (isConnected) ? 1 : 0;
  //String buf;
  //root.printTo(Serial);
  //root.printTo(buf);
  delay(50);
  //_DEBUG(buf);
  File rFile = SPIFFS.open("/log.dat", "a+");
  if (!rFile) {
    _DEBUG(F("Error! Failed to open file!"));
  } else {
    //rFile.println(buf);
  }
  rFile.close();
}

String _readLog() {
  String buf;
  File rFile = SPIFFS.open("/log.dat", "r");
  //_DEBUG("Searching file...");
  while (!rFile) {
    rFile = SPIFFS.open("/log.dat", "r");
    delay(100);
  }
  //_DEBUG(F("File was found!"));
  //_DEBUG(F("Reading file..."));
  while (rFile.available()) {
    String line = rFile.readStringUntil('\n');
    buf += line;
    buf += "<br/>";
    yield();
  }
  _DEBUG(buf);
  rFile.close();
  return buf;
}

String _ntpInit() {
  WiFiUDP ntpUDP;
  int16_t utc = -2; //UTC -3:00 Brazil
  uint32_t currentMillis = 0;
  uint32_t previousMillis = 0;
  NTPClient timeClient(ntpUDP, "a.st1.ntp.br", utc * 3600, 60000);
  timeClient.begin();
  timeClient.update();
  currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    printf("Time Epoch: %d: ", timeClient.getEpochTime());
    _DEBUG(timeClient.getFormattedTime());
  }
  return timeClient.getFormattedTime();
}

void _sendToServer() {

  ;
}

void setup() {
#ifdef ESP8266
  system_update_cpu_freq(160);
#endif
  Serial.begin(115200);
  Serial.setDebugOutput(1);

  pinMode(inp, INPUT);
  attachInterrupt(digitalPinToInterrupt(inp), interrupt, RISING);

  _DEBUG("Initializing file system...");
  SPIFFS.begin();
  //SPIFFS.format();
  _createLog();
  if (_readLog().length() == 0) _sendToServer();
  // Check if already exists data in SPIFSS and send to server

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
}

void interrupt() {
  Serial.print("interrupt!");
  if (WiFi.status() == WL_CONNECTED)_sendToServer();
  else {
    //_writeLog(String(millis()), false);
    StaticJsonBuffer<500> jsonBuffer;
    //DynamicJsonBuffer jsonBuffer(150);
    JsonObject& root = jsonBuffer.createObject();
    root["time"] = String(millis());
    root["isConnected"] = 0;
    root["id"] = id;
    String buf;
    root.printTo(buf);
    _DEBUG(buf);
    File rFile = SPIFFS.open("/log.dat","a+");
    if(!rFile){
      _DEBUG(F("Error! Failed to open file!"));
    } else {
      rFile.println(buf);
    }
    rFile.close();
    //_readLog();
    //_scanWifi();
    
  }
}

void _connect(char* ssid) {
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  int time_init = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    _DEBUG(".");
    if (millis() - time_init > 10000)return;
  }
  /*_DEBUG("");
    _DEBUG("WiFi connected");
    _DEBUG("IP address: ");*/
  _DEBUG(WiFi.localIP());
}

void _scanWifi() {
  uint8_t n = WiFi.scanNetworks();
  for (uint8_t i = 0; i < n; i++) {
    /*Serial.print(WiFi.SSID(i));
      Serial.print("  ");
      _DEBUG(WiFi.RSSI(i));*/
    if (WiFi.encryptionType(i) == ENC_TYPE_NONE) {
      char* buf;
      WiFi.SSID(i).toCharArray(buf, WiFi.SSID(i).length());
      _connect(buf);
    }
  }
}

void loop() {
  noInterrupts();
  //_readLog();
  delay(100);
  interrupts();
  delay(2000);
}
