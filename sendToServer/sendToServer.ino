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
#include <WiFiClient.h>

#define inp 5
#define id 1

#ifdef ESP8266
extern "C" {
  bool system_update_cpu_freq(int);
}
#define _DEBUG(x) Serial.println(x)
#endif

unsigned long _initialTime;
unsigned long _baseMillis;

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

unsigned long _getNTP() {
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
    return timeClient.getEpochTime();
    //_DEBUG(timeClient.getFormattedTime());
  }
  timeClient.end();
  return 0;
}

void _sendToServer() {
  unsigned long _actualTime = (millis() - _baseMillis)/1000 + _initialTime;
  _DEBUG(_actualTime);
  
  unsigned long hours = (_actualTime % 86400L) / 3600 - 1;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (_actualTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = _actualTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);
  
  _DEBUG(hoursStr + ":" + minuteStr + ":" + secondStr);
  const char http_site[] = "http://localhost/";
  const int http_port = 80;

  WiFiClient client;
  IPAddress server(192,168,0,108);
  
  if ( !client.connect(server, http_port) ) {
    Serial.println("Falha na conexao com o site ");
    return;
  }
  
  String buf = "Test";
  String param = "?dt=" + String(buf);
  client.println("GET /teste.php" + param + " HTTP/1.1");
  client.println("Host: ");
  client.println(http_site);
  client.println("Connection: close");
  client.println();
  client.println();
 
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }    
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
  _initialTime = _getNTP();
  _baseMillis = millis();
  SPIFFS.begin();
  //SPIFFS.format();
  _createLog();
  // Check if already exists data in SPIFSS and send to server
  if (_readLog().length() == 0) _sendToServer();

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
    WiFi.begin("Rep_ABate_Caverma","tilasesujos69");
    _DEBUG(WiFi.localIP());  
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
  _DEBUG(WiFi.localIP());
}

void _scanWifi(uint8_t mode) {
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
  //noInterrupts();
  //_readLog();
  //delay(100);
  //interrupts();
  //delay(2000);
}
