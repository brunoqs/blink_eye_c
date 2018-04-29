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
#include "edal.h"

#define inp 5
#define id 1

unsigned long _initialTime;
unsigned long _baseMillis;

void interrupt();

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
  if (_readLog().length() == 0) _sendToServer(_baseMillis, _initialTime);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
}

void interrupt() {
  Serial.print("interrupt!");
  if (WiFi.status() == WL_CONNECTED)_sendToServer(_baseMillis, _initialTime);
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
