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

WiFiClient client;
IPAddress server(192,168,0,105);
const int http_port = 80;

unsigned long _initialTime;
unsigned long _baseMillis;

String json = "0";

void interrupt();

void setup() {
#ifdef ESP8266
  system_update_cpu_freq(160);
#endif
  Serial.begin(115200);
  Serial.setDebugOutput(1);

  WiFi.mode(WIFI_STA);
  WiFi.begin("Rep_ABate_Caverma","tilasesujos69");
  delay(50);
  _DEBUG(WiFi.localIP());  
  
  pinMode(inp, INPUT);
  attachInterrupt(digitalPinToInterrupt(inp), interrupt, RISING);

  _DEBUG("Initializing file system...");
  _initialTime = _getNTP();
  _baseMillis = millis();
  SPIFFS.begin();
  //SPIFFS.format();
  _createLog();
  // Check if already exists data in SPIFSS and send to server
  if (_readLog().length() == 0); // _sendToServer(_baseMillis, _initialTime);
}

void interrupt() {
  Serial.print("interrupt!");
  if (WiFi.status() == WL_CONNECTED){
  json = _sendToServer(_baseMillis, _initialTime);
  }
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

void loop() {
  if(json != "0"){
    if ( !client.connect(server, http_port) ){
      _DEBUG("Falha na conexao com o site ");
      return;
    }
    String url = "GET /edal/teste.php?data=" + json + " HTTP/1.1";
    client.println(url);
    client.println("Host: 192.168.0.105");
    client.println("Connection: close");
    client.println();
    
    while(client.available()){
      String line = client.readStringUntil('\r');
      _DEBUG(line);
    }     
    json = "0";
    _readLog();
  }
  
  //_readLog();
  //  delay(1000);
}
