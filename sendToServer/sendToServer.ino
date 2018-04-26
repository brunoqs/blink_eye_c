/*
 * CODE:
 * Connect to a public network, wait for a HIGH pulse from DE10-nano on an interrupt pin and send it to a extern server
 * The system will check if some data wasn't sent since last startup and try to resend  
 * 
 * IDEAS:
 * https://gist.github.com/dogrocker/f998dde4dbac923c47c1 (Webserver running on AP mode to set ssid and pass to STA mode)
 * 
 */
#include "ESP8266WiFi.h"
#include "FS.h"
 
#define led 4
#define inp 5

#ifdef ESP8266
  extern "C" {
    bool system_update_cpu_freq(int);
  }
#endif

void interrupt();

void _createLog(){
  File wFile;
  if(SPIFFS.exists("/log.dat")){
    Serial.println("File already exists!");
  } else {
    Serial.println("Creating file...");
    wFile = SPIFFS.open("/log.dat","w+");
    if(!wFile){
      Serial.println("Error! Failed to create file!");
    } else {
      Serial.println("File has sucessfully created!");
    }
  }
  wFile.close();
}
 
void _writeLog(String msg) { 
  File rFile = SPIFFS.open("/log.dat","a+");
  if(!rFile){
    Serial.println("Error! Failed to open file!");
  } else {
    rFile.println("Log: " + msg);
    Serial.println(msg);
  }
  rFile.close();
}
 
String _readLog() {
  String buf;
  File rFile = SPIFFS.open("/log.dat","r");
  Serial.println("Searching file...");
  while(!rFile){
    rFile = SPIFFS.open("/log.dat","r");
    delay(100);
  }
  Serial.println("File was found!");
  Serial.println("Reading file...");
  while(rFile.available()) {
    String line = rFile.readStringUntil('\n');
    buf += line;
    buf += "<br/>";
    yield();
  }
  rFile.close();
  return buf;
}

void _sendToServer(){
;
}

void setup() {
  #ifdef ESP8266
    system_update_cpu_freq(160);
  #endif
  Serial.begin(115200);
  Serial.println("OIOIOI");
  pinMode(led,OUTPUT);
  pinMode(inp,INPUT);
  attachInterrupt(digitalPinToInterrupt(inp), interrupt, RISING);
  Serial.println("Initializing file system...");
  SPIFFS.begin();
  _createLog();
  if(_readLog().length() == 0) _sendToServer();
  // Check if already exists data in SPIFSS and send to server
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
}

void interrupt(){
  Serial.print("interrupt!");
  if (WiFi.status()==WL_CONNECTED)_sendToServer();
  else{
    _writeLog(String(millis()));
    _scanWifi();
  }
}

void _connect(char* ssid){
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  int time_init = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (millis() - time_init > 10000)return;
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void _scanWifi(){
  uint8_t n = WiFi.scanNetworks();
  for(uint8_t i = 0; i < n; i++) {
    Serial.print(WiFi.SSID(i));
    Serial.print("  ");
    Serial.println(WiFi.RSSI(i));
    if(WiFi.encryptionType(i) == ENC_TYPE_NONE){
      char* buf;
      WiFi.SSID(i).toCharArray(buf,WiFi.SSID(i).length());
      _connect(buf);
    }
  }
}

void loop() {
  _readLog();
  Serial.println("Good afternoon");
  delay(1000);  
}
