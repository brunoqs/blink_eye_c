
#include "edal.h"

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

void _sendToServer(unsigned long _baseMillis, unsigned long _initialTime) {
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


