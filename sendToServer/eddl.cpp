
#include "eddl.h"

void _createLog() {
  File wFile;
  if (SPIFFS.exists("/log.dat")) {
  //  _DEBUG("File already exists!");
  ;
  } else {
    wFile = SPIFFS.open("/log.dat", "w+");
    if (!wFile) {
      //_DEBUG("Error! Failed to create file!");
    }
  }
  wFile.close();
}

String _readLog() {
  String buf;
  File rFile = SPIFFS.open("/log.dat", "r");
  while (!rFile) {
    rFile = SPIFFS.open("/log.dat", "r");
    delay(100);
  }
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
  int16_t utc = -3; //UTC -3:00 Brazil
  NTPClient timeClient(ntpUDP, "a.st1.ntp.br", utc * 3600, 60000);
  timeClient.begin();
  timeClient.update();
  return timeClient.getEpochTime();
}

String _sendToServer(unsigned long _baseMillis, unsigned long _initialTime) {
  String buf;
  
  unsigned long _actualTime = (millis() - _baseMillis)/1000 + _initialTime;
  
  unsigned long hours = (_actualTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (_actualTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = _actualTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  //_DEBUG(hoursStr + ":" + minuteStr + ":" + secondStr);
  const int http_port = 80;
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  
  root["time"] = hoursStr + ":" + minuteStr + ":" + secondStr ;
  root["isConnected"] = 1;
  root["id"] = id;
  
  root.printTo(buf);
  //_DEBUG(buf);
  return buf;
}


