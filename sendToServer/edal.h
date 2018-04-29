#include "FS.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

#define inp 5
#define id 1
#define _DEBUG(x) Serial.println(x)

#ifdef ESP8266
extern "C" {
  bool system_update_cpu_freq(int);
}

#endif
void _createLog();
void _writeLog(String, bool);
String _readLog();
unsigned long _getNTP();
String  _sendToServer(unsigned long, unsigned long);
