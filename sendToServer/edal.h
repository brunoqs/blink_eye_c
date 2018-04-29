#include "FS.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

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
void _sendToServer(unsigned long, unsigned long);
