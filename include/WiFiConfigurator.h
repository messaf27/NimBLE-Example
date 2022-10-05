#ifndef _WIFICONFIGURATOR_H_
#define _WIFICONFIGURATOR_H_

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

#define WiFiConfig_AP_DEFAULT_NAME     "LegoWeDo2-Joystic-ID:"
#define WiFiConfig_STA_DEFAULT_IP       "192.168.0.10"
#define WiFiConfig_STA_DEFAULT_GW       "192.168.0.1"
#define WiFiConfig_STA_DEFAULT_MSK      "255.255.255.0"


bool WiFiConfig_Init(void);

#endif /*_WIFICONFIGURATOR_H_*/