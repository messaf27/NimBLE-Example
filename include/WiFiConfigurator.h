#ifndef _WIFICONFIGURATOR_H_
#define _WIFICONFIGURATOR_H_

#include <Arduino.h>
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncTCP.h>
// #include "SPIFFS.h"
#include "lpf2_smart_hub.h"



typedef struct
{
    int drvUpMaxSpeed;
    int drvUpMinSpeed;
    int drvDownMaxSpeed;
    int drvDownMinSpeed;
    int sensDetectValue;
}
web_conf_t;

#define WiFiConfig_AP_DEFAULT_NAME      "WeDo2-Controller-ID:"
#define WiFiConfig_STA_DEFAULT_IP       "192.168.0.10"
#define WiFiConfig_STA_DEFAULT_GW       "192.168.0.1"
#define WiFiConfig_STA_DEFAULT_MSK      "255.255.255.0"


bool WiFiConfig_Init(eeConstVal_t *eeConf);

#endif /*_WIFICONFIGURATOR_H_*/