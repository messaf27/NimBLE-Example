
#include "WiFiConfigurator.h"
#include "esp32-hal-log.h"

IPAddress localIP;
IPAddress localGateway;
IPAddress localMask;

String idAPName;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

/* 
Истинный идентификатор чипа ESP32 — это, по сути, его MAC-адрес.
Этот скетч предоставляет альтернативный идентификатор чипа, 
который соответствует вывод функции ESP.getChipId() на ESP8266
(т.е. 32-битное целое число, соответствующее последним 3 байтам
MAC-адрес. Это менее уникально, чем идентификатор чипа MAC-адреса, 
но полезен, когда вам нужен идентификатор, который может быть не 
более чем 32-битным целым числом 
*/
static inline uint32_t GenerateChipID(void)
{
    uint32_t chipId = 0;

    for (int i = 0; i < 17; i = i + 8)
    {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }

    return chipId;
}

bool WiFiConfig_Init(void)
{
    if (!WiFi.mode(WIFI_AP))
    {
        log_e("Fault WiFi set AP Mode!");
        return false;
    }

    idAPName = WiFiConfig_AP_DEFAULT_NAME + String(GenerateChipID());
    log_i("Generate SoftAP Name: %s", idAPName.c_str());

    if (!WiFi.softAP(idAPName.c_str()))
    {
        log_e("Fault WiFi set AP Name!");
        return false;
    }

    localIP.fromString(WiFiConfig_STA_DEFAULT_IP);
    localGateway.fromString(WiFiConfig_STA_DEFAULT_GW);
    localMask.fromString(WiFiConfig_STA_DEFAULT_MSK);

    if (!WiFi.softAPConfig(localIP, localGateway, localMask))
    {
        log_e("STA Failed to configure!!!");
        return false;
    }

    log_i("Create SoftAP SSID: %s, IP: %s", WiFi.softAPSSID().c_str(), WiFi.softAPIP().toString().c_str());
    // log_i("Create SoftAP? SSID: %s, IP: %s", idAPName, WiFi.softAPIP().toString().c_str());

    return true;
}