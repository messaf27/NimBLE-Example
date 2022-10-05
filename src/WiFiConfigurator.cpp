
#include "WiFiConfigurator.h"
#include "esp32-hal-log.h"
#include <GyverPortal.h>

IPAddress localIP;
IPAddress localGateway;
IPAddress localMask;

String idAPName;
// Create AsyncWebServer object on port 80
// AsyncWebServer server(80);

GyverPortal webPortal;
TaskHandle_t WiFiTask; //Определяем задачи

// Search for parameter in HTTP POST request
const char *PARAM_INPUT_1 = "minspeed";
const char *PARAM_INPUT_2 = "maxspeed";

web_conf_t webConfig = {};

//Конструктор WEB страницы
void build(GyverPortal &portal)
{
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  GP.TITLE("Lego WeDo 2.0 Controller");
  GP.HR();

  // GP.NAV_TABS_LINKS("/,/sett,/upd", "Опции,Обновление", GP_BLUE);
  GP.NAV_TABS_LINKS("/,/upd", "Опции,Обновление", GP_BLUE);

  // GP.FORM_BEGIN("/");

  if (portal.uri("/upd"))
  {
    GP_MAKE_BOX(GP_CENTER,GP.LABEL("Обновление ПО");); GP.BREAK();
    GP.BUTTON("btnUpdFw", "Обновить", "", GP_BLUE, "");
    // главная страница, корень, "/"
  }
  else
  {
    GP_MAKE_BOX(GP_CENTER; GP.LABEL("Настройки двигателей");); GP.BREAK();
    GP_MAKE_BOX(GP_CENTER; GP.LABEL("Max speed"); GP.SLIDER("sldMinSpd", webConfig.drvMinSpeed, 50, 100, 1, 0, GP_BLUE);); GP.BREAK();
    GP_MAKE_BOX(GP_CENTER; GP.LABEL("Min speed"); GP.SLIDER("sldMaxSpd", webConfig.drvMaxSpeed, 50, 100, 1, 0, GP_BLUE);); GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("Настройки датчика препядствий");); GP.BREAK();
    GP_MAKE_BOX(GP_CENTER; GP.LABEL("Расстояние"); GP.SLIDER("sldSens", webConfig.sensDetectValue, 1, 10, 1, 0, GP_BLUE););
    GP.BUTTON("btnSave", "Сохранить", "", GP_BLUE, "");
  }

  GP.BUILD_END();
}

//Обработчик событий
void action(GyverPortal &portal)
{
  if (portal.click())
  {
    if (portal.click("sld"))
    {
      // LedPause = portal.getInt("sld");
    }
  }
}

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

static void WiFiConfig_Task(void *pvParameters)
{
  GyverPortal *portal = (GyverPortal *)pvParameters;

  log_i("Run WiFiConfig_Task (CoreID: %d)", xPortGetCoreID());

  if (!WiFi.mode(WIFI_AP))
    log_e("Fault WiFi set AP Mode!");

  idAPName = WiFiConfig_AP_DEFAULT_NAME + String(GenerateChipID());

  if (!WiFi.softAP(idAPName.c_str()))
    log_e("Fault WiFi set AP Name!");

  localIP.fromString(WiFiConfig_STA_DEFAULT_IP);
  localGateway.fromString(WiFiConfig_STA_DEFAULT_GW);
  localMask.fromString(WiFiConfig_STA_DEFAULT_MSK);

  if (!WiFi.softAPConfig(localIP, localGateway, localMask))
    log_e("STA Failed to configure!!!");

  log_i("Create SoftAP SSID: %s, IP: %s", WiFi.softAPSSID().c_str(), WiFi.softAPIP().toString().c_str());

  portal->attachBuild(build);
  portal->attach(action);
  portal->start();

  for (;;)
  {
    while (portal->tick())
      ;

    vTaskDelay(10);
  }
}

bool WiFiConfig_Init(void)
{
  webConfig.drvMinSpeed = 65;
  webConfig.drvMaxSpeed = 100;
  webConfig.sensDetectValue = 8;

  BaseType_t createResult = xTaskCreatePinnedToCore(WiFiConfig_Task, "Task_WiFiConfig", 8192, (void *)&webPortal, 4, &WiFiTask, ARDUINO_RUNNING_CORE);
  if (createResult == pdFAIL)
    log_e("Fail create Task_WiFiConfig!");

  return true;
}