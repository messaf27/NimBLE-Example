
#include "WiFiConfigurator.h"
#include "esp32-hal-log.h"


// #include <LittleFS.h>     // !!! подключить библиотеку файловой системы (до #include GyverPortal) !!!
#include <SPIFFS.h>
#include <GyverPortal.h>

IPAddress localIP;
IPAddress localGateway;
IPAddress localMask;

String idAPName;
// Create AsyncWebServer object on port 80
// AsyncWebServer server(80);

// GyverPortal webPortal;
GyverPortal webPortal(&SPIFFS); // передать ссылку на fs (SPIFFS/LittleFS)

TaskHandle_t WiFiTask; //Определяем задачи

web_conf_t webConfig = {};
eeConstVal_t *eeConf;

#include <math.h>
// Private functions
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
// Private functions

//Конструктор WEB страницы
void build(GyverPortal &portal)
{
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  GP.TITLE("Lego WeDo 2.0 Controller");
  GP.HR();

  // GP.NAV_TABS_LINKS("/,/sett,/upd", "Опции,Обновление", GP_BLUE);
  GP.NAV_TABS_LINKS("/,/ota_update", "Опции,Обновление", GP_BLUE); 
  // GP.NAV_TABS_LINKS("/,/ota_update", "Опции,Обновление", GP_BLUE);

  // GP.FORM_BEGIN("/");

  if (portal.uri("/ota_update"))
  {
    GP.OTA_FIRMWARE("Обновление ПО");
    // GP.OTA_FILESYSTEM();

    GP_MAKE_BOX(GP_CENTER, GP.LABEL("Текущая версия ПО:"););
    GP.BREAK();
    GP.LABEL("v1.0");
    // GP.BUTTON("btnUpdFw", "Обновить", "", GP_BLUE, "");
    // главная страница, корень, "/"
  }
  else
  {
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("Настройки двигателей"););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("UP:Max speed"); GP.SLIDER("sldMaxSpdUp", webConfig.drvUpMaxSpeed, 50, 100, 1, 0, GP_BLUE););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("UP:Min speed"); GP.SLIDER("sldMinSpdUp", webConfig.drvUpMinSpeed, 50, 100, 1, 0, GP_BLUE););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("DW:Max speed"); GP.SLIDER("sldMaxSpdDwn", webConfig.drvDownMaxSpeed, 50, 100, 1, 0, GP_BLUE););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("DW:Min speed"); GP.SLIDER("sldMinSpdDwn", webConfig.drvDownMinSpeed, 50, 100, 1, 0, GP_BLUE););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("Настройки датчика препядствий"););
    GP.BREAK();
    GP_MAKE_BOX(GP_CENTER, GP.LABEL("Расстояние"); GP.SLIDER("sldSens", webConfig.sensDetectValue, 1, 10, 1, 0, GP_BLUE););
    GP.BUTTON("btnSave", "Сохранить", "", GP_BLUE, "");
  }

  GP.BUILD_END();
}

//Обработчик событий
void action(GyverPortal &portal)
{
  if (portal.click())
  {
    if (portal.click("sldMaxSpdUp"))
    {
      webConfig.drvUpMaxSpeed = portal.getInt("sldMaxSpdUp");
    }

    if (portal.click("sldMinSpdUp"))
    {
      webConfig.drvUpMinSpeed = portal.getInt("sldMinSpdUp");
    }

    if (portal.click("sldMaxSpdDwn"))
    {
      webConfig.drvDownMaxSpeed = portal.getInt("sldMaxSpdDwn");
    }

    if (portal.click("sldMinSpdDwn"))
    {
      webConfig.drvDownMinSpeed = portal.getInt("sldMinSpdDwn");
    }

    if (portal.click("sldSens"))
    {
      webConfig.sensDetectValue = portal.getInt("sldSens");
    }

    if (portal.click("btnSave"))
    {
      log_i("Begin save parametres");

      eeConf->drvRightDoubleVal = webConfig.drvUpMaxSpeed;
      eeConf->drvRightVal = webConfig.drvUpMinSpeed;
      eeConf->drvLeftDoubleVal = fabs(webConfig.drvDownMaxSpeed); 
      eeConf->drvLeftVal = fabs(webConfig.drvDownMinSpeed);
      eeConf->detectSensorStopValue = webConfig.sensDetectValue;

      log_i(
      "EEParam = {%d, %d, %d, %d, %d}",
                                        eeConf->drvRightDoubleVal,
                                        eeConf->drvRightVal,
                                        eeConf->drvLeftDoubleVal,
                                        eeConf->drvLeftVal,
                                        eeConf->detectSensorStopValue
      );

      if(l2fp_SaveEESettings())
      {
        log_i("Save EEParams OK");
      }else{
        log_e("Save EEParams FAIL!");
      }

    }

    // if (portal.click("btnUpdFw"))
    // {
    //   log_i("Begin Update firmware");
    // }
  }
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

  if (!SPIFFS.begin(true))
    log_e("Begin FS Error!");

  portal->attachBuild(build);
  portal->attach(action);
  portal->start();
  // portal->enableOTA();   // без пароля
  portal->enableOTA("admin", "11101987");  // с паролем
  // portal->downloadAuto(true);

  for (;;)
  {
    while (portal->tick()){}

    vTaskDelay(10);
  }
}

bool WiFiConfig_Init(eeConstVal_t *_eeConf)
{
  if(_eeConf == NULL)
  {
    log_e("_eeConf is NULL value!");
    return false;
  }

  eeConf = _eeConf;

  webConfig.drvUpMaxSpeed = eeConf->drvRightDoubleVal;
  webConfig.drvUpMinSpeed = eeConf->drvRightVal;
  webConfig.drvDownMaxSpeed = abs(eeConf->drvLeftDoubleVal);
  webConfig.drvDownMinSpeed = abs(eeConf->drvLeftVal);
  webConfig.sensDetectValue = eeConf->detectSensorStopValue;

  BaseType_t createResult = xTaskCreatePinnedToCore(WiFiConfig_Task, "Task_WiFiConfig", 8192, (void *)&webPortal, 4, &WiFiTask, ARDUINO_RUNNING_CORE);
  if (createResult == pdFAIL)
    log_e("Fail create Task_WiFiConfig!");

  return true;
}