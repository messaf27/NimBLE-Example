#include <Arduino.h>
#include <NimBLEDevice.h>
#include <EncButton.h>

// For WiFi Configuration Mode
#include "WiFiConfigurator.h"

#include "esp32-hal-log.h"
/**
    log_e – ошибка (минимальная)
    log_w - предупреждение
    log_i - Информация
    log_d - отладка
    log_v – многословный (самый высокий)
    Example:  log_d("MY DEV", "Hello %s", "world!!!");
 */

#include "lpf2_smart_hub.h"

#define LED_SYSTEM      GPIO_NUM_25 // 5
#define LedSysInit()    pinMode(LED_SYSTEM, OUTPUT);
#define LedSysOn()      digitalWrite(LED_SYSTEM, HIGH)
#define LedSysOff()     digitalWrite(LED_SYSTEM, LOW)
#define LedSysToogle()  digitalWrite(LED_SYSTEM, !digitalRead(LED_SYSTEM))

#define POWER_BTN_PIN   GPIO_NUM_4
#define UP_BTN_PIN      GPIO_NUM_19
#define DOWN_BTN_PIN    GPIO_NUM_18
#define LEFT_BTN_PIN    GPIO_NUM_32
#define RIGHT_BTN_PIN   GPIO_NUM_33

EncButton<EB_TICK, POWER_BTN_PIN>   BtnPwrLink; // Кнопка "ВКЛЮЧЕНИЯ/ОТКЛЮЧЕНИЯ" 
EncButton<EB_TICK, UP_BTN_PIN>      BtnUp;      // Кнопка "ВПЕРЕД/ВВЕРХ" 
EncButton<EB_TICK, DOWN_BTN_PIN>    BtnDwn;     // Кнопка "НАЗАД/ВНИЗ" 
EncButton<EB_TICK, LEFT_BTN_PIN>    BtnLft;     // Кнопка "ВЛЕВО/ВПЕРЕД" 
EncButton<EB_TICK, RIGHT_BTN_PIN>   BtnRght;    // Кнопка "ВПРАВО/НАЗАД" 

WeDoHub_Client_t HubClient = WEDOHUB_CLIENT_SET_DEFAULT; 

void scanEndedCB(NimBLEScanResults results);

String DevCurrentAddress = "";
String DevCurrentName = "";

static NimBLEAdvertisedDevice *advDevice;
static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */
static bool WiFiCinfigEnable = false;

/** FreeRTOS Defines **/
xQueueHandle BtnActQueue;

static void Task_Led(void *pvParameters);
static void Task_Main(void *pvParameters);
static void Task_Buttons(void *pvParameters);

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient *pClient)
    {
        log_i("Connected");
        /** После подключения мы должны изменить параметры, если нам не нужно быстрое время отклика.
          * Эти настройки: интервал 150 мс, задержка 0, тайм-аут 450 мс.
          * Время ожидания должно быть кратно интервалу, минимум 100 мс.
          * Я считаю, что интервал кратен 3-5 * этот интервал лучше всего подходит для быстрого ответа/повторного подключения.
          * Минимальный интервал: 120 * 1,25 мс = 150, максимальный интервал: 120 * 1,25 мс = 150, 0 задержек, 60 * 10 мс = 600 мс тайм-аут
         */
        pClient->updateConnParams(120, 120, 0, 60);

        HubClient.DevConnected = true;
        HubClient.LedSysCurStatus = ledSt_CONNECTED;
    };

    void onDisconnect(NimBLEClient *pClient)
    {
        log_i("%s Disconnected - Starting scan", pClient->getPeerAddress().toString().c_str());
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);

        HubClient.DevConnected = false;
        HubClient.LedSysCurStatus = ledSt_DISCONNECTED;
    };

    /** Вызывается, когда периферийное устройство запрашивает изменение параметров подключения.
      * Верните true, чтобы принять и применить их, или false, чтобы отклонить и сохранить
      * текущие используемые параметры. По умолчанию будет возвращено значение true.
     */
    bool onConnParamsUpdateRequest(NimBLEClient *pClient, const ble_gap_upd_params *params)
    {
        if (params->itvl_min < 24)
        { /** 1.25ms units */
            return false;
        }
        else if (params->itvl_max > 40)
        { /** 1.25ms units */
            return false;
        }
        else if (params->latency > 2)
        { /** Number of intervals allowed to skip */
            return false;
        }
        else if (params->supervision_timeout > 100)
        { /** 10ms units */
            return false;
        }

        return true;
    };
};

/** Определите класс для обработки обратных вызовов при получении рекламы.*/
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
        bool pushButtonTempVar = true;

        log_i("Advertised Device found: %s", advertisedDevice->toString().c_str());

        if (l2fp_isMainService(advertisedDevice))
        {
            DevCurrentAddress = advertisedDevice->getAddress().toString().c_str();
            DevCurrentName = advertisedDevice->getName().c_str();

            log_i("Device Addr: %s", DevCurrentAddress.c_str());
            if (HubClient.LinkDevAddress == configLEGO_HUB_DEFAULT_ADDRES)
            {
                log_i("Found Our Service, device without binding (%s)", HubClient.LinkDevAddress.c_str());

                /** остановить сканирование перед подключением */
                NimBLEDevice::getScan()->stop();
                /** Сохраните ссылку на устройство в глобальном масштабе, чтобы клиент мог ее использовать.*/
                advDevice = advertisedDevice;
                /** Ready to connect now */
                doConnect = true;
            }
            else if (DevCurrentAddress == HubClient.LinkDevAddress)
            {
                log_i("Found Our Service Linked device");
                /** остановить сканирование перед подключением */
                NimBLEDevice::getScan()->stop();
                /** Сохраните ссылку на устройство в глобальном масштабе, чтобы клиент мог ее использовать.*/
                advDevice = advertisedDevice;
                /** Ready to connect now */
                doConnect = true;
            }
            else
            {
                log_i("Found Our Service, but address don't link to this controller...");
            }
        }
    };
};

/** Обратный вызов для обработки результатов последнего сканирования или его перезапуска */
void scanEndedCB(NimBLEScanResults results)
{
    log_i("Scan Ended (Stop)");
}

/** Создайте один глобальный экземпляр класса обратного вызова, который будет использоваться всеми клиентами.*/
static ClientCallbacks clientCB;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer()
{
    NimBLEClient *pClient = nullptr;

    /** Проверьте, есть ли у нас клиент, который мы должны повторно использовать в первую очередь. **/
    if (NimBLEDevice::getClientListSize())
    {
        /** Особый случай, когда мы уже знаем это устройство, мы отправляем false в качестве
         * второй аргумент в connect() для предотвращения обновления базы данных сервиса.
         * Это значительно экономит время и энергию.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
            log_i("[2] Wait stable Reconnection...(%d ms)",configLEGO_HUB_DEFAULT_STABLE_DELAY_TIMEOUT_MS);
            /** Делаем небольшую паузу перед первым подключением - сервер только запущен,
             * сразу возможны сбои при подключении (зависает звуковой сигнал хаба)**/
            vTaskDelay(configLEGO_HUB_DEFAULT_STABLE_DELAY_TIMEOUT_MS);

            if (!pClient->connect(advDevice, false))
            {
                log_e("Reconnect failed");
                return false;
            }
            log_i("Reconnected client");
        }
        /** У нас еще нет клиента, который знает это устройство,
         * мы проверим отключенный клиент, который мы можем использовать.
         */
        else
        {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** Нет клиента для повторного использования? Создайте новый. */
    if (!pClient)
    {
        log_i("[1] Wait stable connection... (%d ms)", configLEGO_HUB_DEFAULT_STABLE_DELAY_TIMEOUT_MS);
        /** Делаем небольшую паузу перед первым подключением - сервер только запущен,
         * сразу возможны сбои при подключении (зависает звуковой сигнал хаба)**/
        vTaskDelay(configLEGO_HUB_DEFAULT_STABLE_DELAY_TIMEOUT_MS);

        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
            log_i("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        log_i("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Установите начальные параметры подключения: эти настройки: интервал 15 мс, задержка 0, тайм-аут 120 мс.
          * Эти настройки безопасны для 3 клиентов для надежного подключения, могут работать быстрее, если у вас меньше
          * соединения. Тайм-аут должен быть кратен интервалу, минимум 100 мс.
          * Минимальный интервал: 12 * 1,25 мс = 15, максимальный интервал: 12 * 1,25 мс = 15, 0 задержек, 51 * 10 мс = 510 мс тайм-аут
         */
        pClient->setConnectionParams(12, 12, 0, 51);

        /** Установите, как долго мы готовы ждать завершения соединения (в секундах), по умолчанию 30. */
        pClient->setConnectTimeout(5);

        if (!pClient->connect(advDevice))
        {
            /** Создал клиент, но не смог подключиться, не нужно его оставлять, так как в нем нет данных */
            NimBLEDevice::deleteClient(pClient);
            log_e("Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            log_e("Failed to connect");
            return false;
        }
    }

    return l2fp_ClientInitHub(pClient);
}

void setup()
{
    bool PowerOnEnable = false;

    Serial.begin(115200);
    log_i("WakeUp Power button");

    pinMode(POWER_BTN_PIN, INPUT_PULLUP);
    esp_sleep_enable_ext0_wakeup(POWER_BTN_PIN, 0); // 1 = High, 0 = Low

    pinMode(UP_BTN_PIN, INPUT_PULLUP); // For WiFi Configuration Detect

    while (digitalRead(POWER_BTN_PIN) == LOW)
    {
        if (millis() >= configLEGO_HUB_TIMEOUT_MS_POWER_ON)
        {
            log_i("Hold power button");
            PowerOnEnable = true;
            break;
        }
    }

    if (!PowerOnEnable)
    {
        log_i("Going to sleep now");
        vTaskDelay(1000);
        esp_deep_sleep_start();
        log_i("This will never be printed");
    }
    else if(digitalRead(UP_BTN_PIN) == LOW)
    {
        WiFiCinfigEnable = true;
        log_i("WiFi Config Enable");
    }

    log_i("Starting NimBLE Client, begin init... (CoreID: %d)", xPortGetCoreID());

    // l2fp_SetDefaultEESettings();

    // Иинициализация конфигурации клиента, чтение из EEPROM необходимых настроек
    l2fp_InitClientConfig(&HubClient);

    /** Инициализируйте NimBLE, имя устройства не указано, так как мы не рекламируем*/
    NimBLEDevice::init("");

    /** Установите возможности ввода-вывода устройства, каждый параметр будет запускать другой метод сопряжения.
      * BLE_HS_IO_DISPLAY_ONLY — сопряжение ключей доступа
      * BLE_HS_IO_DISPLAY_YESNO — спаривание числового сравнения
      * BLE_HS_IO_NO_INPUT_OUTPUT — настройка ПО УМОЛЧАНИЮ — работает только сопряжение
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // использовать пароль
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); // использовать числовое сравнение

    /** 2 разных способа установить безопасность - оба вызова достигают одного и того же результата.
      * без склеивания, без защиты «человек посередине», надежные соединения.
     *
     *  Это значения по умолчанию, показанные здесь только для демонстрации.
     */
    // NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan *pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Активное сканирование будет собирать данные ответов на сканирование от рекламодателей, 
     * но будет потреблять больше энергии от обоих устройств.
     */
    pScan->setActiveScan(true);
    /** Начать сканирование рекламодателей на указанное время сканирования (в секундах) 0 = навсегда
     *  Необязательный обратный вызов для остановки сканирования.
     */
    pScan->start(scanTime, scanEndedCB);

    // FreeRTOS Init Task's and Queue's
    BtnActQueue = xQueueCreate(4, sizeof(MotorTransCmd_t));
    if (BtnActQueue == NULL) // Queue not created
    {
        log_i("BtnActQueue Not Created!!!");
    }
    else
    {
        log_i("BtnActQueue Create OK");

        BaseType_t createResult = pdFAIL;

        createResult = xTaskCreatePinnedToCore(Task_Led, "Task_Led", 2048, (void *)&HubClient, 5, NULL, ARDUINO_RUNNING_CORE);
        if(createResult == pdFAIL) 
            log_e("Fail create Task_Led!");

        createResult = xTaskCreatePinnedToCore(Task_Main, "Task_Main", 4092, (void *)&HubClient, 4, NULL, ARDUINO_RUNNING_CORE); // 8192
        if(createResult == pdFAIL) 
            log_e("Fail create Task_Main!"); 

        createResult = xTaskCreatePinnedToCore(Task_Buttons, "Task_Buttons", 4092, (void *)&HubClient, 3, NULL, ARDUINO_RUNNING_CORE); // 8192
        if(createResult == pdFAIL) 
            log_e("Fail create Task_Buttons!"); 
    }

    if(WiFiCinfigEnable)
    {
        WiFiConfig_Init();
    }
}

void loop()
{
    // Use FreeRTOS, The whole program is divided into tasks
}

// FreeRTOS Task's
static void Task_Main(void *pvParameters)
{
    // (void)pvParameters;
    WeDoHub_Client_t *devParam = (WeDoHub_Client_t *)pvParameters;
    MotorTransCmd_t qMotorSendCmd;
    static TickType_t lstTickCount = 0;
    bool result = false;

    log_i("Run Task_Main (CoreID: %d)", xPortGetCoreID());

    for (;;)
    {
        /** Loop here until we find a device we want to connect to */
        while (!doConnect)
        {
            if (devParam->DevConnected)
            {
                if (xQueueReceive(BtnActQueue, &qMotorSendCmd, 10) == pdPASS)
                {
                    log_i("Queue recive: Motor = %d, SpdDir = %d", qMotorSendCmd.driveNum, qMotorSendCmd.rotSpeedDir);
                    l2fp_WriteMotorCommand(qMotorSendCmd.driveNum, qMotorSendCmd.rotSpeedDir);
                }
                vTaskDelay(10);

                if (xTaskGetTickCount() - lstTickCount > configLEGO_HUB_SENSOR_POLL_PERIOD_MS)
                {
                    if (devParam->PortNumTitleSensor > 0)
                    {
                        result = l2fp_SetTiltSensor(devParam->PortNumTitleSensor);
                        // log_i("Update TiltSensor (port %d) %s", devParam->PortNumTitleSensor, result?"OK":"FAIL");
                    }
                    // Проверяем подключен ли датчик препядствий
                    if (devParam->PortNumDetectSensor > 0)
                    {
                        result = l2fp_SetDetectSensor(devParam->PortNumDetectSensor);
                        // log_i("Update DetectSensor (port %d) %s", devParam->PortNumDetectSensor, result?"OK":"FAIL");
                    }

                    lstTickCount = xTaskGetTickCount();
                }

                if (devParam->PortNumDetectSensor > 0)
                    l2fp_DetectSensorAction();
            }
            vTaskDelay(10);
        }

        doConnect = false;

        /** Found a device we want to connect to, do it now */
        if (connectToServer())
        {
            log_i("Success! we should now be getting notifications!");
            devParam->DevConnected = true;
            devParam->LedSysCurStatus = ledSt_CONNECTED;
            l2fp_WriteIndexColor(LEGO_COLOR_GREEN);
        }
        else
        {
            log_i("Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
            devParam->DevConnected = false;
            devParam->LedSysCurStatus = ledSt_DISCONNECTED;
        }

        vTaskDelay(10);
    }
}

static void Task_Led(void *pvParameters)
{
    // (void)pvParameters;
    WeDoHub_Client_t *devParam = (WeDoHub_Client_t *)pvParameters;

    bool ConStatusFlag = false;
    LedSysInit();

    log_i("Run Task_Led (CoreID: %d)", xPortGetCoreID());

    for (;;)
    {

        switch (devParam->LedSysCurStatus)
        {
            case ledSt_CONNECTING_INIT_HUB:
                LedSysToogle();
                vTaskDelay(250);
            break;

            case ledSt_CONNECTED:
                if (!ConStatusFlag)
                {
                    LedSysOn();
                    ~ConStatusFlag;
                }

                // Если мы подключены к хабу, то ысветодиод горит не прерывно
                /** Обязательная задержка для работы других задач
                 * (никакие условия в это задачи не выполняются) 
                 * иначе задача зависнет а вместе с ней остальные потоки**/
                vTaskDelay(250);
        
                break;

            case ledSt_DISCONNECTED:
                LedSysToogle();
                vTaskDelay(500);

                if (ConStatusFlag)
                    ~ConStatusFlag;
                break;

            case ledSt_SEARCH_CONTROLLER:
                devParam->LedSysCurStatus = ledSt_CONNECTED;
                for (uint i = 0; i < 8; i++)
                {
                    LedSysOff();
                    vTaskDelay(100);
                    LedSysOn();
                    vTaskDelay(50);
                }
                break;

            case ledSt_HOLD_PWR_BUTTON:
                LedSysOff();
                vTaskDelay(100);
                LedSysOn();
                vTaskDelay(50);
                LedSysOff();

                LedSysOff();
                vTaskDelay(100);
                LedSysOn();
                vTaskDelay(50);
                LedSysOff();

                vTaskDelay(750);
                break;

            case ledSt_LINK_HUB_ADDR_ACTION:
                devParam->LedSysCurStatus = ledSt_CONNECTED;
                for (uint i = 0; i < 4; i++)
                {
                    LedSysOff();
                    vTaskDelay(50);
                    LedSysOn();
                    vTaskDelay(25);
                }
                break;

            default:
                /** Обязательная задержка для работы других задач
                 * (никакие условия в это задачи не выполняются) **/
                vTaskDelay(250);
                break;
        }
    }
}

static void Task_Buttons(void *pvParameters)
{
    // (void)pvParameters;
    WeDoHub_Client_t *devParam = (WeDoHub_Client_t *)pvParameters;

    MotorTransCmd_t qMotorSendCmd;
    static bool drvStopFlag = false;
    static uint8_t lastLedState = ledSt_DISCONNECTED;
    static uint8_t PwrBtnStepCount = 0;
    // memset(qBtnAction, 0, sizeof(qBtnAction));
    BtnPwrLink.setHoldTimeout(configLEGO_HUB_TIMEOUT_MS_HELD_PWR_BUTTON);

    BtnUp.setHoldTimeout(configLEGO_HUB_TIMEOUT_MS_HELD_CONTROL_BUTTON);
    BtnDwn.setHoldTimeout(configLEGO_HUB_TIMEOUT_MS_HELD_CONTROL_BUTTON);
    BtnLft.setHoldTimeout(configLEGO_HUB_TIMEOUT_MS_HELD_CONTROL_BUTTON);
    BtnRght.setHoldTimeout(configLEGO_HUB_TIMEOUT_MS_HELD_CONTROL_BUTTON);

    log_i("Run Task_Buttons (CoreID: %d)", xPortGetCoreID());

    for (;;)
    {
        BtnPwrLink.tick();
        BtnUp.tick();
        BtnDwn.tick();
        BtnLft.tick();
        BtnRght.tick();

        if (BtnPwrLink.held())
        {
            log_i("Held BtnPwrLink");
            lastLedState = devParam->LedSysCurStatus;
            devParam->LedSysCurStatus = ledSt_HOLD_PWR_BUTTON;

        }
        
        if (BtnPwrLink.step())
        {
            PwrBtnStepCount++;
            log_i("Step BtnPwrLink (count: %d)", PwrBtnStepCount);

            if(PwrBtnStepCount >= configLEGO_HUB_NUM_OF_STEPS_HOLD_PWR_BUTTON)
            {
                log_i("Held Steps BtnPwrLink is %d -> Going to sleep now", PwrBtnStepCount);
                // Go to sleep now
                vTaskDelay(1000);
                esp_deep_sleep_start();
            }
        }
        
        if(BtnPwrLink.releaseStep())
        {
            PwrBtnStepCount = 0;
            devParam->LedSysCurStatus = lastLedState;
        }
        
        // if (BtnPwrLink.click())
        //     log_i("Click BtnPwrLink");
        // if (BtnUp.click())
        //     log_i("Click BtnUp");
        // if (BtnDwn.click())
        //     log_i("Click BtnDwn");
        // if (BtnLft.click())
        //     log_i("Click BtnLft");
        // if (BtnRght.click())
        //     log_i("Click BtnRght");

        // if (BtnUp.getState())
        //     log_i("isHolded BtnUp");

        // if (BtnDwn.isHolded())
        //     log_i("isHolded BtnDwn");

        if (devParam->DevConnected)
        {
            /** Send Queue**/
            // Проверяем подключен ли двигатель к порту 1
            if (devParam->DriveOneEnabled)
            {
                // Проверяем подключен ли датчик препядствий
                if (devParam->PortNumDetectSensor > 0)
                {
                    // Обработка нажатия кнопки "ВПЕРЁД"
                    if (BtnUp.press() && l2fp_IsPermissibleDistance())
                    {
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                    // Обработка удержания кнопки "ВПЕРЁД"
                    else if (BtnUp.held() && l2fp_IsPermissibleDistance())
                    {
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightDoubleVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                    // Если дистанция меньше нормы -> препядствие обнаружено, останавливаемся
                    if(l2fp_CriticalDistance())
                    {
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = MSD_STOP;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                }
                else //(devParam->PortNumDetectSensor > 0)
                {
                    // Обработка нажатия кнопки "ВПЕРЁД"
                    if (BtnUp.press())
                    {
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                    else
                    // Обработка удержания кнопки "ВПЕРЁД"
                    if (BtnUp.held())
                    {
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightDoubleVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                } // (devParam->PortNumDetectSensor > 0)

                // Обработка нажатия кнопки "НАЗАД"
                if (BtnDwn.press())
                {
                    qMotorSendCmd.driveNum = 1;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                // Обработка удержания кнопки "НАЗАД"
                else if (BtnDwn.held())
                {
                    qMotorSendCmd.driveNum = 1;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftDoubleVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else if (
                    (BtnUp.release() && !BtnDwn.state()) ||
                    (BtnDwn.release() && !BtnUp.state())
                )
                {
                    qMotorSendCmd.driveNum = 1;
                    qMotorSendCmd.rotSpeedDir = MSD_STOP;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
            }

            // Проверяем подключен ли датчик препядствий
            if (devParam->PortNumDetectSensor > 0)
            {
                // Обработка нажатия кнопки "ВПРАВО"
                if (BtnLft.press() && l2fp_IsPermissibleDistance())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                // Обработка удержания кнопки "ВПРАВО"
                else if (BtnLft.held() && l2fp_IsPermissibleDistance())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftDoubleVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                // Если дистанция меньше нормы -> препядствие обнаружено, останавливаемся
                if(l2fp_CriticalDistance())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = MSD_STOP;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
            }
            else //(devParam->PortNumDetectSensor > 0)
            {
                // Обработка нажатия кнопки "ВПРАВО"
                if (BtnLft.press())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else
                // Обработка удержания кнопки "ВПРАВО"
                if (BtnLft.held())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftDoubleVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
            } // (devParam->PortNumDetectSensor > 0)

            // Обработка нажатия кнопки "ВЛЕВО"
            if (BtnRght.press())
            {
                qMotorSendCmd.driveNum = 2;
                qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightVal;
                xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
            }
            // Обработка удержания кнопки "ВЛЕВО"
            else if (BtnRght.held())
            {
                qMotorSendCmd.driveNum = 2;
                qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightDoubleVal;
                xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
            }
            else if (
                (BtnRght.release() && !BtnLft.state()) ||
                (BtnLft.release() && !BtnRght.state())
            )
            {
                qMotorSendCmd.driveNum = 2;
                qMotorSendCmd.rotSpeedDir = MSD_STOP;
                xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
            }

            if (BtnPwrLink.hasClicks(3))
            {
                log_i("Save link address hub to EEPROM: %s", DevCurrentAddress.c_str());
                l2fp_LinkDevAddress(DevCurrentAddress);
                devParam->LedSysCurStatus = ledSt_LINK_HUB_ADDR_ACTION;
            }
        } // if (devParam->DevConnected)

        BtnPwrLink.resetState();
        BtnUp.resetState();
        BtnDwn.resetState();
        BtnLft.resetState();
        BtnRght.resetState();

        vTaskDelay(10);
    }
}