#include <Arduino.h>
#include <NimBLEDevice.h>
#include <EncButton.h>
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

#define LED_SYSTEM 25 // 5
#define LedSysInit() pinMode(LED_SYSTEM, OUTPUT);
#define LedSysOn() digitalWrite(LED_SYSTEM, HIGH)
#define LedSysOff() digitalWrite(LED_SYSTEM, LOW)
#define LedSysToogle() digitalWrite(LED_SYSTEM, !digitalRead(LED_SYSTEM))

#define POWER_BTN_PIN GPIO_NUM_4
#define UP_BTN_PIN GPIO_NUM_19
#define DOWN_BTN_PIN GPIO_NUM_18
#define LEFT_BTN_PIN GPIO_NUM_32
#define RIGHT_BTN_PIN GPIO_NUM_33

// RTC_DATA_ATTR char rtcLinkDevAddress[10];

// // EEPROM Settings struct;
// ee_settings_t eeSettigs = {};

EncButton<EB_TICK, POWER_BTN_PIN> BtnPwrLink; // просто кнопка <KEY>
EncButton<EB_TICK, UP_BTN_PIN> BtnUp;         // просто кнопка <KEY>
EncButton<EB_TICK, DOWN_BTN_PIN> BtnDwn;      // просто кнопка <KEY>
EncButton<EB_TICK, LEFT_BTN_PIN> BtnLft;      // просто кнопка <KEY>
EncButton<EB_TICK, RIGHT_BTN_PIN> BtnRght;    // просто кнопка <KEY>

WeDoHub_Client_t HubClient = WEDOHUB_CLIENT_SET_DEFAULT; //{false, false, "", ledSt_DISCONNECTED, 0, 0, 0, false, false};

void scanEndedCB(NimBLEScanResults results);

String DevCurrentAddress = "";
String DevCurrentName = "";

static NimBLEAdvertisedDevice *advDevice;
static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

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
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        pClient->updateConnParams(120, 120, 0, 60);

        HubClient.DevConnected = true;
        HubClient.LedSysCurStatus = ledSt_CONNECTED;
    };

    void onDisconnect(NimBLEClient *pClient)
    {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);

        HubClient.DevConnected = false;
        HubClient.LedSysCurStatus = ledSt_DISCONNECTED;
    };

    /** Called when the peripheral requests a change to the connection parameters.
     *  Return true to accept and apply them or false to reject and keep
     *  the currently used parameters. Default will return true.
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

/** Define a class to handle the callbacks when advertisments are received */
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

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results)
{
    Serial.println("Scan Ended");
}

/** Create a single global instance of the callback class to be used by all clients */
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
            if (!pClient->connect(advDevice, false))
            {
                Serial.println("Reconnect failed");
                return false;
            }
            log_i("Reconnected client");

            log_i("[2] Wait stable connection...");
            /** Делаем небольшую паузу перед первым подключением - сервер только запущен,
             * сразу возможны сбои при подключении (зависает звуковой сигнал хаба)**/
            delay(2000);
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
        log_i("[1] Wait stable connection...");
        /** Делаем небольшую паузу перед первым подключением - сервер только запущен,
         * сразу возможны сбои при подключении (зависает звуковой сигнал хаба)**/
        delay(2000);

        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
            log_i("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        log_i("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);

        if (!pClient->connect(advDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            log_i("Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            log_i("Failed to connect");
            return false;
        }
    }

    return l2fp_ConnectToHub(pClient);
}

void setup()
{
    bool PowerOnEnable = false;

    Serial.begin(115200);
    log_i("WakeUp Power button");

    pinMode(POWER_BTN_PIN, INPUT_PULLUP);
    esp_sleep_enable_ext0_wakeup(POWER_BTN_PIN, 0); // 1 = High, 0 = Low

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

    log_i("Starting NimBLE Client, begin init... (CoreID: %d)", xPortGetCoreID());

    // Иинициализация конфигурации клиента, чтение из EEPROM необходимых настроек
    l2fp_InitClientConfig(&HubClient);

    /** Initialize NimBLE, no device name spcified as we are not advertising */
    NimBLEDevice::init("");

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
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

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime, scanEndedCB);

    BtnActQueue = xQueueCreate(4, sizeof(MotorTransCmd_t));
    if (BtnActQueue == NULL) // Queue not created
    {
        log_i("BtnActQueue Not Created!!!");
    }
    else
    {
        log_i("BtnActQueue Create OK");
        xTaskCreatePinnedToCore(Task_Led, "Task_Led", 2048, (void *)&HubClient, 5, NULL, ARDUINO_RUNNING_CORE);
        xTaskCreatePinnedToCore(Task_Main, "Task_Main", 8192, (void *)&HubClient, 4, NULL, ARDUINO_RUNNING_CORE);
        xTaskCreatePinnedToCore(Task_Buttons, "Task_Buttons", 8192, (void *)&HubClient, 3, NULL, ARDUINO_RUNNING_CORE);
    }
}

void loop()
{
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
        case ledSt_CONNECTED:
            if (!ConStatusFlag)
            {
                LedSysOn();
                ~ConStatusFlag;
            }
            /** Обязательная задержка для работы других задач
             * (никакие условия в это задачи не выполняются) **/
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
    // memset(qBtnAction, 0, sizeof(qBtnAction));
    BtnPwrLink.setHoldTimeout(2000);

    BtnUp.setHoldTimeout(1000);
    BtnDwn.setHoldTimeout(1000);
    BtnLft.setHoldTimeout(1000);
    BtnRght.setHoldTimeout(1000);

    log_i("Run Task_Buttons (CoreID: %d)", xPortGetCoreID());

    for (;;)
    {
        BtnPwrLink.tick();
        BtnUp.tick();
        BtnDwn.tick();
        BtnLft.tick();
        BtnRght.tick();

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
                    if (BtnUp.press() && (l2fp_IsPermissibleDistance() == true))
                    {
                        log_i("(BtnUp.press() && (l2fp_IsPermissibleDistance() == true))");
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                    // Обработка удержания кнопки "ВПЕРЁД"
                    else if (BtnUp.held() && (l2fp_IsPermissibleDistance() == true))
                    {
                        log_i("(BtnUp.held() && (l2fp_IsPermissibleDistance() == true))");
                        qMotorSendCmd.driveNum = 1;
                        qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightDoubleVal;
                        xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                    }
                    // Если дистанция меньше нормы -> препядствие обнаружено, останавливаемся
                    if(l2fp_CriticalDistance())
                    {
                        log_i("(l2fp_DistanceStop())");
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

            // Проверяем подключен ли двигатель к порту 2
            if (devParam->DriveTwoEnabled)
            {
                // Right/Left Buttons commands
                if (BtnRght.press())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else if (BtnRght.held())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvRightDoubleVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else if (BtnLft.press())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else if (BtnLft.held())
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = devParam->eeConstConfig.drvLeftDoubleVal;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
                else if (
                    (BtnRght.release() && !BtnLft.state()) ||
                    (BtnLft.release() && !BtnRght.state()))
                {
                    qMotorSendCmd.driveNum = 2;
                    qMotorSendCmd.rotSpeedDir = MSD_STOP;
                    xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
                }
            }

            if (BtnPwrLink.hasClicks(3))
            {
                log_i("Save link address hub to EEPROM: %s", DevCurrentAddress.c_str());
                l2fp_LinkDevAddress(DevCurrentAddress);
                devParam->LedSysCurStatus = ledSt_LINK_HUB_ADDR_ACTION;
            }
        } // if (devParam->DevConnected)

        if (BtnPwrLink.held())
        {
            log_i("Held BtnPwrLink");
            // Go to sleep now
            log_i("Going to sleep now");
            vTaskDelay(1000);
            esp_deep_sleep_start();
        }

        vTaskDelay(10);
    }
}