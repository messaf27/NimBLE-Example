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

#define LED_SYSTEM 25 // 5
#define LedSysInit() pinMode(LED_SYSTEM, OUTPUT);
#define LedSysOn() digitalWrite(LED_SYSTEM, HIGH)
#define LedSysOff() digitalWrite(LED_SYSTEM, LOW)

enum LedSysBlinkStatus
{
    ledSt_CONNECTED,
    ledSt_CONNECTED_BAT_LOW,
    ledSt_DISCONNECTED,
    ledSt_DISCONNECTED_BAT_LOW,
    ledSt_SEARCH_CONTROLLER
};

#define POWER_BTN_PIN       GPIO_NUM_4
#define UP_BTN_PIN          GPIO_NUM_19
#define DOWN_BTN_PIN        GPIO_NUM_18
#define LEFT_BTN_PIN        GPIO_NUM_32
#define RIGHT_BTN_PIN       GPIO_NUM_33

#define BUTTON_PIN_BITMASK  0x200000000 // 2^33 in hex

RTC_DATA_ATTR int bootCount = 0;

EncButton<EB_TICK, POWER_BTN_PIN> BtnPwrLink;   // просто кнопка <KEY>
EncButton<EB_TICK, UP_BTN_PIN> BtnUp;           // просто кнопка <KEY>
EncButton<EB_TICK, DOWN_BTN_PIN> BtnDwn;        // просто кнопка <KEY>
EncButton<EB_TICK, LEFT_BTN_PIN> BtnLft;        // просто кнопка <KEY>
EncButton<EB_TICK, RIGHT_BTN_PIN> BtnRght;      // просто кнопка <KEY>

typedef struct
{
    bool DevConnected;
    bool DevSearch;
    String LinkDevAddres;
    uint8_t LedSysCurStatus;
    uint8_t PortNumTitleSensor;
    uint8_t PortNumDetectSensor;
    uint8_t DistanceCurrentValue;

} WeDoHub_Client_t;

WeDoHub_Client_t HubClient = {false, false, "", ledSt_DISCONNECTED, 0, 0, 0};

#define LEGO_HUB_NAME_STR   "LPF2 Smart Hub"



// Удаленный сервис, к которому мы хотим подключиться.
static NimBLEUUID LEGO_WeDo_advertisingUUID("00001523-1212-efde-1523-785feabcd123");
// static NimBLEUUID LEGO_LedButton_advertisingUUID("00001523-1212-efde-1523-785feabcd123");
// Сервис и характеристика по адреу хаба, где данные о уровне заряда батареи
static NimBLEUUID LEGO_HUB_BatteryServiceUUID("180F");
static NimBLEUUID LEGO_HUB_BatteryChUUID("2A19");

// Характеристика интересующего нас удаленного сервиса.
static NimBLEUUID LEGO("00004f0e-1212-efde-1523-785feabcd123");
static NimBLEUUID LEGOOutput("00001565-1212-efde-1523-785feabcd123");
static NimBLEUUID LEGOSensor("00001560-1212-efde-1523-785feabcd123");
static NimBLEUUID LEGOInput("00001563-1212-efde-1523-785feabcd123");
static NimBLEUUID LEGOButton("00001526-1212-efde-1523-785feabcd123");
static NimBLEUUID LEGOPorts("00001527-1212-efde-1523-785feabcd123");

// Глобальные переменные
NimBLERemoteService* pAdvertisingService = nullptr;
NimBLERemoteService* pLEGO = nullptr;
NimBLERemoteCharacteristic* pLEGOOutput = nullptr;
NimBLERemoteCharacteristic* pLEGOSensor = nullptr;
NimBLERemoteCharacteristic* pLEGOInput = nullptr;
NimBLERemoteCharacteristic* pLEGOButton = nullptr;
NimBLERemoteCharacteristic* pLEGOPorts = nullptr;

// String LinkDevAddres = "";
String LinkDevAddres = "";



void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice *advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

/** FreeRTOS Defines **/
xQueueHandle BtnActQueue;

enum MotorSpeedDir
{
    MSD_STOP = 0,
    MSD_SET_LEFT = -65,
    MSD_SET_LEFT_DOUBLE = -100,
    MSD_SET_RIGHT = 65,
    MSD_SET_RIGHT_DOUBLE = 100
};

typedef struct 
{
    uint8_t driveNum;
    int rotSpeedDir;
}
MotorTransCmd_t;

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
        String DevAddr = "";
        String DevName = "";
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice->toString().c_str());
        // DevName = advertisedDevice->getName().c_str();
        // Serial.print("Name: ");
        // Serial.println(DevName);

        // if(advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD")))
        if (advertisedDevice->isAdvertisingService(LEGO_WeDo_advertisingUUID))
        {
            DevAddr = advertisedDevice->getAddress().toString().c_str();
            Serial.println("Device Addr: " + DevAddr);
            if (LinkDevAddres == "" && pushButtonTempVar)
            {
                Serial.println("Found Our Service, begin link address device");

                LinkDevAddres = DevAddr;

                /** остановить сканирование перед подключением */
                NimBLEDevice::getScan()->stop();
                /** Сохраните ссылку на устройство в глобальном масштабе, чтобы клиент мог ее использовать.*/
                advDevice = advertisedDevice;
                /** Ready to connect now */
                doConnect = true;
            }
            else if (DevAddr == LinkDevAddres)
            {
                Serial.println("Found Our Service Linked device");
                /** остановить сканирование перед подключением */
                NimBLEDevice::getScan()->stop();
                /** Сохраните ссылку на устройство в глобальном масштабе, чтобы клиент мог ее использовать.*/
                advDevice = advertisedDevice;
                /** Ready to connect now */
                doConnect = true;
            }
            else
            {
                Serial.println("Found Our Service, but address don't link to this controller...");
            }
        }
    };
};

/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char *)pData, length);
    Serial.println(str.c_str());
}

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

    // /** Делаем небольшую паузу перед подключением - сервер только запущен,
    //  * сразу возможны сбои при подключении **/
    // vTaskDelay(3000);

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
            Serial.println("Reconnected client");
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
        Serial.println("[1] Wait stable connection...");
        /** Делаем небольшую паузу перед первым подключением - сервер только запущен,
         * сразу возможны сбои при подключении (зависает звуковой сигнал хаба)**/
        delay(2000);

        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

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
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    // delay(2000);

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService *pSvc = nullptr;
    NimBLERemoteCharacteristic *pChr = nullptr;
    NimBLERemoteDescriptor *pDsc = nullptr;

    NimBLERemoteService *pBatSvc = nullptr;
    NimBLERemoteCharacteristic *pBatChr = nullptr;

    pBatSvc = pClient->getService(LEGO_HUB_BatteryServiceUUID);
    if (pBatSvc)
    { /** make sure it's not null */
        pBatChr = pBatSvc->getCharacteristic(LEGO_HUB_BatteryChUUID);
        if (pBatChr)
        { /** make sure it's not null */
            if (pBatChr->canRead())
            {
                Serial.print(pBatChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                uint8_t batValue = pBatChr->readUInt8();
                Serial.println(batValue);
            }
        }
    }
    else
    {
        Serial.print("Not found: ");
        Serial.println(LEGO_HUB_BatteryServiceUUID.toString().c_str());
        return false;
    }

    pAdvertisingService = pClient->getService(LEGO_WeDo_advertisingUUID);
    if(pAdvertisingService == nullptr) {     /** убедитесь, что это не нольl */
        Serial.print("Failed to find our service UUID: ");
        Serial.println(LEGO_WeDo_advertisingUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.print("Found our service: ");
    Serial.println(LEGO_WeDo_advertisingUUID.toString().c_str());
    
    pLEGOPorts = pAdvertisingService->getCharacteristic(LEGOPorts);
    if(pLEGOPorts == nullptr) {     /** убедитесь, что это не нольl */
        Serial.print("Failed to find our service UUID(LEGOPorts): ");
        Serial.println(LEGOPorts.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.print("Found our service: ");
    Serial.println(LEGOPorts.toString().c_str());


    pLEGO = pClient->getService(LEGO);
    if(pLEGO == nullptr) {     /** убедитесь, что это не нольl */
        Serial.print("Failed to find our service UUID: ");
        Serial.println(LEGO.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.print("Found our service: ");
    Serial.println(LEGO.toString().c_str());

    pLEGOOutput = pLEGO->getCharacteristic(LEGOOutput);
    if (pLEGOOutput == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(LEGOOutput.toString().c_str());
        pClient->disconnect();
        return false;
    }       
    Serial.print("Found our characteristic: ");
    Serial.println(LEGOOutput.toString().c_str());

    pLEGOInput = pLEGO->getCharacteristic(LEGOInput);
    if (pLEGOInput == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(LEGOInput.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.print("Found our characteristic: ");
    Serial.println(LEGOInput.toString().c_str());

    pLEGOButton = pAdvertisingService->getCharacteristic(LEGOButton);
    if (pLEGOButton == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(LEGOButton.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.print("Found our characteristic: ");
    Serial.println(LEGOButton.toString().c_str());

    if(pLEGOButton->canNotify()){
        if(!pLEGOButton->subscribe(true, notifyCB)) {
            /** Disconnect if subscribe failed */
            pClient->disconnect();
            return false;
        }
        Serial.println("Subscribe to LEGOButton OK");
    }

    if(pLEGOPorts->canNotify()) {
        //if(!pChr->registerForNotify(notifyCB)) {
        if(!pLEGOPorts->subscribe(true, notifyCB)) {
            /** Disconnect if subscribe failed */
            pClient->disconnect();
            return false;
        }
        Serial.println("Subscribe pLEGOPorts Notify");
    }


    Serial.println("Done with this device!");
    return true;
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        log_i("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        log_i("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        log_i("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        log_i("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        log_i("Wakeup caused by ULP program");
        break;
    default:
        log_i("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

void writeMotor(uint8_t wedo_port,int wedo_speed)
{
    //From http://www.ev3dev.org/docs/tutorials/controlling-wedo2-motor/
    //conversion from int (both pos and neg) to unsigned 8 bit int
    uint8_t speed_byte = wedo_speed;
    uint8_t command[] = {wedo_port, 0x01, 0x01, speed_byte};
    pLEGOOutput->writeValue(command,sizeof(command));
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
        if (millis() >= 2000)
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

    log_i("Starting NimBLE Client, begin init...");
    // //Increment boot number and print it every reboot
    // ++bootCount;
    // log_i("Boot number: %d",bootCount);

    // //Print the wakeup reason for ESP32
    // print_wakeup_reason();

    // esp_sleep_enable_ext0_wakeup(POWER_BTN_PIN, 0); //1 = High, 0 = Low

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

    BtnActQueue = xQueueCreate(4, sizeof (MotorTransCmd_t));
    if (BtnActQueue == NULL)  // Queue not created 
    {    
        log_i("BtnActQueue Not Created!!!");
    }else{
        log_i("BtnActQueue Create OK");
        xTaskCreatePinnedToCore(Task_Led, "Task_Led", 2048, (void *)&HubClient, 5, NULL, ARDUINO_RUNNING_CORE);
        xTaskCreatePinnedToCore(Task_Main, "Task_Main", 8192, (void *)&HubClient, 4, NULL, ARDUINO_RUNNING_CORE);
        xTaskCreatePinnedToCore(Task_Buttons, "Task_Buttons", 4092, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
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

    log_i("Run Task_Main");

    for (;;)
    {
        /** Loop here until we find a device we want to connect to */
        while (!doConnect)
        {
            if (xQueueReceive(BtnActQueue, &qMotorSendCmd, 10) == pdPASS)
            {
                log_i("Queue recive: Motor = %d, SpdDir = %d", qMotorSendCmd.driveNum, qMotorSendCmd.rotSpeedDir);
                writeMotor(qMotorSendCmd.driveNum, qMotorSendCmd.rotSpeedDir);
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

    log_i("Run Task_Led");

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
                LedSysOn();
                vTaskDelay(500);

                LedSysOff();
                vTaskDelay(500);

                if (ConStatusFlag)
                    ~ConStatusFlag;
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
    (void)pvParameters;

    MotorTransCmd_t qMotorSendCmd;
    // memset(qBtnAction, 0, sizeof(qBtnAction));
    BtnPwrLink.setHoldTimeout(2000);

    BtnUp.setHoldTimeout(1000);
    BtnDwn.setHoldTimeout(1000);
    BtnLft.setHoldTimeout(1000);
    BtnRght.setHoldTimeout(1000);

    log_i("Run Task_Buttons");

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

        /** Send Queue**/
        // Up/Down Buttons commands
        if (BtnUp.press()){ 
            qMotorSendCmd.driveNum = 1;
            qMotorSendCmd.rotSpeedDir = MSD_SET_RIGHT;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(BtnUp.held()){
            qMotorSendCmd.driveNum = 1;
            qMotorSendCmd.rotSpeedDir = MSD_SET_RIGHT_DOUBLE;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if (BtnDwn.press()){ 
            qMotorSendCmd.driveNum = 1;
            qMotorSendCmd.rotSpeedDir = MSD_SET_LEFT;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(BtnDwn.held()){
            qMotorSendCmd.driveNum = 1;
            qMotorSendCmd.rotSpeedDir = MSD_SET_LEFT_DOUBLE;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(
            (BtnUp.release() && !BtnDwn.state())
            ||
            (BtnDwn.release() && !BtnUp.state())
        ){
            qMotorSendCmd.driveNum = 1;
            qMotorSendCmd.rotSpeedDir = MSD_STOP;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }
        
        // Right/Left Buttons commands
        if (BtnRght.press()){ 
            qMotorSendCmd.driveNum = 2;
            qMotorSendCmd.rotSpeedDir = MSD_SET_RIGHT;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(BtnRght.held()){
            qMotorSendCmd.driveNum = 2;
            qMotorSendCmd.rotSpeedDir = MSD_SET_RIGHT_DOUBLE;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if (BtnLft.press()){ 
            qMotorSendCmd.driveNum = 2;
            qMotorSendCmd.rotSpeedDir = MSD_SET_LEFT;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(BtnLft.held()){
            qMotorSendCmd.driveNum = 2;
            qMotorSendCmd.rotSpeedDir = MSD_SET_LEFT_DOUBLE;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }else
        if(
            (BtnRght.release() && !BtnLft.state())
            ||
            (BtnLft.release() && !BtnRght.state())
        ){
            qMotorSendCmd.driveNum = 2;
            qMotorSendCmd.rotSpeedDir = MSD_STOP;
            xQueueSend(BtnActQueue, &qMotorSendCmd, portMAX_DELAY);
        }

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