#include <Arduino.h>

#include <NimBLEDevice.h>

#include <EncButton.h>

#define LED_SYSTEM          25//5
#define LedSysInit()        pinMode(LED_SYSTEM, OUTPUT);
#define LedSysOn()          digitalWrite(LED_SYSTEM,LOW)
#define LedSysOff()         digitalWrite(LED_SYSTEM,HIGH)

EncButton<EB_TICK, 19> BtnUp;        // просто кнопка <KEY>
EncButton<EB_TICK, 18> BtnDwn;        // просто кнопка <KEY>
EncButton<EB_TICK, 32> BtnLft;        // просто кнопка <KEY>
EncButton<EB_TICK, 33> BtnRght;        // просто кнопка <KEY>

typedef struct 
{
    bool DevConnected;
    String LinkDevAddres;
    uint8_t PortNumTitleSensor;
    uint8_t PortNumDetectSensor;
    uint8_t DistanceCurrentValue;

} WeDoHub_Client_t;

WeDoHub_Client_t HubClient = { false, "", 0, 0, 0 };

#define LEGO_HUB_NAME_STR "LPF2 Smart Hub"
// String LinkDevAddres = "";
String LinkDevAddres = "";

// Удаленный сервис, к которому мы хотим подключиться.
static NimBLEUUID LEGO_WeDo_advertisingUUID("00001523-1212-efde-1523-785feabcd123");

static NimBLEUUID LEGO_HUB_BatteryServiceUUID("180F");
static NimBLEUUID LEGO_HUB_BatteryChUUID("2A19");

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice *advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

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
    };

    void onDisconnect(NimBLEClient *pClient)
    {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
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

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize())
    {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
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
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else
        {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
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

    delay(2000);

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

    Serial.println("Done with this device!");
    return true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");
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

    xTaskCreatePinnedToCore(Task_Led, "Task_Led", 1024, NULL, 4, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(Task_Main, "Task_Main", 8192, (void*)&HubClient, 3, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(Task_Buttons, "Task_Buttons", 1024, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void loop()
{
    // /** Loop here until we find a device we want to connect to */
    // while (!doConnect)
    // {
    //     delay(1);
    // }

    // doConnect = false;

    // /** Found a device we want to connect to, do it now */
    // if (connectToServer())
    // {
    //     Serial.println("Success! we should now be getting notifications!");
    // }
    // else
    // {
    //     Serial.println("Failed to connect, starting scan");
    //     NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    // }


}


// FreeRTOS Task's
static void Task_Main(void *pvParameters)
{
    // (void)pvParameters;

    WeDoHub_Client_t *devParam = (WeDoHub_Client_t*) pvParameters;

    Serial.println("Run Task_Main");

    for(;;)
    {
        /** Loop here until we find a device we want to connect to */
        while (!doConnect)
        {
            vTaskDelay(10);
        }

        doConnect = false;

        /** Found a device we want to connect to, do it now */
        if (connectToServer())
        {
            Serial.println("Success! we should now be getting notifications!");
        }
        else
        {
            Serial.println("Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
        }

        vTaskDelay(100);
    }
}

static void Task_Led(void *pvParameters)
{
    (void)pvParameters;

    LedSysInit();

    Serial.println("Run Task_Led");

    for(;;)
    {
        LedSysOn();
        vTaskDelay(500);

        LedSysOff();
        vTaskDelay(500);
    }
}

static void Task_Buttons(void *pvParameters)
{
    (void)pvParameters;

    BtnUp.setHoldTimeout(1000);
    BtnDwn.setHoldTimeout(1000);
    BtnLft.setHoldTimeout(1000);
    BtnRght.setHoldTimeout(1000);

    Serial.println("Run Task_Buttons");

    for(;;)
    {
        BtnUp.tick();
        BtnDwn.tick();
        BtnLft.tick();
        BtnRght.tick();

        vTaskDelay(10);

        if (BtnUp.click()) Serial.println("Click BtnUp");
        if (BtnDwn.click()) Serial.println("Click BtnDwn");
        if (BtnLft.click()) Serial.println("Click BtnLft");
        if (BtnRght.click()) Serial.println("Click BtnRght");
    }
}