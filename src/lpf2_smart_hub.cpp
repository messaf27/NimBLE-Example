
#include "lpf2_smart_hub.h"
#include "esp32-hal-log.h"

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

bool l2fp_isMainService(NimBLEAdvertisedDevice *advertisedDevice)
{
    return advertisedDevice->isAdvertisingService(LEGO_WeDo_advertisingUUID);
}

bool l2fp_ConnectToHub(NimBLEClient *pClient)
{
    log_i("Connected to: %s", pClient->getPeerAddress().toString().c_str());
    log_i("RSSI: %d", pClient->getRssi());

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

bool l2fp_WriteMotorCommand(uint8_t wedo_port,int wedo_speed)
{
    //From http://www.ev3dev.org/docs/tutorials/controlling-wedo2-motor/
    //conversion from int (both pos and neg) to unsigned 8 bit int
    uint8_t speed_byte = wedo_speed;
    uint8_t command[] = {wedo_port, 0x01, 0x01, speed_byte};
    return pLEGOOutput->writeValue(command,sizeof(command));
}
