
#include "lpf2_smart_hub.h"
#include "esp32-hal-log.h"

WeDoHub_Client_t *hubClientData;

// EEPROM Settings struct;
ee_settings_t eeSettigs = {};

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


static const char* getLedColorStr(uint8_t *pData){
    switch(pData[2])
    {
        case LEGO_COLOR_BLACK:          return "BLACK";        
        case LEGO_COLOR_PINK:           return "PINK";         
        case LEGO_COLOR_PURPLE:         return "PURPLE"; 
        case LEGO_COLOR_BLUE:           return "BLUE"; 
        case LEGO_COLOR_CYAN:           return "CYAN"; 
        case LEGO_COLOR_LIGHTGREEN:     return "LIGHTGREEN";
        case LEGO_COLOR_GREEN:          return "GREEN";
        case LEGO_COLOR_YELLOW:         return "YELLOW"; 
        case LEGO_COLOR_ORANGE:         return "ORANGE"; 
        case LEGO_COLOR_RED:            return "RED"; 
        case LEGO_COLOR_WHITE:          return "WHITE"; 
        default:
            return "Unknown color!!!";
    }              
} 

static uint8_t parseDigit_0_to_10(uint8_t *pData)
{
    if(pData[4] == 0 && pData[5] == 0)
        return 0;
    else 
    if(pData[4] == 0x80 && pData[5] == 0x3F)
        return 1;
    else
    if(pData[4] == 0x00 && pData[5] == 0x40)
        return 2;
    else
    if(pData[4] == 0x40 && pData[5] == 0x40)
        return 3;
    else
    if(pData[4] == 0x80 && pData[5] == 0x40)
        return 4;
    else
    if(pData[4] == 0xA0 && pData[5] == 0x40)
        return 5;
    else
    if(pData[4] == 0xC0 && pData[5] == 0x40)
        return 6;   
    else
    if(pData[4] == 0xE0 && pData[5] == 0x40)
        return 7;          
    else
    if(pData[4] == 0x00 && pData[5] == 0x41)
        return 8;
    else
    if(pData[4] == 0x10 && pData[5] == 0x41)
        return 9;     
    else
    if(pData[4] == 0x20 && pData[5] == 0x41)
        return 10; 
    
    return 0;                                 
}


/////////////////////////////////////////////////////////////////////////////////
bool l2fp_InitClientConfig(WeDoHub_Client_t *dtClient)
{
    if(dtClient == NULL)
        return false;

    if(!l2fp_ReadEESettings())
        return false;

    hubClientData = dtClient;

    hubClientData->LinkDevAddress = String(eeSettigs.eeLinkDevAddress);    
    hubClientData->eeConstConfig.drvLeftVal = eeSettigs.eeMotorConfig[0];
    hubClientData->eeConstConfig.drvLeftDoubleVal = eeSettigs.eeMotorConfig[1];
    hubClientData->eeConstConfig.drvRightVal = eeSettigs.eeMotorConfig[2];
    hubClientData->eeConstConfig.drvRightDoubleVal = eeSettigs.eeMotorConfig[3];
    hubClientData->DistanceCurrentValue = configLEGO_HUB_DEFAULT_DETECT_SENS_MAX_VALUE; // Для начальной инициализации
    
    return true;
}

bool l2fp_ReadEESettings(void)
{
    bool ret = false;

    ret = EEPROM.begin(512);
    EEPROM.get(0, eeSettigs);
    EEPROM.end();
    log_i("EE-Settings.eeLinkDevAddress = %s",eeSettigs.eeLinkDevAddress);
    log_i("EE-Settings.eeMotorConfig = [%d,%d,%d,%d]", eeSettigs.eeMotorConfig[0], eeSettigs.eeMotorConfig[1], eeSettigs.eeMotorConfig[2], eeSettigs.eeMotorConfig[3]);
    log_i("EE-Settings.eeSaveCounter = %ld", eeSettigs.eeSaveCounter);
    return ret;
}

bool l2fp_SaveEESettings(void)
{
    bool ret = false;

    ret = EEPROM.begin(512);
    eeSettigs.eeSaveCounter++;
    EEPROM.put(0, eeSettigs);
    EEPROM.commit();
    EEPROM.end();
    
    return ret;
}

bool l2fp_SetDefaultEESettings(void)
{
    eeSettigs.eeSaveCounter = 0; // reset save eeprom conter
    String(configLEGO_HUB_DEFAULT_ADDRES).toCharArray(eeSettigs.eeLinkDevAddress, configLEGO_HUB_DEFAULT_ADDRES_STR_SIZE + 1); //"ff:ff:ff:ff:ff:ff";
    eeSettigs.eeMotorConfig[0] = MSD_SET_LEFT;
    eeSettigs.eeMotorConfig[1] = MSD_SET_LEFT_DOUBLE;
    eeSettigs.eeMotorConfig[2] = MSD_SET_RIGHT;
    eeSettigs.eeMotorConfig[3] = MSD_SET_RIGHT_DOUBLE;
    eeSettigs.eeDetectSensorStopValue = configLEGO_HUB_DEFAULT_DETECT_SENS_STOP_VALUE;

    return l2fp_SaveEESettings();
}

bool l2fp_LinkDevAddress(String linkAddr)
{
    linkAddr.toCharArray(eeSettigs.eeLinkDevAddress, configLEGO_HUB_DEFAULT_ADDRES_STR_SIZE + 1);
    return l2fp_SaveEESettings();
}

/** Notification / Indication receiving handler callback */
void l2fp_notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    if(pRemoteCharacteristic->getUUID() == LEGOButton)
    {
        log_i("Hub Button: %s", (pData[0] == 1) ? "ON" : "OFF");
        if(pData[0] == 1)
            hubClientData->LedSysCurStatus = ledSt_SEARCH_CONTROLLER;
    }else 
    if(pRemoteCharacteristic->getUUID() == LEGOSensor){
        uint8_t SensOnCurrPort = pData[1];

        switch (SensOnCurrPort) // Check port function
        {
            case PORT_UNI_1:
            case PORT_UNI_2:
            
                if(SensOnCurrPort == hubClientData->PortNumDetectSensor){
                    hubClientData->DistanceCurrentValue = parseDigit_0_to_10(pData);
                    log_i("Distance: %d [port %d]", hubClientData->DistanceCurrentValue, hubClientData->PortNumDetectSensor);
                }else 
                if(SensOnCurrPort == hubClientData->PortNumTitleSensor){
                    // log_i("TitleSensor data: [");
                    // for (int i = 0; i < length; i++){
                    //     log_i("%02x", pData[i]);
                    // }
                    // log_i("] (Size: %d, port: %d)", length, SensOnCurrPort);
                } 
            break;

            case PORT_RGB_LED:
                log_i("Led color set to: %s", getLedColorStr(pData));
            break;  

            default:
                log_i("Undefined port!!!");
            break;
        }
    }else
    if(pRemoteCharacteristic->getUUID() == LEGOPorts){
        uint8_t changePort = pData[0];
        if(pData[1] == 0){
            log_i("Port %d disconnected", changePort);

            if(hubClientData->PortNumDetectSensor == changePort)
                hubClientData->PortNumDetectSensor = 0;
            else
            if(hubClientData->PortNumTitleSensor == changePort)
                hubClientData->PortNumTitleSensor = 0;
            else
            if(hubClientData->DriveOneEnabled && (changePort == 1))
                hubClientData->DriveOneEnabled = false;    
            else
            if(hubClientData->DriveTwoEnabled && (changePort == 2))
                hubClientData->DriveTwoEnabled = false;        

        }else if(pData[1] == 1){
            
            switch(pData[3])
            {
                case ID_MOTOR: log_i("Motor connected on port %d", changePort);

                if(changePort == 1)
                    hubClientData->DriveOneEnabled = true;
                else 
                if(changePort == 2)   
                    hubClientData->DriveTwoEnabled = true;

                break;

                case ID_DETECT_SENSOR: log_i("DetectSensor connected on port %d", changePort);
                    hubClientData->PortNumDetectSensor = changePort;
                break;

                case ID_TILT_SENSOR: log_i("TitleSensor connected on port %d", changePort);
                    hubClientData->PortNumTitleSensor  = changePort;
                    break;

                default: 
                break;
            }
        }
    }
}

bool l2fp_isMainService(NimBLEAdvertisedDevice *advertisedDevice)
{
    return advertisedDevice->isAdvertisingService(LEGO_WeDo_advertisingUUID);
}

bool l2fp_ClientInitHub(NimBLEClient *pClient)
{
    log_i("Connected to: %s", pClient->getPeerAddress().toString().c_str());
    log_i("RSSI: %d", pClient->getRssi());

    hubClientData->LedSysCurStatus = ledSt_CONNECTING_INIT_HUB;

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
                log_i("UUID: %s, Bat Level: %d%%", 
                        pBatChr->getUUID().toString().c_str(), 
                        // pBatChr->readUInt8());
                        pBatChr->readValue<uint8_t>());
            }
        }
    }
    else
    {
        log_i("Not found: %s", LEGO_HUB_BatteryServiceUUID.toString().c_str());
        return false;
    }

    pAdvertisingService = pClient->getService(LEGO_WeDo_advertisingUUID);
    if(pAdvertisingService == nullptr) {     /** убедитесь, что это не нольl */
        log_i("Failed to find our service UUID: %s", LEGO_WeDo_advertisingUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    log_i("Found our service: %s", LEGO_WeDo_advertisingUUID.toString().c_str());
    
    pLEGOPorts = pAdvertisingService->getCharacteristic(LEGOPorts);
    if(pLEGOPorts == nullptr) {     /** убедитесь, что это не нольl */
        log_i("Failed to find our service UUID (LEGOPorts): %s",LEGOPorts.toString().c_str());
        pClient->disconnect();
        return false;
    }
    log_i("Found our service: %s", LEGOPorts.toString().c_str());


    pLEGO = pClient->getService(LEGO);
    if(pLEGO == nullptr) {     /** убедитесь, что это не нольl */
        log_i("Failed to find our service UUID: %", LEGO.toString().c_str());
        pClient->disconnect();
        return false;
    }
    log_i("Found our service: %s", LEGO.toString().c_str());

    pLEGOOutput = pLEGO->getCharacteristic(LEGOOutput);
    if (pLEGOOutput == nullptr) {
        log_i("Failed to find our characteristic UUID: %s", LEGOOutput.toString().c_str());
        pClient->disconnect();
        return false;
    }       
    log_i("Found our characteristic: %s", LEGOOutput.toString().c_str());

    // Индикация (проверка) инициализации портов на выход.
    l2fp_WriteIndexColor(LEGO_COLOR_ORANGE);

    pLEGOInput = pLEGO->getCharacteristic(LEGOInput);
    if (pLEGOInput == nullptr) {
        log_i("Failed to find our characteristic UUID: %d", LEGOInput.toString().c_str());
        pClient->disconnect();
        return false;
    }
    log_i("Found our characteristic: %s", LEGOInput.toString().c_str());

    pLEGOButton = pAdvertisingService->getCharacteristic(LEGOButton);
    if (pLEGOButton == nullptr) {
        log_i("Failed to find our characteristic UUID: %s", LEGOButton.toString().c_str());
        pClient->disconnect();
        return false;
    }
    log_i("Found our characteristic: %s", LEGOButton.toString().c_str());

    if(pLEGOButton->canNotify()){
        if(!pLEGOButton->subscribe(true, l2fp_notifyCB)) {
            /** Disconnect if subscribe failed */
            pClient->disconnect();
            return false;
        }
        log_i("Subscribe to LEGOButton OK");
    }

    if(pLEGOPorts->canNotify()) {
        //if(!pChr->registerForNotify(notifyCB)) {
        if(!pLEGOPorts->subscribe(true, l2fp_notifyCB)) {
            /** Disconnect if subscribe failed */
            pClient->disconnect();
            return false;
        }
        log_i("Subscribe pLEGOPorts Notify");
    }

    // Init notify Sensors
    // Read the value of the characteristic.
    pLEGOSensor = pLEGO->getCharacteristic(LEGOSensor);
    if (pLEGOSensor == nullptr) {
        log_i("Failed to find our characteristic UUID: %s", LEGOSensor.toString().c_str());
        return false;
    }
    log_i("Found our characteristic: %s", LEGOSensor.toString().c_str());

    if(pLEGOSensor->canNotify()) 
    {
        if(!pLEGOSensor->subscribe(true, l2fp_notifyCB)) 
            return false;

        log_i("Subscribe Sensor Notify");
    }    

    log_i("Done with this device!");
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

bool l2fp_WriteIndexColor(uint8_t color)
{
    //From http://ofalcao.pt/blog/2016/wedo-2-0-colors-with-python
    uint8_t command[] = {0x06, 0x04, 0x01, color};
    return pLEGOOutput->writeValue(command,sizeof(command));
}

bool l2fp_WriteRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t command[] = {0x06, 0x04 ,0x03, red,green,blue};
    return pLEGOOutput->writeValue(command,sizeof(command));
}

bool l2fp_SetTiltSensor(uint8_t port)
{
  uint8_t command[] = {0x01, 0x02, port, ID_TILT_SENSOR, 0, 0x01, 0x00, 0x00, 0x00, TILT_SENSOR_MODE_TILT, 0x01};
  return pLEGOInput->writeValue(command,sizeof(command));
}

bool l2fp_SetDetectSensor(uint8_t port)
{
  uint8_t command[] = {0x01, 0x02, port, ID_DETECT_SENSOR, 0, 0x01, 0x00, 0x00, 0x00, MOTION_SENSOR_MODE_DETECT, 0x01};
  return pLEGOInput->writeValue(command,sizeof(command));
}

bool l2fp_IsPermissibleDistance(void)
{
    if(hubClientData->DistanceCurrentValue <= eeSettigs.eeDetectSensorStopValue)
        return false;

    return true;
}

void l2fp_DetectSensorAction(void)
{
    static bool setLedFlag;
    if(!l2fp_IsPermissibleDistance() && !setLedFlag)
    {
        l2fp_WriteIndexColor(LEGO_COLOR_RED);
        setLedFlag = true;
    }
    else
    if(l2fp_IsPermissibleDistance() && setLedFlag)
    {
        l2fp_WriteIndexColor(LEGO_COLOR_GREEN);
        setLedFlag = false;
    }
} 

bool l2fp_CriticalDistance(void)
{
    static bool actionFlag;

    if(!l2fp_IsPermissibleDistance() && !actionFlag)
    {
        actionFlag = true;
        return true;
    }
    else if(l2fp_IsPermissibleDistance() && actionFlag)
        actionFlag = false;

    return false;
}