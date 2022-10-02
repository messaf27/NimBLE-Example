#ifndef __LPF2_SMART_HUB__
#define __LPF2_SMART_HUB__

#include "Arduino.h"
#include <NimBLEDevice.h>

#define configLEGO_HUB_DEFAULT_ADDRES                   "ff:ff:ff:ff:ff:ff"
#define configLEGO_HUB_DEFAULT_ADDRES_STR_SIZE          17
#define configLEGO_HUB_DEFAULT_NAME_STR                 "LPF2 Smart Hub"
#define configLEGO_HUB_DEFAULTDETECT_SENS_STOP_VALUE    (5)
#define configLEGO_HUB_SENSOR_POLL_PERIOD_MS            (1000)
#define configLEGO_HUB_TIMEOUT_MS_POWER_ON              (750)

typedef struct{
    int drvLeftVal;
    int drvLeftDoubleVal;
    int drvRightVal;
    int drvRightDoubleVal;
    int detectSensorStopValue;
}eeConstVal_t;

typedef struct
{
    bool DevConnected;
    bool DevSearch;
    String LinkDevAddress;
    uint8_t LedSysCurStatus;
    uint8_t PortNumTitleSensor;
    uint8_t PortNumDetectSensor;
    uint8_t DistanceCurrentValue;
    bool DriveOneEnabled;
    bool DriveTwoEnabled;
    eeConstVal_t eeConstConfig;

} WeDoHub_Client_t;

// #define WEDOHUB_CLIENT_SET_DEFAULT  {false, false, "", ledSt_DISCONNECTED, 0, 0, 0, false, false}
#define WEDOHUB_CLIENT_SET_DEFAULT  {}

enum DefaultConfigMotorSpeedDir
{
    MSD_STOP =              0,
    MSD_SET_LEFT =          -65,
    MSD_SET_LEFT_DOUBLE =   -100,
    MSD_SET_RIGHT =         65,
    MSD_SET_RIGHT_DOUBLE =  100
};

enum LedSysBlinkStatus
{
    ledSt_DISCONNECTED,
    ledSt_DISCONNECTED_BAT_LOW,
    ledSt_CONNECTED,
    ledSt_CONNECTED_BAT_LOW,
    ledSt_SEARCH_CONTROLLER,
    ledSt_LINK_HUB_ADDR_ACTION
};

typedef struct 
{
    uint8_t driveNum;
    int rotSpeedDir;
}
MotorTransCmd_t;

#define ID_MOTOR            1
#define ID_TILT_SENSOR      34
//0x23
#define ID_DETECT_SENSOR    35
//0x24
// #define RANGE_10            0
// #define RANGE_100           1
// #define RANGE_RAW           2

enum LedColors{
    LEGO_COLOR_BLACK = 0,
    LEGO_COLOR_PINK,        
    LEGO_COLOR_PURPLE,       
    LEGO_COLOR_BLUE,         
    LEGO_COLOR_CYAN,         
    LEGO_COLOR_LIGHTGREEN,   
    LEGO_COLOR_GREEN,        
    LEGO_COLOR_YELLOW,       
    LEGO_COLOR_ORANGE,       
    LEGO_COLOR_RED,          
    LEGO_COLOR_WHITE        
};

enum MotionSensorMode{
    MOTION_SENSOR_MODE_DETECT = 0,
    MOTION_SENSOR_MODE_COUNT = 1,
    MOTION_SENSOR_MODE_UNKNOWN = 2
};

enum TiltSensorMode{
    TILT_SENSOR_MODE_ANGLE = 0,
    TILT_SENSOR_MODE_TILT = 1,
    TILT_SENSOR_MODE_CRASH = 2,
    TILT_SENSOR_MODE_UNKNOWN = 4
};

enum PortFunctions{
    PORT_UNI_1 = 1,
    PORT_UNI_2 = 2,
    PORT_RGB_LED = 6
};

#include <EEPROM.h>

typedef struct {
    uint32_t eeSaveCounter;
    // String eeLinkDevAddress;
    char eeLinkDevAddress[17];
    int eeMotorConfig[4];
    int eeDetectSensorStopValue;
}ee_settings_t;

// bool l2fp_ReadEESettings(ee_settings_t *settings);
// bool l2fp_SaveEESettings(ee_settings_t *settings);
// bool l2fp_SetDefaultEESettings(ee_settings_t *settings);

bool l2fp_ReadEESettings(void);
bool l2fp_SaveEESettings(void);
bool l2fp_SetDefaultEESettings(void);
bool l2fp_LinkDevAddress(String linkAddr);

bool l2fp_InitClientConfig(WeDoHub_Client_t *dtClient);
bool l2fp_isMainService(NimBLEAdvertisedDevice *advertisedDevice);
bool l2fp_ConnectToHub(NimBLEClient *pClient);
bool l2fp_WriteMotorCommand(uint8_t wedo_port,int wedo_speed);
bool l2fp_WriteIndexColor(uint8_t color);
bool l2fp_WriteRGB(uint8_t red, uint8_t green, uint8_t blue);
bool l2fp_SetTiltSensor(uint8_t port);
bool l2fp_SetDetectSensor(uint8_t port);
bool l2fp_IsPermissibleDistance(void);
void l2fp_DetectSensorAction(void);
bool l2fp_CriticalDistance(void);

#endif /*__LPF2_SMART_HUB__*/