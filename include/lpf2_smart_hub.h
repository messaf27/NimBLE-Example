#ifndef __LPF2_SMART_HUB__
#define __LPF2_SMART_HUB__

#include "Arduino.h"
#include <NimBLEDevice.h>

#define LEGO_HUB_NAME_STR   "LPF2 Smart Hub"

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

enum MotorSpeedDir
{
    MSD_STOP =              0,
    MSD_SET_LEFT =          -65,
    MSD_SET_LEFT_DOUBLE =   -100,
    MSD_SET_RIGHT =         65,
    MSD_SET_RIGHT_DOUBLE =  100
};

enum LedSysBlinkStatus
{
    ledSt_CONNECTED,
    ledSt_CONNECTED_BAT_LOW,
    ledSt_DISCONNECTED,
    ledSt_DISCONNECTED_BAT_LOW,
    ledSt_SEARCH_CONTROLLER
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
#define RANGE_10            0
#define RANGE_100           1
#define RANGE_RAW           2

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

bool l2fp_SetClientData(WeDoHub_Client_t *dtClient);
bool l2fp_isMainService(NimBLEAdvertisedDevice *advertisedDevice);
bool l2fp_ConnectToHub(NimBLEClient *pClient);
bool l2fp_WriteMotorCommand(uint8_t wedo_port,int wedo_speed);

#endif /*__LPF2_SMART_HUB__*/