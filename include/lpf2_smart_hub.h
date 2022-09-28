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

typedef struct 
{
    uint8_t driveNum;
    int rotSpeedDir;
}
MotorTransCmd_t;

bool l2fp_isMainService(NimBLEAdvertisedDevice *advertisedDevice);
bool l2fp_ConnectToHub(NimBLEClient *pClient);
bool l2fp_WriteMotorCommand(uint8_t wedo_port,int wedo_speed);

#endif /*__LPF2_SMART_HUB__*/