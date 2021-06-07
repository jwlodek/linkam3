#include "asynPortDriver.h"
#include <epicsEvent.h>

#define P_TempString          "LINKAM_TEMP"
#define P_RampRateSetString   "LINKAM_RAMPRATE_SET"
#define P_RampRateString      "LINKAM_RAMPRATE"
#define P_SetpointSetString   "LINKAM_SETPOINT_SET"
#define P_SetpointString      "LINKAM_SETPOINT"
#define P_PowerString         "LINKAM_POWER"
#define P_StartHeatingString  "LINKAM_START_HEATING"
#define P_LNPSpeedString      "LINKAM_LNP_SPEED"
#define P_DSCString           "LINKAM_DSC"
#define P_LNPSetModeString    "LINKAM_LNP_MODE_SET"
#define P_LNPSetSpeedString   "LINKAM_LNP_SPEED_SET"
#define P_HoldTimeSetString   "LINKAM_HOLD_TIME_SET"
#define P_HoldTimeLeftString  "LINKAM_HOLD_TIME_LEFT"
#define P_NameString          "LINKAM_NAME"
#define P_SerialString        "LINKAM_SERIAL"
#define P_StageNameString     "LINKAM_STAGE_NAME"
#define P_StageSerialString   "LINKAM_STAGE_SERIAL"
#define P_FirmVerString       "LINKAM_FIRM_VERSION"
#define P_HardVerString       "LINKAM_HARD_VERSION"
#define P_CtrllrErrorString   "LINKAM_CTRLLR_ERROR"
#define P_CtrlConfigString    "LINKAM_CONFIG"
#define P_CtrlStatusString    "LINKAM_STATUS"
#define P_StageConfigString   "LINKAM_STAGE_CONFIG"

class Linkam3 : public asynPortDriver {

public:

    Linkam3(const char* portName, const char* connectionType, 
            unsigned int vendorID, unsigned int productID, const char* serialPort,
            const char* licenseFilePath, const char* logFilePath);

    ~Linkam3();

    virtual asynStatus readFloat64(asynUser *, epicsFloat64 *);
    virtual asynStatus readOctet(asynUser *, char *, size_t, size_t *, int *);
    virtual asynStatus writeFloat64(asynUser *, epicsFloat64);
    virtual asynStatus writeInt32(asynUser *, epicsInt32);
    virtual asynStatus readInt32(asynUser *, epicsInt32 *);

protected:

    //epicsEventId eventId_;
    int P_Temp;
    #define FIRST_LINKAM_COMMAND P_Temp
    int P_RampRateSet;
    int P_RampRate;
    int P_SetpointSet;
    int P_Setpoint;
    int P_Power;
    int P_StartHeating;
    int P_LNPSpeed;
    int P_DSC;
    int P_LNPSetMode;
    int P_LNPSetSpeed;
    int P_HoldTimeSet;
    int P_HoldTimeLeft;
    int P_StageName;
    int P_StageSerial;
    int P_FirmVer;
    int P_HardVer;
    int P_CtrllrError;
    int P_CtrlConfig;
    int P_CtrlStatus;
    int P_StageConfig;
    int P_Name;
    int P_Serial;
    #define LAST_LINKAM_COMMAND P_Serial

    // Connection functions
    bool initUSBConnection(unsigned int vendorID, unsigned int productID);
    bool initSerialConnection(const char* serialPort, 
                                unsigned int baudrate, 
                                unsigned int bytesize, 
                                unsigned int flowcontrol, 
                                unsigned int parity, 
                                unsigned int stopbits);

    // Status printing functions
    void printErrorConnectionStatus(LinkamSDK::Variant connectionResult);
    void printLinkam3Status();


private:
    void rtrim(char *);
    bool LNP_AutoMode;
    int LNP_ManualSpeed;

    // Handle on the device
    CommsHandle handle = 0;
};

#define NUM_LINKAM_PARAMS (&LAST_LINKAM_COMMAND - &FIRST_LINKAM_COMMAND + 1)

