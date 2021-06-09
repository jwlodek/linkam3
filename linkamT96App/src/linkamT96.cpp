#include <epicsExport.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <iocsh.h>
#include <stdio.h>
#include <string.h>
#include "LinkamSDK.h"
#include "CommsAPI.h"
#include "linkamT96.h"


static const char *driverName = "Linkam3";


// Error message formatters
#define ERR(msg) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", \
    driverName, functionName, msg)

#define ERR_ARGS(fmt,...) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
    "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);

// Flow message formatters
#define LOG(msg) asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", \
    driverName, functionName, msg)

#define LOG_ARGS(fmt,...) asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, \
    "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);



/**
 * Callback function called when IOC is terminated.
 * Deletes created object and frees UVC context
 *
 * @params[in]: pPvt -> pointer to the Linkam3 object created in Linkam3Connect
 */
static void exitCallbackC(void* pPvt) {
    Linkam3* pLinkam3 = (Linkam3*) pLinkam3;
    delete(pLinkam3);
}


/**
 * Helper function that prints information about a failed connection to the device
 * 
 * @params[in] connectionResult -> Variant union that contains information about the failed connection
 */
void Linkam3::printErrorConnectionStatus(LinkamSDK::Variant connectionResult){
    printf
    (
        "Error opening connection:\n\nstatus.connected = %d\n\
        status.flags.errorAllocationFailed = %d\n\
        status.flags.errorAlreadyOpen = %d\n\
        status.flags.errorCommsStreams = %d\n\
        status.flags.errorHandleRegistrationFailed = %d\n\
        status.flags.errorMultipleDevicesFound = %d\n\
        status.flags.errorNoDeviceFound = %d\n\
        status.flags.errorPortConfig = %d\n\
        status.flags.errorPropertiesIncorrect = %d\n\
        status.flags.errorSerialNumberRequired = %d\n\
        status.flags.errorTimeout = %d\n\
        status.flags.errorUnhandled = %d\n\n",
        connectionResult.vConnectionStatus.flags.connected,
        connectionResult.vConnectionStatus.flags.errorAllocationFailed,
        connectionResult.vConnectionStatus.flags.errorAlreadyOpen,
        connectionResult.vConnectionStatus.flags.errorCommsStreams,
        connectionResult.vConnectionStatus.flags.errorHandleRegistrationFailed,
        connectionResult.vConnectionStatus.flags.errorMultipleDevicesFound,
        connectionResult.vConnectionStatus.flags.errorNoDeviceFound,
        connectionResult.vConnectionStatus.flags.errorPortConfig,
        connectionResult.vConnectionStatus.flags.errorPropertiesIncorrect,
        connectionResult.vConnectionStatus.flags.errorSerialNumberRequired,
        connectionResult.vConnectionStatus.flags.errorTimeout,
        connectionResult.vConnectionStatus.flags.errorUnhandled
    );
}


asynStatus Linkam3::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    int function = pasynUser->reason;
    epicsTimeStamp timeStamp;
    const char *functionName = "readFloat64";
    asynStatus status = asynSuccess;

    getTimeStamp(&timeStamp);
    pasynUser->timestamp = timeStamp;

    if (function == P_Temp) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeater1Temp;
    } else if (function == P_RampRate) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeaterRate;
    } else if (function == P_Setpoint) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeaterSetpoint;
    } else if (function == P_Power) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeater1Power;
    } else if (function == P_LNPSpeed) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeater1LNPSpeed;
    } else if (function == P_DSC) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeDsc;
    } else if (function == P_HoldTimeLeft) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeRampHoldRemaining;
    } 

    if (linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetValue, handle, &result, param1, param2))
        *value = result.vFloat32;
    else
        status = asynError;

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
            "%s:%s: status=%d, function=%d",
            driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d\n",
            driverName, functionName, function);

    return status;
}

asynStatus Linkam3::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    enum LinkamSDK::LinkamFunctionMsgCode linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerName;
    char string[256];
    int function = pasynUser->reason;
    epicsTimeStamp timeStamp;
    const char *functionName = "readOctet";
    (void)functionName;
    asynStatus status = asynSuccess;

    getTimeStamp(&timeStamp);
    pasynUser->timestamp = timeStamp;

    if (function == P_Serial) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerSerial;
    } else if (function == P_Name) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerName;
    } else if (function == P_StageSerial) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetStageSerial;
    } else if (function == P_StageName) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetStageName;
    } else if (function == P_FirmVer) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerFirmwareVersion;
    } else if (function == P_HardVer) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerHardwareVersion;
    } else if (function == P_CtrllrError) {
        linkamMsgCode = LinkamSDK::eLinkamFunctionMsgCode_GetControllerError;
    }

    if(function == P_CtrllrError){
        if (linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetControllerError, handle, &result)) {
            strcpy(value, LinkamSDK::ControllerErrorStrings[result.vControllerError]);
            *eomReason = 0;
        } 
        else {
            status = asynError;
        }
    } 
    else {
        result.vUint64 = 0;
        param1.vPtr = string;
        param2.vUint32 = 256;
        if (linkamProcessMessage(linkamMsgCode, handle, &result, param1, param2)) {
            rtrim(string);	  
            setStringParam(function, string);

            *nActual = strlen(string) + 1;
            *eomReason = 0;

        }
        else {
            status = asynError;
        }
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
            "%s:%s: status=%d, function=%d",
            driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d\n",
            driverName, functionName, function);

    return status; 
}

/**
 * Helper function for trimming whitespace from the end of a message.
 * 
 * @params[out]: s -> The string to trim
 */
void Linkam3::rtrim(char *s) {
    int i;
    for (i = strlen(s); s[i-1] == ' ' || s[i-1] == '\t'; --i)
        ;
    s[i] = '\0';
}



asynStatus Linkam3::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    int function = pasynUser->reason;
    const char *functionName = "writeFloat64";
    asynStatus status = asynSuccess;

    if (function == P_RampRateSet) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeaterRate;
    }
    else if (function == P_SetpointSet) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeHeaterSetpoint;
    }
    else if (function == P_HoldTimeSet) {
        param1.vStageValueType = LinkamSDK::eStageValueTypeRampHoldTime;
    }

    param2.vFloat32 = value;
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_SetValue, handle, &result, param1, param2);

    if (!result.vBoolean) {
        status = asynError;
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
            "%s:%s: status=%d, function=%d",
            driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d\n",
            driverName, functionName, function);

    return status;
}


asynStatus Linkam3::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    int function = pasynUser->reason;
    const char *functionName = "writeInt32";
    asynStatus status = asynSuccess;

    if (function == P_StartHeating) {
        param2.vUint64 = 0; /* unused */

        if (value > 0) param1.vBoolean = true;
        else param1.vBoolean = false;

        linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_StartHeating,
                             handle, &result, param1, param2);
        
        if (!result.vBoolean) {
            status = asynError;
        }
    }
    else if (function == P_LNPSetMode) {
        param2.vUint64 = 0; /* unused */

        if (value > 0) LNP_AutoMode = param1.vBoolean = true;
        else LNP_AutoMode = param1.vBoolean = false;

        linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_LnpSetMode, handle, &result, param1, param2);

        if (result.vBoolean) {
            if (!LNP_AutoMode) { /* Manual Mode, set LNP speed to LNP_ManualSpeed */
                param1.vUint32 = LNP_ManualSpeed;

                linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_LnpSetSpeed,
                                     handle, &result, param1, param2);
            }
        }
        else {
            status = asynError;
        }
    }
    else if (function == P_LNPSetSpeed) {
        param2.vUint64 = 0; /* unused */

        if (value < 0) value = 0;
        else if (value > 100) value = 100;

        LNP_ManualSpeed = value;

        if (!LNP_AutoMode) {

            param1.vUint32 = value;

            linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_LnpSetSpeed, handle, &result, param1, param2);

            if (!result.vBoolean) {
                status = asynError;
            }
        }
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
            "%s:%s: status=%d, function=%d",
            driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d\n",
            driverName, functionName, function);

    return status;
}

asynStatus Linkam3::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    int function = pasynUser->reason;
    const char *functionName = "readInt32";
    asynStatus status = asynSuccess;

    if (function == P_CtrlConfig) {
        if (linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetControllerConfig, handle, &result)) {
            *value =
              /* result.vControllerConfig.flags.supportsHeater                      << 0  |
                 result.vControllerConfig.flags.supportsDualHeater                  << 1  |
                 result.vControllerConfig.flags.supportsDualHeaterIndependentLimits << 2  |
                 result.vControllerConfig.flags.supportsDualHeaterIndependentRates  << 3  |
                 result.vControllerConfig.flags.vacuumOption                        << 4  |
                 result.vControllerConfig.flags.tensileForceCardReady               << 5  |*/
                 result.vControllerConfig.flags.dscCardReady                        << 0  |
               /*result.vControllerConfig.flags.xMotorCardReady                     << 7  |
                 result.vControllerConfig.flags.yMotorCardReady                     << 8  |
                 result.vControllerConfig.flags.zMotorCardReady                     << 9  |
                 result.vControllerConfig.flags.motorValveCardReady                 << 7  |
                 result.vControllerConfig.flags.tensileMotorCardReady               << 8  |
                 result.vControllerConfig.flags.gradedMotorCardReady                << 9  |
                 result.vControllerConfig.flags.dtcCardReady                        << 7  |
                 result.vControllerConfig.flags.cssMotorCardReady                   << 8  |*/
                 result.vControllerConfig.flags.lnpReady                            << 1/*|
                 result.vControllerConfig.flags.lnpDualReady                        << 2  |
                 result.vControllerConfig.flags.humidityReady                       << 3*/;
        } else {
            status = asynError;
        }
    } 
    else if (function == P_CtrlStatus) {
        if (linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetStatus, handle, &result)) { 
            *value = result.vControllerStatus.flags.controllerError               << 0  |
                 result.vControllerStatus.flags.heater1RampSetPoint           << 1  |
                 result.vControllerStatus.flags.heater1Started                << 2  |
                   /*result.vControllerStatus.flags.heater2RampSetPoint           << 3  |
                 result.vControllerStatus.flags.heater2Started                << 4  |
                 result.vControllerStatus.flags.vacuumRampSetPoint            << 5  |
                 result.vControllerStatus.flags.vacuumCtrlStarted             << 6  |
                 result.vControllerStatus.flags.vacuumValveClosed             << 7  |
                 result.vControllerStatus.flags.vacuumValveOpen               << 8  |
                 result.vControllerStatus.flags.humidityRampSetPoint          << 3  |
                 result.vControllerStatus.flags.humidityCtrlStarted           << 4  |*/
                 result.vControllerStatus.flags.lnpCoolingPumpOn              << 3  |
                 result.vControllerStatus.flags.lnpCoolingPumpAuto            << 4  |
                   /*result.vControllerStatus.flags.HumidityDesiccantConditioning << 13 |
                 result.vControllerStatus.flags.motorTravelMinX               << 14 |
                 result.vControllerStatus.flags.motorTravelMaxX               << 15 |
                 result.vControllerStatus.flags.motorStoppedX                 << 16 |
                 result.vControllerStatus.flags.motorTravelMinY               << 17 |
                 result.vControllerStatus.flags.motorTravelMaxY               << 18 |
                 result.vControllerStatus.flags.motorStoppedY                 << 19 |
                 result.vControllerStatus.flags.motorTravelMinZ               << 20 |
                 result.vControllerStatus.flags.motorTravelMaxZ               << 21 |
                 result.vControllerStatus.flags.motorStoppedZ                 << 22 |*/
                 result.vControllerStatus.flags.sampleCal                     << 5;
                   /*result.vControllerStatus.flags.motorDistanceCalTST           << 24 |
                 result.vControllerStatus.flags.cssRotMotorStopped            << 8  |
                 result.vControllerStatus.flags.cssGapMotorStopped            << 9  |
                 result.vControllerStatus.flags.cssLidOn                      << 10 |
                 result.vControllerStatus.flags.cssRefLimit                   << 11 |
                 result.vControllerStatus.flags.cssZeroLimit                  << 12;*/
            if (result.vControllerStatus.flags.controllerError){
                printf("Controller Error: %i\n", linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetControllerError, handle, &result));
            }
        }
        else {
            status = asynError;
        }
    }
    else if (function == P_StageConfig) {
        if (linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetStageConfig, handle, &result)) {
            *value = result.vStageConfig.flags.standardStage               << 0  |
                   /*result.vStageConfig.flags.highTempStage               << 1  |
                 result.vStageConfig.flags.peltierStage                << 2  |
                 result.vStageConfig.flags.gradedStage                 << 3  |
                 result.vStageConfig.flags.tensileStage                << 4  |*/
                 result.vStageConfig.flags.dscStage                    << 1  |
                   /*result.vStageConfig.flags.warmStage                   << 6  |
                 result.vStageConfig.flags.itoStage                    << 7  |
                 result.vStageConfig.flags.css450Stage                 << 8  |
                 result.vStageConfig.flags.correlativeStage            << 9  |*/
                 result.vStageConfig.flags.coolingManual               << 2  |
                 result.vStageConfig.flags.coolingAutomatic            << 3  |
                   /*result.vStageConfig.flags.coolingDual                 << 12 |
                 result.vStageConfig.flags.coolingDualSpeedIndependent << 13 |
                 result.vStageConfig.flags.heater1                     << 14 |
                 result.vStageConfig.flags.heater1TempCtrl             << 15 |
                 result.vStageConfig.flags.heater1TempCtrlProbe        << 16 |
                 result.vStageConfig.flags.heater2                     << 17 |
                 result.vStageConfig.flags.heater12IndependentLimits   << 18 |
                 result.vStageConfig.flags.waterCoolingSensorFitted    << 19 |
                 result.vStageConfig.flags.home                        << 20 |
                 result.vStageConfig.flags.supportsVacuum              << 21 |
                 result.vStageConfig.flags.motorX                      << 22 |
                 result.vStageConfig.flags.motorY                      << 23 |
                 result.vStageConfig.flags.motorZ                      << 24 |*/
                 result.vStageConfig.flags.supportsHumidity            << 4;
        }
        else {
            status = asynError;
        }
    } 

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
            "%s:%s: status=%d, function=%d",
            driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d\n",
            driverName, functionName, function);

    return status;
}


/**
 * Function for connecting to the Linkam device via USB
 * 
 * @params[in]: vendorID -> The Linkam vendor ID (0x16DA)
 * @params[in]: productID -> The product ID, typically set to discover mode (0x0001)
 * @returns: true if connected successfully, false otherwise
 */
bool Linkam3::initUSBConnection(unsigned int vendorID, unsigned int productID) {

    const char* functionName = "initUSBConnection";
    LinkamSDK::CommsInfo info;
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    bool connected = false;

    param1.vUint32 = LOGGING_LEVEL_MINIMAL;//LOGGING_LEVEL_INVESTIGATION;
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_EnableLogging, 0, &result, param1, param2);
    linkamInitialiseUSBCommsInfo(&info, NULL);

    if (vendorID != 0x0) {
        LinkamSDK::USBCommsInfo* usb = reinterpret_cast<LinkamSDK::USBCommsInfo*>(info.info);
        usb->vendorID = (uint16_t) vendorID;
        printf("USB: Vendor[%X] Product[%d]\n", usb->vendorID, usb->productID);
    }
    else if (productID != 0x0) {
        LinkamSDK::USBCommsInfo* usb = reinterpret_cast<LinkamSDK::USBCommsInfo*>(info.info);
        usb->productID = (uint16_t) productID;
        printf("USB: Vendor[%X] Product[%d]\n", usb->vendorID, usb->productID);
    }

    param1.vPtr = &info;
    param2.vPtr = &handle;
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_OpenComms, 0, &result, param1, param2);
    if (result.vConnectionStatus.flags.connected) {
        LOG("We got a connection to the USB device!");
        connected = true;
    }
    else {
        ERR("Failed to obtain a USB connection!");
        printErrorConnectionStatus(result);
    }

    return connected;
}


/**
 * Function for connecting to the Linkam device via Serial
 * 
 * @params[in] serialPort -> Name of the serial port to connect to
 * @params[in]: baudrate -> The seed of the serial connection
 * @params[in]: bytesize -> How big the serial byte is, must be 8 bits
 * @params[in]: flowcontrol -> Type of flow control
 * @params[in]: parity -> Use of the parity bit
 * @params[in]: stopbits -> Number of bits to indicate a stop
 * @returns: true if connected successfully, false otherwise
 */
bool Linkam3::initSerialConnection(const char* serialPort, 
                                    unsigned int baudrate, 
                                    unsigned int bytesize, 
                                    unsigned int flowcontrol, 
                                    unsigned int parity, 
                                    unsigned int stopbits){

    const char* functionName = "initSerialConnection";

    LinkamSDK::CommsInfo info;
    LinkamSDK::Variant param1;
    LinkamSDK::Variant param2;
    LinkamSDK::Variant result;
    bool connected = false;

    param1.vUint32 = LOGGING_LEVEL_MINIMAL;//LOGGING_LEVEL_INVESTIGATION;
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_EnableLogging, 0, &result, param1, param2);
    
    // Change #if 1 to (0) to enable serial loopback testing.
#if 1
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_DisableSerialLoopbackTest, 0, &result, param1, param2);
#else
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_EnableSerialLoopbackTest, 0, &result, param1, param2);    
#endif

    linkamInitialiseSerialCommsInfo(&info, serialPort);

    if (baudrate != 0x0) {
        LinkamSDK::SerialCommsInfo* serial = reinterpret_cast<LinkamSDK::SerialCommsInfo*>(info.info);
        serial->baudrate = (uint32_t) baudrate;
        printf("SERIAL: Setting baudrate to [%d]\n", serial->baudrate);
    }

    if (bytesize != 0x0) {
        LinkamSDK::SerialCommsInfo* serial = reinterpret_cast<LinkamSDK::SerialCommsInfo*>(info.info);
        serial->bytesize = (LinkamSDK::ByteSize) bytesize;
        printf("SERIAL: Setting bytesize to [%d]\n", serial->bytesize);
    }

    if (flowcontrol != 0x0) {
        LinkamSDK::SerialCommsInfo* serial = reinterpret_cast<LinkamSDK::SerialCommsInfo*>(info.info);
        serial->flowcontrol = (LinkamSDK::FlowControl) flowcontrol;
        printf("SERIAL: Setting flowcontrol to [%d]\n", serial->flowcontrol);
    }

    if (parity != 0x0) {
        LinkamSDK::SerialCommsInfo* serial = reinterpret_cast<LinkamSDK::SerialCommsInfo*>(info.info);
        serial->parity = (LinkamSDK::Parity) parity;
        printf("SERIAL: Setting parity to [%d]\n", serial->parity);
    }

    if (stopbits != 0x0) {
        LinkamSDK::SerialCommsInfo* serial = reinterpret_cast<LinkamSDK::SerialCommsInfo*>(info.info);
        serial->stopbits = (LinkamSDK::Stopbits) stopbits;
        printf("SERIAL: Setting stopbits to [%d]\n", serial->stopbits);
    }

    param1.vPtr = &info;
    param2.vPtr = &handle;

    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_OpenComms, 0, &result, param1, param2);
    if (result.vConnectionStatus.flags.connected) {
        LOG("We got a connection to the Serial device!");
        connected = true;
    }
    else {
        ERR("Failed to connect to the Serial Device!");
        printErrorConnectionStatus(result);
    }

    return connected;
}


/**
 * Helper function that retrieves controller status and prints all information
 */
void Linkam3::printLinkam3Status() {
    LinkamSDK::Variant result;
    linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetStatus, handle, &result);

    printf("controllerError               = %d\n", result.vControllerStatus.flags.controllerError);
    printf("heater1RampSetPoint           = %d\n", result.vControllerStatus.flags.heater1RampSetPoint);
    printf("heater1Started                = %d\n", result.vControllerStatus.flags.heater1Started);
    printf("heater2RampSetPoint           = %d\n", result.vControllerStatus.flags.heater2RampSetPoint);
    printf("heater2Started                = %d\n", result.vControllerStatus.flags.heater2Started);
    printf("vacuumRampSetPoint            = %d\n", result.vControllerStatus.flags.vacuumRampSetPoint);
    printf("vacuumCtrlStarted             = %d\n", result.vControllerStatus.flags.vacuumCtrlStarted);
    printf("vacuumValveClosed             = %d\n", result.vControllerStatus.flags.vacuumValveClosed);
    printf("vacuumValveOpen               = %d\n", result.vControllerStatus.flags.vacuumValveOpen);
    printf("humidityRampSetPoint          = %d\n", result.vControllerStatus.flags.humidityRampSetPoint);
    printf("humidityCtrlStarted           = %d\n", result.vControllerStatus.flags.humidityCtrlStarted);
    printf("lnpCoolingPumpOn              = %d\n", result.vControllerStatus.flags.lnpCoolingPumpOn);
    printf("lnpCoolingPumpAuto            = %d\n", result.vControllerStatus.flags.lnpCoolingPumpAuto);
    printf("HumidityDesiccantConditioning = %d\n", result.vControllerStatus.flags.HumidityDesiccantConditioning);
    printf("motorTravelMinX               = %d\n", result.vControllerStatus.flags.motorTravelMinX);
    printf("motorTravelMaxX               = %d\n", result.vControllerStatus.flags.motorTravelMaxX);
    printf("motorStoppedX                 = %d\n", result.vControllerStatus.flags.motorStoppedX);
    printf("motorTravelMinY               = %d\n", result.vControllerStatus.flags.motorTravelMinY);
    printf("motorTravelMaxY               = %d\n", result.vControllerStatus.flags.motorTravelMaxY);
    printf("motorStoppedY                 = %d\n", result.vControllerStatus.flags.motorStoppedY);
    printf("motorTravelMinTST             = %d\n", result.vControllerStatus.flags.motorTravelMinZ);
    printf("motorTravelMaxTST             = %d\n", result.vControllerStatus.flags.motorTravelMaxZ);
    printf("motorStoppedTST               = %d\n", result.vControllerStatus.flags.motorStoppedZ);
    printf("sampleCal                     = %d\n", result.vControllerStatus.flags.sampleCal);
    printf("motorDistanceCalTST           = %d\n", result.vControllerStatus.flags.motorDistanceCalTST);
    printf("cssRotMotorStopped            = %d\n", result.vControllerStatus.flags.cssRotMotorStopped);
    printf("cssGapMotorStopped            = %d\n", result.vControllerStatus.flags.cssGapMotorStopped);
    printf("cssLidOn                      = %d\n", result.vControllerStatus.flags.cssLidOn);
    printf("cssRefLimit                   = %d\n", result.vControllerStatus.flags.cssRefLimit);
    printf("cssZeroLimit                  = %d\n", result.vControllerStatus.flags.cssZeroLimit);
}


/*
 * Linkam3 driver constructor. Extends from asynPortDriver. Initalizes PV parameters, connects to the device, and populates some inital vals.
 * 
 * @params[in]: portName -> Name of the asynPort assigned
 * @params[in]: connectionType -> Either USB or Serial
 * @params[in]: productID -> USB product ID
 * @params[in]: vendorID -> USB venodr ID
 * @params[in]: serialPort -> Serial port to connect to ex. /dev/ttyS1
 * @params[in]: licenseFilePath -> Path to the Linkam.lsk file license for the SDK
 * @params[in]: logFilePath -> Path to the log file the SDK should generate
 */
Linkam3::Linkam3(const char *portName, const char* connectionType, unsigned int vendorID, unsigned int productID, const char* serialPort, 
                 const char* licenseFilePath, const char* logFilePath)
    : asynPortDriver(portName,
             1, /* maxAddr */
             NUM_LINKAM_PARAMS,
             asynFloat64Mask | asynInt32Mask | asynOctetMask | asynDrvUserMask, /* Interface mask */
             asynFloat64Mask | asynInt32Mask | asynOctetMask, /* Interrupt mask */
             0, /* asynFlags */
             1, /* Autoconnect */
             0, /* Default priority */
             0) /* Default stack size */
{

    const char* functionName = "Linkam3";
    (void) LinkamSDK::ControllerErrorStrings;

    // Initialize our PV parameters
    createParam(P_TempString,        asynParamFloat64, &P_Temp);
    createParam(P_RampRateSetString, asynParamFloat64, &P_RampRateSet);
    createParam(P_RampRateString,    asynParamFloat64, &P_RampRate);
    createParam(P_SetpointSetString, asynParamFloat64, &P_SetpointSet);
    createParam(P_SetpointString,    asynParamFloat64, &P_Setpoint);
    createParam(P_PowerString,       asynParamFloat64, &P_Power);
    createParam(P_StartHeatingString,asynParamInt32,   &P_StartHeating);
    createParam(P_LNPSpeedString,    asynParamFloat64, &P_LNPSpeed);
    createParam(P_DSCString,         asynParamFloat64, &P_DSC);
    createParam(P_HoldTimeSetString, asynParamInt32,   &P_HoldTimeSet);
    createParam(P_HoldTimeLeftString,asynParamInt32,   &P_HoldTimeLeft);
    createParam(P_LNPSetSpeedString, asynParamInt32,   &P_LNPSetSpeed);
    createParam(P_LNPSetModeString,  asynParamInt32,   &P_LNPSetMode);
    createParam(P_NameString,        asynParamOctet,   &P_Name);
    createParam(P_SerialString,      asynParamOctet,   &P_Serial);
    createParam(P_StageNameString,   asynParamOctet,   &P_StageName);
    createParam(P_StageSerialString, asynParamOctet,   &P_StageSerial);
    createParam(P_FirmVerString,     asynParamOctet,   &P_FirmVer);
    createParam(P_HardVerString,     asynParamOctet,   &P_HardVer);
    createParam(P_CtrllrErrorString, asynParamOctet,   &P_CtrllrError);
    createParam(P_CtrlConfigString,  asynParamInt32,   &P_CtrlConfig);
    createParam(P_CtrlStatusString,  asynParamInt32,   &P_CtrlStatus);
    createParam(P_StageConfigString, asynParamInt32,   &P_StageConfig);

    //printf("Disable logging\n");
    //linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_DisableLogging, 0, &result, param1, param2);

    // Initalize the SDK, given the licenseFile and logFile
    LOG("Initialising Linkam3 SDK...\n");
    if (linkamInitialiseSDK(logFilePath, licenseFilePath, false))
        LOG("Linkam SDK initialized successfully");
    else
        ERR("Failed to initialize Linkam SDK!");

    // Collect the SDK version
    char version[256];
    linkamGetVersion(version, 256);
    setStringParam(P_FirmVer, version);

    // Connect to the device
    bool connected = false;

    if(strcmp(connectionType, "Serial") == 0) {
        // /dev/ttyS1 is default - needs to be an iocsh arg TODO
        connected = initSerialConnection(serialPort, 0, 0, 0, 0, 0);
    }
    else if (strcmp(connectionType, "USB") == 0) {
        // Linkam vendor ID is 16da, product ID for T96 is 0002
        // Maybe this should also be an iocsh arg.
        connected = initUSBConnection(vendorID, productID);
    }
    else {
        ERR("No valid connection type selected!");
    }

    // Collect some device info if connected OK.
    if (connected) {

        LinkamSDK::Variant param1;
        LinkamSDK::Variant param2;
        LinkamSDK::Variant result;
        
        char serial[1024] = { 0 };
        char name[1024] = { 0 };
        
        // Get serial number info
        result.vUint64 = 0;
        param1.vPtr = serial;
        param2.vUint32 = 1024;
        linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetControllerSerial, handle, &result, param1, param2);
        setStringParam(P_Serial, serial);

        // Get model number info
        result.vUint64 = 0;
        param1.vPtr = name;
        param2.vUint32 = 1024;
        linkamProcessMessage(LinkamSDK::eLinkamFunctionMsgCode_GetControllerName, handle, &result, param1, param2);
        setStringParam(P_Name, name);

    }

    epicsAtExit(exitCallbackC, this);
}


/**
 * Destructor for the Linkam object.
 * Simply de-initalizes the SDK on IOC exit.
 */
Linkam3::~Linkam3(){
    static const char* functionName = "~Linkam3";
    LOG("Linkam3 Driver exiting...");

    linkamExitSDK();
    LOG("Exited the LinkamSDK. Goodbye");
}


/**
 * External C function called from the IOC shell. Spawns a Linkam3 driver instance.
 * 
 * @params[in]: portName -> Name of the asynPort assigned
 * @params[in]: vendorID -> Vendor ID for the linkam stage. Typically 0x16da
 * @params[in]: productID -> Product ID for the stage. Typically 0x0002
 * @params[in]: licenseFilePath -> Path to the Linkam.lsk file license for the SDK
 * @params[in]: logFilePath -> Path to the log file the SDK should generate
 * @returns: asynSuccess
 * 
 */
extern "C" int Linkam3ConnectUSB(const char* portName, unsigned int vendorID, unsigned int productID, const char* licenseFilePath, const char* logFilePath){

    new Linkam3(portName, "USB", vendorID, productID, "",  licenseFilePath, logFilePath);
    return asynSuccess;
}


/**
 * External C function called from the IOC shell. Spawns a Linkam3 driver instance, and connects via serial
 * 
 * @params[in]: portName -> Name of the asynPort assigned
 * @params[in]: serialPort -> Hardware serial port the stage is connected to (ex. /dev/ttyS1)
 * @params[in]: licenseFilePath -> Path to the Linkam.lsk file license for the SDK
 * @params[in]: logFilePath -> Path to the log file the SDK should generate
 * @returns: asynSuccess
 * 
 */
extern "C" int Linkam3ConnectSerial(const char* portName, const char* serialPort, const char* licenseFilePath, const char* logFilePath){

    new Linkam3(portName, "Serial", 0, 0, serialPort,  licenseFilePath, logFilePath);
    return asynSuccess;
}

//-------------------------------------------------------------
// Linkam3 ioc shell functions registration
//-------------------------------------------------------------


static const iocshArg Linkam3ConnectUSBArg0 = { "Port name",           iocshArgString };
static const iocshArg Linkam3ConnectUSBArg1 = { "Vendor ID",           iocshArgInt    };
static const iocshArg Linkam3ConnectUSBArg2 = { "Product ID",          iocshArgInt    };
static const iocshArg Linkam3ConnectUSBArg3 = { "License File Path",   iocshArgString };
static const iocshArg Linkam3ConnectUSBArg4 = { "Log File Path",       iocshArgString };


static const iocshArg* const Linkam3ConnectUSBArgs[] = { &Linkam3ConnectUSBArg0, 
                                                         &Linkam3ConnectUSBArg1, 
                                                         &Linkam3ConnectUSBArg2, 
                                                         &Linkam3ConnectUSBArg3, 
                                                         &Linkam3ConnectUSBArg4  };


static const iocshArg Linkam3ConnectSerialArg0 = { "Port name",           iocshArgString };
static const iocshArg Linkam3ConnectSerialArg1 = { "Serial Port",         iocshArgString };
static const iocshArg Linkam3ConnectSerialArg2 = { "License File Path",   iocshArgString };
static const iocshArg Linkam3ConnectSerialArg3 = { "Log File Path",       iocshArgString };


static const iocshArg* const Linkam3ConnectSerialArgs[] = { &Linkam3ConnectSerialArg0, 
                                                            &Linkam3ConnectSerialArg1, 
                                                            &Linkam3ConnectSerialArg2, 
                                                            &Linkam3ConnectSerialArg3 };


static void Linkam3ConnectUSBCallFunc(const iocshArgBuf* args){
    Linkam3ConnectUSB(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].sval);
}

static void Linkam3ConnectSerialCallFunc(const iocshArgBuf* args){
    Linkam3ConnectSerial(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}

static const iocshFuncDef Linkam3ConnectUSBFuncDef = {"Linkam3ConnectUSB", 4, Linkam3ConnectUSBArgs};
static const iocshFuncDef Linkam3ConnectSerialFuncDef = {"Linkam3ConnectSerial", 3, Linkam3ConnectSerialArgs};


static void Linkam3Register(void)
{
    iocshRegister(&Linkam3ConnectUSBFuncDef, Linkam3ConnectUSBCallFunc);
    iocshRegister(&Linkam3ConnectSerialFuncDef, Linkam3ConnectSerialCallFunc);
}


extern "C" {
    epicsExportRegistrar(Linkam3Register);
}
