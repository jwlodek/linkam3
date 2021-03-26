TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================


# Include files
INC += CommonStageAPI.h
INC += CommonTypes.h
INC += CommsAPI.h
INC += ControllerAPI.h
INC += DeprecatedAPI.h
INC += EventAPI.h
INC += ExportAPI.h
INC += LinkamSDK.h
INC += Message.h
INC += Serial.h
INC += USB.h

ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libLinkamSDK.so

else ifeq(linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libLinkamSDK.so

endif



#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
