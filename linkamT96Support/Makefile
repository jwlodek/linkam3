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

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libLinkamSDK.so

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32 += ../os/windows-x64/LinkamSDK.lib
BIN_INSTALLS_WIN32 += ../os/windows-x64/LinkamSDK.dll

else ifeq (windows-x64-static, $(findstring windows-x64-static, $(T_A)))
LIB_INSTALLS_WIN32 += ../os/windows-x64/LinkamSDK.lib
BIN_INSTALLS_WIN32 += ../os/windows-x64/LinkamSDK.dll

endif



#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
