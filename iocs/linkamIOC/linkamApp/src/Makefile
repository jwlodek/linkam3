TOP = ../..
include $(TOP)/configure/CONFIG

PROD_IOC = linkamT96App
DBD += linkamT96App.dbd
linkamT96App_DBD += base.dbd
linkamT96App_DBD += asyn.dbd
linkamT96App_DBD += linkamT96Support.dbd
linkamT96App_SRCS += linkamT96App_registerRecordDeviceDriver.cpp
linkamT96App_LIBS += linkamT96 LinkamSDK
linkamT96App_LIBS += asyn
linkamT96App_LIBS += $(EPICS_BASE_IOC_LIBS)
linkamT96App_SRCS += linkamMain.cpp

include $(TOP)/configure/RULES

