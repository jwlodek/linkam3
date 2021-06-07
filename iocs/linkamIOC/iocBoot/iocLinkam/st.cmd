#!../../bin/linux-x86_64/linkamT96App

errLogInit(2000)

< envPaths

dbLoadDatabase("../../dbd/linkamT96App.dbd")
linkamT96App_registerRecordDeviceDriver(pdbbase)

# IOC PV prefix
epicsEnvSet("PREFIX", "TestLinkam1")

# Asyn Port
epicsEnvSet("PORT", "LINK1")

# Path to the Linkam.lsk software license file
epicsEnvSet("LINKAM3_LICENSE_PATH", "/epics/src/support/linkam3/SDK/bin/Release/x64/Linkam.lsk")

# Path for log messages
epicsEnvSet("LINKAM3_LOG_PATH", "tmp/linkam.log")

# USB Connection parameters.
epicsEnvSet("VENDOR_ID", "")
epicsEnvSet("PRODUCT_ID", "")

# Serial Connection Parameter ex. /dev/ttyS1
epicsEnvSet("SERIAL_PORT", "/dev/ttyS1")

# Use one of the below functions to connect to the Linkam Stage

# Linkam 3.0 USB connection function
Linkam3ConnectUSB("$(PORT)", "$(VENDOR_ID)", "$(PRODUCT_ID)", "$(LINKAM3_LICENSE_PATH)", "$(LINKAM3_LOG_PATH)")

# Linkam 3.0 Serial connection function
Linkam3ConnectSerial("$(PORT)", "$(SERIAL_PORT)", "$(LINKAM3_LICENSE_PATH)", "$(LINKAM3_LOG_PATH)")
epicsThreadSleep(2)

# Set asyn log level
asynSetTraceIOMask($(PORT), 0, 0x2)
#asynSetTraceMask($(PORT), 0, 0xff)

# Load linkam records
dbLoadRecords("$(LINKAM3)/db/Linkam.template", "P=$(PREFIX), PORT=$(PORT), ADDR=0, TIMEOUT=1")

iocInit()
