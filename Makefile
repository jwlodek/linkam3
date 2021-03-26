#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure
DIRS := $(DIRS) linkamT96App
DIRS := $(DIRS) linkamT96Support
DIRS := $(DIRS) iocs


linkamT96App_DEPEND_DIRS += linkamT96Support
iocs_DEPEND_DIRS += linkamT96App


include $(TOP)/configure/RULES_TOP
