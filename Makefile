#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure
DIRS := $(DIRS) linkamT96App
DIRS := $(DIRS) linkamT96Support


linkamT96App_DEPEND_DIRS += linkamT96Support

ifeq ($(BUILD_IOCS), YES)
	DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
	iocs_DEPEND_DIRS += linkamT96App
endif


include $(TOP)/configure/RULES_TOP
