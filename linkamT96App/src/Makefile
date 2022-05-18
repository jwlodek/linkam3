TOP=../..

include $(TOP)/configure/CONFIG

LIBRARY_IOC += linkamT96

DBD += linkamT96Support.dbd

LinkamSDK_DIR = $(TOP)/linkamT96App/src

LIB_SYS_LIBS_Linux += usb-1.0
LIB_LIBS += asyn LinkamSDK

linkamT96_SRCS += linkamT96.cpp

include $(TOP)/configure/RULES

