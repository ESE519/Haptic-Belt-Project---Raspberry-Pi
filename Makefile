BIN_DIR = ../Bin

INC_DIRS = ../../Include =/usr/include/ni

SRC_FILES = ./*.cpp + ./*.c

EXE_NAME = Sample-NiSimpleRead

USED_LIBS = OpenNI
USED_LIBS += wiringPi

LIB_DIRS += ../../Lib

include ../Build/Common/CommonCppMakefile