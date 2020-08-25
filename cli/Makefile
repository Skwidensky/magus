include ../deps/makefiles/platform.mak

LIBRARIES += ../src ../deps/stlplus/subsystems ../deps/stlplus/containers ../deps/stlplus/portability ../deps/RxCpp/Rx/v2/src

ifeq ($(RELEASE),on)
CXXFLAGS += -ggdb
endif

CXXFLAGS += -Wall -Wextra -std=c++14 -g
CXXFLAGS += -I../include -I../deps/plog/include -I/usr/local/include/opencv4
LDLIBS += -L/usr/local/lib -lrealsense2 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_dnn -lm
LDFLAGS += -Wall -pthread
LOADLIBES += -lm

ifeq ($(RELEASE),on)
TARGETDIR := ../../bin/release
else
TARGETDIR := ../../bin/debug
endif

IMAGENAME := $(TARGETDIR)/magus
include ../deps/makefiles/gcc.mak
