# Uncomment to add libraries
# LIBRARIES += ../deps/stlplus/containers
CXXFLAGS += -ggdb -std=c++14 -Wall -Wextra
CXXFLAGS += -I./ -I../include -I../deps/plog/include -I../deps/RxCpp/Rx/v2/src -I/usr/local/include/opencv4
# Enable to see the state machine transition
# CXXFLAGS += -DTRACE_STATE_MACHINE
LDFLAGS += -Wall -pthread
LDLIBS += -L/usr/local/lib -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc
include ../deps/makefiles/gcc.mak