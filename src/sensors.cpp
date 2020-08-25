#include "sensors.h"
#include "plog/Log.h"
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <exception>

using namespace std;
using namespace cv;
namespace rx = rxcpp;

namespace Magus
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline mD435Pipe;
    rs2::pipeline mT265Pipe;

    // Struct for managing D435
    struct d435State
    {
        double yaw, pitch, last_x, last_y;
        bool ml;
        float offset_x, offset_y;
    };

    // Struct for managing T265
    struct t265State
    {
        double yaw, pitch, last_x, last_y;
        bool ml;
        float offset_x, offset_y;
    };

    void startD435()
    {
        LOG_INFO << "LOG: Starting pipeline to D435";

        // Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 30);

        // Configure and start the pipeline
        mD435Pipe.start(cfg);
    }

    void startT265()
    {
        LOG_INFO << "LOG: Starting pipeline to T265";
        // Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        // Enable both image streams
        // Note: It is not currently possible to enable only one
        cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
        // Add pose stream
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        // Add gyro stream
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        // Add accelerometer stream
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        mT265Pipe.start(cfg);
    }

    /** Fires up all sensors and creates Reactive streams for their frames */
    void initializeSensors()
    {
        startD435();
        // startT265();
    }

    rx::observable<rs2::frameset> observableD435()
    {
        LOG_INFO << "LOG: Fetching D435 RX stream";
        // Declare depth colorizer for enhanced color visualization of depth data
        rs2::colorizer color_map;
        // generate information
        auto d435Obs(streamdriver(1000 / 30, mD435Pipe, color_map));
        return d435Obs;
    }

    rx::observable<rs2::frameset> observableT265()
    {
        LOG_INFO << "LOG: Fetching T265 RX stream";
        // Declare depth colorizer for enhanced color visualization of depth data
        rs2::colorizer color_map;
        // generate information
        auto t265Obs(streamdriver(1000 / 30, mT265Pipe, color_map));
        return t265Obs;
    }
} // namespace Magus
