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
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);

        // Declare depth colorizer for enhanced color visualization of depth data
        rs2::colorizer color_map;

        // Configure and start the pipeline
        mD435Pipe.start(cfg);
        while (1)
        {
            //Wait for all configured streams to produce a frame
            rs2::frameset fs = mD435Pipe.wait_for_frames();

            // The Depth data is delivered as uint16_t type which cannot be rendered directly, therefore we use rs2::colorizer
            // to convert the depth representation into human-readable RGB map:
            // Convert the newly-arrived frames to render-friendly format
            // Try to get a frame of a depth image/*  */
            rs2::depth_frame depth = fs.get_depth_frame();

            rs2::frame depth_frame = color_map.colorize(fs.get_depth_frame()); // Find and colorize the depth data
            // Query frame size (width and height)
            const int w = depth_frame.as<rs2::video_frame>().get_width();
            const int h = depth_frame.as<rs2::video_frame>().get_height();

            // Query the distance from the camera to the object in the center of the image
            float dist_to_center = depth.get_distance(w / 2, h / 2);

            // Print the distance
            cout << "The camera is facing an object " << dist_to_center << " meters away \r";

            // get left and right infrared frames from frameset
            rs2::video_frame ir_frame_left = fs.get_infrared_frame(1);
            rs2::video_frame ir_frame_right = fs.get_infrared_frame(2);
            // Creating OpenCV matrix from IR image
            cv::Mat irMat_left = cv::Mat(cv::Size(w, h), CV_8UC1, (void *)ir_frame_left.get_data());
            cv::Mat irMat_right = cv::Mat(cv::Size(w, h), CV_8UC1, (void *)ir_frame_right.get_data());

            // Apply Histogram Equalization
            equalizeHist(irMat_left, irMat_left);
            applyColorMap(irMat_left, irMat_left, COLORMAP_MAGMA);
            equalizeHist(irMat_right, irMat_right);
            applyColorMap(irMat_right, irMat_right, COLORMAP_OCEAN);

            //Get each frame
            rs2::frame color_frame = fs.get_color_frame();
            // Creating OpenCV Matrix from a color image
            Mat color(Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
            // Creating OpenCV Matrix from a depth image
            Mat depth1(Size(w, h), CV_8UC3, (void *)depth_frame.get_data(), Mat::AUTO_STEP);

            // Display in a GUI
            imshow("Color Image", color);
            imshow("Depth Image", depth1);
            imshow("IR Left", irMat_left);
            imshow("IR Right", irMat_right);
            waitKey(1); // Wait for a keystroke in the window
        }
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
        mT265Pipe.start(cfg);
        // Main loop
        while (1)
        {
            // Wait for the next set of frames from the camera
            auto fs = mT265Pipe.wait_for_frames();
            // Get a frame from the pose stream
            auto f = fs.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
            // Print the x, y, z values of the translation, relative to initial position
            cout << "\r"
                 << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)";

            rs2::video_frame fisheye_frame1 = fs.get_fisheye_frame(1);
            rs2::video_frame fisheye_frame2 = fs.get_fisheye_frame(2);
            // Query frame size (width and height)
            const int w = fisheye_frame1.as<rs2::video_frame>().get_width();
            const int h = fisheye_frame1.as<rs2::video_frame>().get_height();
            cv::Mat fisheye_mat1 = Mat(Size(w, h), CV_8UC1, (void *)fisheye_frame1.get_data(), Mat::AUTO_STEP);
            cv::Mat fisheye_mat2 = Mat(Size(w, h), CV_8UC1, (void *)fisheye_frame2.get_data(), Mat::AUTO_STEP);
            // Display in a GUI
            namedWindow("Fisheye1", WINDOW_AUTOSIZE);
            imshow("Fisheye1", fisheye_mat1);
            namedWindow("Fisheye2", WINDOW_AUTOSIZE);
            imshow("Fisheye2", fisheye_mat2);
            waitKey(1);
        }
    }

    /** Fires up all sensors and creates Reactive streams for their frames */
    void initializeSensors()
    {
        startD435();
        startT265();
    }

    void showPointCloud()
    {
        // Declarations
        rs2::pointcloud pc;
        rs2::points points;
        rs2::pipeline pipe;
        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        // Declare depth colorizer for enhanced color visualization of depth data
        rs2::colorizer color_map;

        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        //Instruct pipeline to start streaming with the requested configuration
        pipe.start(cfg);
        // Camera warmup - dropping several first frames to let auto-exposure stabilize
        rs2::frameset frames;
        for (int i = 0; i < 30; i++)
        {
            //Wait for all configured streams to produce a frame
            frames = pipe.wait_for_frames();
        }

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // pc.map_to(color);
        pc.map_to(color_map.colorize(depth)); // Find and colorize the depth data)
        points = pc.calculate(depth);

        // Creating OpenCV Matrix from a color image
        Mat pointsMat(Size(640, 480), CV_8UC3, (void *)points.get_data(), Mat::AUTO_STEP);
        // Display in a GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE);
        imshow("Display Image", pointsMat);
        waitKey(0); // Wait for a keystroke in the window
        // points.export_to_ply("your_filename.ply", color);
    }

    rx::observable<rs2::frameset> observableD435()
    {
        // auto start = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
        // auto o = rxcpp::observable<>::timer(start);
        auto period = std::chrono::milliseconds(1000 / 60);
        auto o = rxcpp::observable<>::interval(period);
        // generate information
        auto d435Obs(driver(1, mD435Pipe));
        return d435Obs;
    }
} // namespace Magus
