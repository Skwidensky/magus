#include <sensorsrx.h>
#include <thread>
#include <memory>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <processing.h>
#include "rxcpp/rx.hpp"

namespace rx = rxcpp;
using namespace std;
using namespace cv;

rs2::colorizer m_color_map;

namespace Magus
{
    template <class Obs>
    class sensordriver
    {
    public:
        sensordriver(Obs o, rs2::pipeline pipe, int t) : o_(o), pipe_(pipe), t_(t) {}
        void listen()
        {
            for (;;)
            {
                if (!o_.is_subscribed())
                {
                    return;
                }
                o_.on_next(pipe_.wait_for_frames());
                if (!o_.is_subscribed())
                {
                    return;
                }
            }
        }
        Obs o_;
        rs2::pipeline pipe_;
        int t_;
    };

    template <typename T>
    sensordriver<T> CreateDriver(const T &obs, rs2::pipeline pipe, int t)
    {
        return sensordriver<T>(obs, pipe, t);
    }

    rx::observable<rs2::frameset> streamdriver(int t, rs2::pipeline pipe, rs2::colorizer color_map)
    {
        m_color_map = color_map;
        return rx::sources::create<rs2::frameset>([t, pipe](auto out) {
            std::thread th([t, pipe, out]() {
                auto a = CreateDriver(out, pipe, t);
                a.listen();
            });
            th.detach();
            return out.get_subscription();
        });
    }

    streamhandler::streamhandler() {}

    /** D435 provides four channels of information -- RGB, IR1, IR2, and depth/stereo */
    double streamhandler::handleD435Frameset(rs2::frameset fs)
    {
        // The Depth data is delivered as uint16_t type which cannot be rendered directly, therefore we use rs2::colorizer
        // to convert the depth representation into human-readable RGB map:
        // Convert the newly-arrived frames to render-friendly format
        // Try to get a frame of a depth image
        rs2::depth_frame depth = fs.get_depth_frame();

        rs2::frame depth_frame = m_color_map.colorize(fs.get_depth_frame()); // Find and colorize the depth data
        // Query frame size (width and height)
        const int w = depth_frame.as<rs2::video_frame>().get_width();
        const int h = depth_frame.as<rs2::video_frame>().get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(w / 2, h / 2);

        // Print the distance
        // cout << "\r" 
        //      << "The camera is facing an object " << dist_to_center << " meters away ";

        // get left and right infrared frames from frameset
        rs2::video_frame ir_frame_left = fs.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = fs.get_infrared_frame(2);
        // Creating OpenCV matrix from IR image
        Mat irMat_left = Mat(Size(w, h), CV_8UC1, (void *)ir_frame_left.get_data());
        Mat irMat_right = Mat(Size(w, h), CV_8UC1, (void *)ir_frame_right.get_data());

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
        read(fs);
        // imshow("Color Image", color);
        // imshow("Depth Image", depth1);
        // imshow("IR Left", irMat_left);
        // imshow("IR Right", irMat_right);
        // waitKey(1); // Wait for a keystroke in the window
        return fs.get_timestamp();
    }

    /** T265 provides 5-channels of information: fisheye1, fisheye2, pose, gyro, and accel */
    double streamhandler::handleT265Frameset(rs2::frameset fs)
    {
        // Get a frame from the pose stream
        auto pf = fs.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = pf.as<rs2::pose_frame>().get_pose_data();
        // Print the x, y, z values of the translation, relative to initial position
        // cout
        //     << "Device Position: " << setprecision(3) << fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters) ";

        // Get a frame from the gyro stream
        auto gf = fs.first_or_default(RS2_STREAM_GYRO);
        // Cast the frame to gyro_frame and get its data
        auto gyro_frame = gf.as<rs2::motion_frame>();
        // Get the timestamp of the current frame
        double ts = gyro_frame.get_timestamp();
        // Get gyro measurements
        rs2_vector gyro_data = gyro_frame.get_motion_data();
        // cout
        //     << "Gyro Position: " << setprecision(3) << fixed << " Pitch: " << gyro_data.x << " Yaw: " << pose_data.translation.y << " Roll: " << pose_data.translation.z << " ";

        // Get a frame from the accel stream
        auto af = fs.first_or_default(RS2_STREAM_ACCEL);
        // Cast the frame to gyro_frame and get its data
        auto accel_frame = af.as<rs2::motion_frame>();
        // Get accelerometer measurements
        rs2_vector accel_data = accel_frame.get_motion_data();
        // cout 
        //      << "Acceleration: " << setprecision(3) << fixed << " X: " << accel_data.x << " Y: " << accel_data.y << " Z: " << accel_data.z << "\r";

        rs2::video_frame fisheye_frame1 = fs.get_fisheye_frame(1);
        rs2::video_frame fisheye_frame2 = fs.get_fisheye_frame(2);
        // Query frame size (width and height)
        const int w = fisheye_frame1.as<rs2::video_frame>().get_width();
        const int h = fisheye_frame1.as<rs2::video_frame>().get_height();
        Mat fisheye_mat1 = Mat(Size(w, h), CV_8UC1, (void *)fisheye_frame1.get_data(), Mat::AUTO_STEP);
        Mat fisheye_mat2 = Mat(Size(w, h), CV_8UC1, (void *)fisheye_frame2.get_data(), Mat::AUTO_STEP);
        // Display in a GUI
        imshow("Fisheye1", fisheye_mat1);
        imshow("Fisheye2", fisheye_mat2);
        waitKey(1);
        return fs.get_timestamp();
    }
}; // namespace Magus