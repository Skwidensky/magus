#pragma once
#include <sensorsrx.h>
#include <string>
#include <librealsense2/rs.hpp>
#include "rxcpp/rx.hpp"

enum Sensor
{
    D435,
    T265
};

namespace Magus
{
    void initializeSensors();
    void showDepthImg();
    void showPointCloud();
    rxcpp::observable<rs2::frameset> observableD435();
} // namespace Magus