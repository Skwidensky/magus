#pragma once
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

namespace Magus
{
    void read(rs2::frameset);
} // namespace Magus