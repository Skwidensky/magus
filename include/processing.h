#pragma once
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

using namespace cv;
using namespace cv::dnn;

namespace Magus
{
    void initModels();
    void read(rs2::frameset);
    void decodeBoundingBoxes(const Mat &scores, const Mat &geometry, float scoreThresh,
                             std::vector<RotatedRect> &detections, std::vector<float> &confidences);
    void fourPointsTransform(const Mat &frame, Point2f vertices[4], Mat &result);
    void decodeText(const Mat &scores, std::string &text);
} // namespace Magus