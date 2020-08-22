#include "plog/Log.h"
#include <iostream>
#include <exception>
#include <processing.h>

using namespace std;
using namespace cv;

namespace Magus
{
    void detectLetters(Mat img)
    {
        Mat rgb;
        // downsample and use it for processing
        pyrDown(img, rgb);
        Mat small;
        cvtColor(rgb, small, COLOR_BGR2GRAY);
        // morphological gradient
        Mat grad;
        Mat morphKernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        morphologyEx(small, grad, MORPH_GRADIENT, morphKernel);
        // binarize
        Mat bw;
        threshold(grad, bw, 0.0, 255.0, THRESH_BINARY | THRESH_OTSU);
        // connect horizontally oriented regions
        Mat connected;
        morphKernel = getStructuringElement(MORPH_RECT, Size(9, 1));
        morphologyEx(bw, connected, MORPH_CLOSE, morphKernel);
        // find contours
        Mat mask = Mat::zeros(bw.size(), CV_8UC1);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(connected, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
        // filter contours
        for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        {
            Rect rect = boundingRect(contours[idx]);
            Mat maskROI(mask, rect);
            maskROI = Scalar(0, 0, 0);
            // fill the contour
            drawContours(mask, contours, idx, Scalar(255, 255, 255), FILLED);
            // ratio of non-zero pixels in the filled region
            double r = (double)countNonZero(maskROI) / (rect.width * rect.height);

            if (r > .45 /* assume at least 45% of the area is filled if it contains text */
                &&
                (rect.height > 4 && rect.width > 4) /* constraints on region size */
                /* these two conditions alone are not very robust. better to use something 
            like the number of significant peaks in a horizontal projection as a third condition */
            )
            {
                rectangle(rgb, rect, Scalar(0, 255, 0), 2);
            }
        }
        // Tesseract setup
        tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
        ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
        ocr->SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
        ocr->SetImage(small.data, small.cols, small.rows, CV_8U, small.step);
        string outText = string(ocr->GetUTF8Text());
        cout << outText;
        ocr->End();
        imshow("Text", rgb);

        // std::vector<Rect> boundRect;
        // Mat img_gray, img_sobel, img_threshold, element;
        // cvtColor(img, img_gray, COLOR_RGB2GRAY);
        // Sobel(img_gray, img_sobel, CV_8U, 1, 0, 3, 1, 0, BORDER_DEFAULT);
        // threshold(img_sobel, img_threshold, 0, 255, THRESH_OTSU + THRESH_BINARY);
        // element = getStructuringElement(MORPH_RECT, Size(17, 3));
        // morphologyEx(img_threshold, img_threshold, MORPH_CLOSE, element); //Does the trick
        // std::vector<std::vector<Point>> contours;
        // findContours(img_threshold, contours, 0, 1);
        // std::vector<std::vector<Point>> contours_poly(contours.size());
        // for (int i = 0; i < contours.size(); i++)
        //     if (contours[i].size() > 100)
        //     {
        //         approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        //         Rect appRect(boundingRect(Mat(contours_poly[i])));
        //         if (appRect.width > appRect.height)
        //             boundRect.push_back(appRect);
        //     }
        // return boundRect;
    }

    void read(rs2::frameset fs)
    {
        // Get the RGB camera frame from this frameset
        rs2::frame color_frame = fs.get_color_frame();
        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
        detectLetters(color);

        // //Detect
        // std::vector<Rect> letterBBoxes1 = detectLetters(color);
        // //Display
        // for (int i = 0; i < letterBBoxes1.size(); i++)
        // {
        //     rectangle(color, letterBBoxes1[i], cv::Scalar(0, 255, 0), 3, 8, 0);
        // }
        // imshow("Text", color);
        // waitKey(1);

        // Tesseract setup
        // tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
        // ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
        // ocr->SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
        // ocr->SetImage(gray.data, gray.cols, gray.rows, 3, gray.step);
        // string outText = string(ocr->GetUTF8Text());
        // cout << outText;
        // ocr->End();
    }
} // namespace Magus