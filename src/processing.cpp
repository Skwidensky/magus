#include "plog/Log.h"
#include <iostream>
#include <exception>
#include <processing.h>

using namespace std;

/*
    Text detection model: https://github.com/argman/EAST
    Download link: https://www.dropbox.com/s/r2ingd0l3zt8hxs/frozen_east_text_detection.tar.gz?dl=1
    CRNN Text recognition model taken from here: https://github.com/meijieru/crnn.pytorch
    How to convert from pb to onnx:
    Using classes from here: https://github.com/meijieru/crnn.pytorch/blob/master/models/crnn.py
    More converted onnx text recognition models can be downloaded directly here:
    Download link: https://drive.google.com/drive/folders/1cTbQ3nuZG-EKWak6emD_s8_hHXWz7lAr?usp=sharing
    And these models taken from here:https://github.com/clovaai/deep-text-recognition-benchmark
    import torch
    from models.crnn import CRNN
    model = CRNN(32, 1, 37, 256)
    model.load_state_dict(torch.load('crnn.pth'))
    dummy_input = torch.randn(1, 1, 32, 100)
    torch.onnx.export(model, dummy_input, "crnn.onnx", verbose=True)

    "{ input i     | | Path to input image or video file. Skip this argument to capture frames from a camera.}"
    "{ model m     | | Path to a binary .pb file contains trained detector network.}"
    "{ ocr         | | Path to a binary .pb or .onnx file contains trained recognition network.}"
    "{ width       | 320 | Preprocess input image by resizing to a specific width. It should be multiple by 32. }"
    "{ height      | 320 | Preprocess input image by resizing to a specific height. It should be multiple by 32. }"
    "{ thr         | 0.5 | Confidence threshold. }"
    "{ nms         | 0.4 | Non-maximum suppression threshold. }";
*/
namespace Magus
{
    static const string sWinName = "Efficient and Accurate Scene Text Detector";
    static const string m_modelDecoder = "src/models/frozen_east_text_detection.pb";
    static const string m_modelRecognition = "src/models/ResNet_CTC.onnx";
    static const float m_confThreshold = 0.5;
    static const float m_nmsThreshold = 0.4;
    static Net m_txtDetector;
    static Net m_txtRecognizer;
    TickMeter m_tickMeter;

    void initModels()
    {
        LOG_INFO << "Initializing inference models";
        // Load networks.
        m_txtDetector = readNet(m_modelDecoder);
        m_txtRecognizer = readNet(m_modelRecognition);
    }

    void detectText(Mat &img)
    {
        vector<Mat> outs;
        vector<String> outNames(2);
        outNames[0] = "feature_fusion/Conv_7/Sigmoid";
        outNames[1] = "feature_fusion/concat_3";
        Mat blob;

        resize(img, img, Size(640, 480));

        blobFromImage(img, blob, 1.0, img.size(), Scalar(123.68, 116.78, 103.94), true, false);
        m_txtDetector.setInput(blob);
        m_tickMeter.start();
        m_txtDetector.forward(outs, outNames);
        m_tickMeter.stop();
        cout << format("Inference time: %.2f ms", m_tickMeter.getTimeMilli()) << endl;

        Mat scores = outs[0];
        Mat geometry = outs[1];

        // Decode predicted bounding boxes.
        vector<RotatedRect> boxes;
        vector<float> confidences;
        decodeBoundingBoxes(scores, geometry, m_confThreshold, boxes, confidences);

        // Apply non-maximum suppression procedure.
        vector<int> indices;
        NMSBoxes(boxes, confidences, m_confThreshold, m_nmsThreshold, indices);

        Point2f ratio((float)img.cols / img.cols, (float)img.rows / img.rows);

        // Render text.
        for (size_t i = 0; i < indices.size(); ++i)
        {
            RotatedRect &box = boxes[indices[i]];

            Point2f vertices[4];
            box.points(vertices);

            // for (int j = 0; j < 4; ++j)
            // {
            //     vertices[j].x *= ratio.x;
            //     vertices[j].y *= ratio.y;
            // }

            // Mat cropped;
            // fourPointsTransform(img, vertices, cropped);

            // cvtColor(cropped, cropped, cv::COLOR_BGR2GRAY);

            // Mat blobCrop = blobFromImage(cropped, 1.0 / 127.5, Size(), Scalar::all(127.5));
            // txtRecognizer.setInput(blobCrop);

            // tickMeter.start();
            // Mat result = m_txtRecognizer.forward();
            // tickMeter.stop();

            // string wordRecognized = "";
            // decodeText(result, wordRecognized);
            // putText(img, wordRecognized, vertices[1], FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255));

            for (int j = 0; j < 4; ++j)
            {
                line(img, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 1);
            }
        }

        // Put efficiency information.
        string label = format("Inference time: %.2f ms", m_tickMeter.getTimeMilli());
        putText(img, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

        imshow(sWinName, img);
        m_tickMeter.reset();
        waitKey(1);
    }

    void decodeBoundingBoxes(const Mat &scores, const Mat &geometry, float scoreThresh,
                             vector<RotatedRect> &detections, vector<float> &confidences)
    {
        detections.clear();
        CV_Assert(scores.dims == 4);
        CV_Assert(geometry.dims == 4);
        CV_Assert(scores.size[0] == 1);
        CV_Assert(geometry.size[0] == 1);
        CV_Assert(scores.size[1] == 1);
        CV_Assert(geometry.size[1] == 5);
        CV_Assert(scores.size[2] == geometry.size[2]);
        CV_Assert(scores.size[3] == geometry.size[3]);

        const int height = scores.size[2];
        const int width = scores.size[3];
        for (int y = 0; y < height; ++y)
        {
            const float *scoresData = scores.ptr<float>(0, 0, y);
            const float *x0_data = geometry.ptr<float>(0, 0, y);
            const float *x1_data = geometry.ptr<float>(0, 1, y);
            const float *x2_data = geometry.ptr<float>(0, 2, y);
            const float *x3_data = geometry.ptr<float>(0, 3, y);
            const float *anglesData = geometry.ptr<float>(0, 4, y);
            for (int x = 0; x < width; ++x)
            {
                float score = scoresData[x];
                if (score < scoreThresh)
                {
                    continue;
                }

                // Decode a prediction.
                // Multiply by 4 because feature maps are 4 time less than input image.
                float offsetX = x * 4.0f, offsetY = y * 4.0f;
                float angle = anglesData[x];
                float cosA = cos(angle);
                float sinA = sin(angle);
                float h = x0_data[x] + x2_data[x];
                float w = x1_data[x] + x3_data[x];

                Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                               offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
                Point2f p1 = Point2f(-sinA * h, -cosA * h) + offset;
                Point2f p3 = Point2f(-cosA * w, sinA * w) + offset;
                // padding to mitigate letters getting cut off
                // endX = int(endX * rW)+2
                // endY = int(endY * rH)+3
                RotatedRect r(0.5f * (p1 + p3), Size2f(w, h), -angle * 180.0f / (float)CV_PI);
                detections.push_back(r);
                confidences.push_back(score);
            }
        }
    }

    void fourPointsTransform(const Mat &frame, Point2f vertices[4], Mat &result)
    {
        const Size outputSize = Size(100, 32);

        Point2f targetVertices[4] = {
            Point(0, outputSize.height - 1),
            Point(0, 0),
            Point(outputSize.width - 1, 0),
            Point(outputSize.width - 1, outputSize.height - 1),
        };
        Mat rotationMatrix = getPerspectiveTransform(vertices, targetVertices);

        warpPerspective(frame, result, rotationMatrix, outputSize);
    }

    void decodeText(const Mat &scores, string &text)
    {
        static const string alphabet = "0123456789abcdefghijklmnopqrstuvwxyz";
        Mat scoresMat = scores.reshape(1, scores.size[0]);

        vector<char> elements;
        elements.reserve(scores.size[0]);

        for (int rowIndex = 0; rowIndex < scoresMat.rows; ++rowIndex)
        {
            Point p;
            minMaxLoc(scoresMat.row(rowIndex), 0, 0, 0, &p);
            if (p.x > 0 && static_cast<size_t>(p.x) <= alphabet.size())
            {
                elements.push_back(alphabet[p.x - 1]);
            }
            else
            {
                elements.push_back('-');
            }
        }

        if (elements.size() > 0 && elements[0] != '-')
            text += elements[0];

        for (size_t elementIndex = 1; elementIndex < elements.size(); ++elementIndex)
        {
            if (elementIndex > 0 && elements[elementIndex] != '-' &&
                elements[elementIndex - 1] != elements[elementIndex])
            {
                text += elements[elementIndex];
            }
        }
    }

    void read(rs2::frameset fs)
    {
        // Get the RGB camera frame from this frameset
        rs2::video_frame color_frame = fs.get_color_frame();
        const int w_c = color_frame.get_width();
        const int h_c = color_frame.get_height();
        // Creating OpenCV Matrix from a color image
        Mat color(Size(w_c, h_c), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
        detectText(color);
    }
} // namespace Magus