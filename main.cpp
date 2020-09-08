#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>            // C++

const double PI = 3.14159265358979f;
int wheelBase = 2845; // 축거(mm)
int tread = 1614;  // 윤거(mm)

using namespace cv;

static void on_tracker(int, void*) {
}

int main() {
    static Scalar RED(0, 0, 255);
    static Scalar BLUE(255, 0, 0);

    Mat srcImg = imread("parking.jpg");
    Mat dstImg;    // undistorted img
    Mat dstWarpImg;
    if (srcImg.empty()) { return -1; }

    namedWindow("image");

    createTrackbar("degree+540", "image", 0, 1080, on_tracker);
    createTrackbar("line ratio", "image", 0, 3, on_tracker);
    createTrackbar("wheelBase", "image", 0, 5000, on_tracker);
    createTrackbar("tread", "image", 0, 2000, on_tracker);

    setTrackbarPos("degree+540", "image", 540);
    setTrackbarPos("line ratio", "image", 3);
    setTrackbarPos("wheelBase", "image", 2845);
    setTrackbarPos("tread", "image", 1614);

    int w = srcImg.cols;
    int h = srcImg.rows;

    double intrinsicMtx[3][3] = {
            {680.028000, 0.000000,   502.443000},
            {0.000000,   678.789000, 267.333000},
            {0.000000,   0.000000,   1.000000}
    };
    Mat mtxMat(Size(3, 3), CV_64FC1, intrinsicMtx);

    double dist[4] = { -0.000989, 0.000000, 0.000000, 0.000000 };
    Mat distMat(Size(1, 4), CV_64FC1, dist);

    // matrix for set line =======================================================================
    double x1 = 0.0, x2 = w, x3 = w * 0.3 + 130, x4 = w * 0.6 + 5;
    double x_1 = x1, x_2 = x2, x_3 = 0.0, x_4 = w;
    double y1 = 0.0, y2 = 0.0, y3 = h, y4 = h;
    double y_1 = h * 0.6, y_2 = h * 0.6, y_3 = y3, y_4 = y4;

    Point2f pts1[4], pts2[4];

    pts1[0] = Point2f(x1, y1);
    pts1[1] = Point2f(x2, y2);
    pts1[2] = Point2f(x3, y3);
    pts1[3] = Point2f(x4, y4);

    pts2[0] = Point2f(x_1, y_1);
    pts2[1] = Point2f(x_2, y_2);
    pts2[2] = Point2f(x_3, y_3);
    pts2[3] = Point2f(x_4, y_4);

    Mat M1 = getPerspectiveTransform(pts2, pts1);	// warp
    Mat M2 = getPerspectiveTransform(pts1, pts2);

    undistort(srcImg, dstImg, mtxMat, distMat);
    warpPerspective(dstImg, dstWarpImg, M1, Size(w, h));
    // ===========================================================================================

    // different each picture
    int center = int(round(((w * 0.3 + 170) + (w * 0.6 - 25)) / 2));
    int roadWidth = int(round((w * 0.6 - 25) - (w * 0.3 + 170)));

    int circle1_r;
    int circle2_r;
    int circle3_r;

    int boxX = w / 2;
    int boxY = h / 2;
    Rect roi(boxX, boxY, 60, 60);

    while (true) {
        int pos = getTrackbarPos("degree+540", "image") - 540;
        int lineLengthRatio = getTrackbarPos("line ratio", "image");
        double steeringPos = pos / 15.4;
        double rad = steeringPos * PI / 180;
        double r = 0;

        wheelBase = getTrackbarPos("wheelBase", "image");
        tread = getTrackbarPos("tread", "image");

        Mat drawing(540, 960, CV_8UC3, Scalar(0, 0, 0));
        Mat drawingWarp;
        Mat imgResult;

        if (pos == 0) {
            line(drawing, Point(round(w * 0.3 + 170), round(h * 0.05)), Point(round(w * 0.3 + 170), h), RED, 1);
            line(drawing, Point(center, round(h * 0.05)), Point(center, h), BLUE, 1);
            line(drawing, Point(round(w * 0.6 - 25), round(h * 0.05)), Point(round(w * 0.6 - 25), h), RED, 1);
            line(drawing, Point(round(w * 0.3 + 170), round(h * 0.05)), Point(round(w * 0.6 - 25), round(h * 0.05)), RED, 1);

            // inside guide line
            for (int y = 0; y < drawing.rows; ++y) {
                for (int x = 0; x < drawing.cols; ++x) {
                    if (x > round(w * 0.3 + 170) && x < round(w * 0.6 - 25)) {
                        line(drawing, Point(x, y), Point(x, y), BLUE, 1);
                    }
                }
            }
        }
        else if (0 < pos) {
            r = (wheelBase / sin(rad) * cos(rad) + tread / 2 - 160) * (0.06);
            circle1_r = round(abs(r + round(roadWidth / 2)));  // left arc
            circle2_r = round(abs(r));
            circle3_r = round(abs(r - round(roadWidth / 2)));  // right arc

            x1 = center + round(r) - round(cos(rad * lineLengthRatio) * circle1_r);
            y1 = round(h - abs(sin(rad * lineLengthRatio)) * circle1_r);
            x2 = center + round(r) - round(cos(rad * lineLengthRatio) * circle3_r);
            y2 = round(h - abs(sin(rad * lineLengthRatio)) * circle3_r);

            ellipse(drawing, Point(round(center + r), h), Point(circle1_r, circle1_r), 0, 180, 180 + ceil(steeringPos * lineLengthRatio), RED, 1, LINE_AA);
            ellipse(drawing, Point(round(center + r), h), Point(circle2_r, circle2_r), 0, 180, 180 + ceil(steeringPos * lineLengthRatio), BLUE, 1, LINE_AA);
            ellipse(drawing, Point(round(center + r), h), Point(circle3_r, circle3_r), 0, 180, 180 + ceil(steeringPos * lineLengthRatio), RED, 1, LINE_AA);
            line(drawing, Point(x1, y1), Point(x2, y2), RED, 1);

            // inside guide line
            for (int y = 0; y < drawing.rows; ++y) {
                for (int x = 0; x < drawing.cols; ++x) {
                    if (pow((r + center - x), 2) + pow(h - y, 2) < pow(circle1_r, 2) && pow((r + center - x), 2) + pow(h - y, 2) > pow(circle3_r, 2)) {
                        line(drawing, Point(x, y), Point(x, y), BLUE, 1);
                    }
                }
            }
        }
        else if (pos < 0) {
            r = (wheelBase / sin(rad) * cos(rad) - tread / 2 + 160) * 0.06;
            circle1_r = round(abs(r + round(roadWidth / 2)));  // left arc
            circle2_r = round(abs(r));
            circle3_r = round(abs(r - round(roadWidth / 2)));  // right arc

            x1 = center + round(r) + round(cos(rad * lineLengthRatio) * circle1_r);
            y1 = round(h - abs(sin(rad * lineLengthRatio)) * circle1_r);
            x2 = center + round(r) + round(cos(rad * lineLengthRatio) * circle3_r);
            y2 = round(h - abs(sin(rad * lineLengthRatio)) * circle3_r);

            ellipse(drawing, Point(round(center + r), h), Point(circle1_r, circle1_r), 0, 0, floor(steeringPos * lineLengthRatio), RED, 1, LINE_AA);
            ellipse(drawing, Point(round(center + r), h), Point(circle2_r, circle2_r), 0, 0, floor(steeringPos * lineLengthRatio), BLUE, 1, LINE_AA);
            ellipse(drawing, Point(round(center + r), h), Point(circle3_r, circle3_r), 0, 0, floor(steeringPos * lineLengthRatio), RED, 1, LINE_AA);
            line(drawing, Point(x1, y1), Point(x2, y2), RED, 1);

            // inside guide line
            for (int y = 0; y < drawing.rows; ++y) {
                for (int x = 0; x < drawing.cols; ++x) {
                    if (pow((x - (r + center)), 2) + pow(h - y, 2) > pow(circle1_r, 2) && pow((x - (r + center)), 2) + pow(h - y, 2) < pow(circle3_r, 2)) {
                        line(drawing, Point(x, y), Point(x, y), BLUE, 1);
                    }
                }
            }
        }

        warpPerspective(drawing, drawingWarp, M2, Size(w, h));

        // check box =======================================================================
        Mat checkBox = drawingWarp(roi);

        if (drawingWarp.at<Vec3b>(boxY + 60, boxX + 30) == Vec3b(0, 0, 0)) {
            checkBox = Scalar(0, 255, 0);
        }
        else {
            checkBox = RED;
        }
        // =================================================================================

        addWeighted(drawingWarp, 1, dstImg, 1, 0, drawingWarp);
        addWeighted(drawing, 1, dstWarpImg, 1, 0, drawing);

        hconcat(drawingWarp, drawing, imgResult);
        imshow("image", imgResult);

        if (waitKey(1) == 27)
            break;
    }
}