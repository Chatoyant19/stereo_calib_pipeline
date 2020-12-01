#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
const int imageWidth = 1920;
const int imageHeight = 1200;
const int boardWidth = 6;
const int boardHeight = 7;
const int boardCorner = boardWidth * boardHeight;
const int squareSize = 100;
const int imgNums = 43; // frames' numbers
const Size boardSize = Size(boardWidth, boardHeight);
Size imageSize = Size(imageWidth, imageHeight);
Mat R, T, E, F;

std::string pathL = "/home/allride/project/tools/Calibration/StereoCalibration/data/R004/left";
std::string pathR = "/home/allride/project/tools/Calibration/StereoCalibration/data/R004/right";

vector<vector<Point2f>> imagePointL;
vector<vector<Point2f>> imagePointR;
vector<vector<Point3f>> objRealPoint;
vector<Point2f> cornerL;
vector<Point2f> cornerR;
Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat Rl, Rr, Pl, Pr, Q;
Mat mapLx, mapLy, mapRx, mapRy;
Rect validROIL, validROIR;

Mat cameraMatrixL = (Mat_<double>(3, 3) << 2692.970232, 0.000000, 971.990827,
                     0.000000, 2701.796658, 591.614391,
                     0.000000, 0.000000, 1.000000);
Mat distCoeffL = (Mat_<double>(4, 1) << -0.122643, 0.901881, -0.001705, 0.002473);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 4144.131299, 0.000000, 1011.117363,
                     0.000000, 4131.299629, 640.252240,
                     0.000000, 0.000000, 1.000000);
Mat distCoeffR = (Mat_<double>(4, 1) << -0.038629, -0.881473, 0.004732, 0.002998);

void calcRealPoint3d(vector<vector<Point3f>> &obj)
{
    vector<Point3f> imgpoint;
    for (int rowIndex = 0; rowIndex < boardHeight; rowIndex++)
    {
        for (int colIndex = 0; colIndex < boardWidth; colIndex++)
        {
            imgpoint.push_back(Point3f(rowIndex * squareSize, colIndex * squareSize, 0));
        }
    }
    for (int imgIndex = 0; imgIndex < imgNums; imgIndex++)
    {
        obj.push_back(imgpoint);
    }
}
void outputCameraParam(void)
{
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "cameraMatrixL" << cameraMatrixL
           << "cameraDistcoeffL" << distCoeffL
           << "cameraMatrixR" << cameraMatrixR
           << "cameraDistcoeffR" << distCoeffR;
        fs.release();
        // cout << "cameraMatrixL=:" << cameraMatrixL << endl
        //      << "cameraDistcoeffL=:" << distCoeffL << endl
        //      << "cameraMatrixR=:" << cameraMatrixR << endl
        //      << "cameraDistcoeffR=:" << distCoeffR << endl;
    }
    else
    {
        cout << "Error: can not save the intrinsics!!!!!" << endl;
    }
    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
        // cout << "R=" << R << endl
        //      << "T=" << T << endl
        //      << "Rl=" << Rl << endl
        //      << "Rr=" << Rr << endl
        //      << "Pl=" << Pl << endl
        //      << "Pr=" << Pr << endl
        //      << "Q=" << Q << endl;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";
}

int main(int argc, char *argv[])
{
    std::vector<cv::String> paths_L;
    std::vector<cv::String> paths_R;
    cv::glob(pathL, paths_L);
    cv::glob(pathR, paths_R);
    for (int i = 0; i < paths_R.size(); ++i)
    {
        rgbImageL = imread(paths_L[i], CV_LOAD_IMAGE_COLOR);
        cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
        rgbImageR = imread(paths_R[i], CV_LOAD_IMAGE_COLOR);
        cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
        bool isFindL, isFindR;
        isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
        isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
        if (isFindL == true && isFindR == true)
        {
            cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
            drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
            imshow("chessboardL", rgbImageL);
            imagePointL.push_back(cornerL);
            cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
            drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
            imshow("chessboardR", rgbImageR);
            imagePointR.push_back(cornerR);
        }
        if (waitKey(10) == 'q')
        {
            break;
        }
    }
    calcRealPoint3d(objRealPoint);
    cout << "cal real successful" << endl;
    double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
                                 cameraMatrixL, distCoeffL,
                                 cameraMatrixR, distCoeffR,
                                 Size(imageWidth, imageHeight), R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS,
                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 80, 1e-5));
    cout << "Stereo Calibration done with RMS error = " << rms << endl;
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);

    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
    Mat rectifyImageL, rectifyImageR;
    cout << "debug" << endl;
    for (int i = 0; i < paths_R.size(); ++i)
    {
        rectifyImageL = imread(paths_L[i]);
        rectifyImageR = imread(paths_R[i]);
        imshow("Rectify Before", rectifyImageL);
        Mat rectifyImageL2, rectifyImageR2;
        remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
        remap(rectifyImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
        imshow("rectifyImageL", rectifyImageL2);
        imshow("rectifyImageR", rectifyImageR2);
        outputCameraParam();
        Mat canvas;
        double sf;
        int w, h;
        sf = 600. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);
        Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
        resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
        Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
                   cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
        rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);
        cout << "Painted ImageL" << endl;
        canvasPart = canvas(Rect(w, 0, w, h));
        resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
                   cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
        rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
        // cout << "Painted ImageR" << endl;
        for (int i = 0; i < canvas.rows; i += 16)
            line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        //cout << "wait key" << endl;
        waitKey(10); //system("pause");
    }
    return 0;
}