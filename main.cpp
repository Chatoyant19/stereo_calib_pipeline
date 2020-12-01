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

std::string pathL = "/home/allride/project/tools/Calibration/StereoCalibration/data/left";
std::string pathR = "/home/allride/project/tools/Calibration/StereoCalibration/data/right";
const int boardWidth = 6;
const int boardHeight = 7;
const cv::Size patternSize = cv::Size(boardWidth, boardHeight);
const int imgNums = 79;
const int squareSize = 99; //mm
const int imgWidth = 1920;
const int imgHeight = 1200;
cv::Size imgSize = cv::Size(imgWidth, imgHeight);
cv::Mat cameraMatrixL = (cv::Mat_<double>(3, 3) << 2739.892779, 0.000000, 937.923870,
                         0.000000, 2744.022581, 608.525789,
                         0.000000, 0.000000, 1.000000);
cv::Mat distCoeffsL = (cv::Mat_<double>(4, 1) << -0.103383, 0.794893, 0.002980, -0.002612);
cv::Mat cameraMatrixR = (cv::Mat_<double>(3, 3) << 4221.266385, 0.000000, 979.843051,
                         0.000000, 4227.004087, 600.627402,
                         0.000000, 0.000000, 1.000000);
cv::Mat distCoeffsR = (cv::Mat_<double>(4, 1) << 0.006736, -1.614485, 0.003468, 0.002181);

cv::Mat R, T, E, F;
cv::Mat Rl, Rr, Pl, Pr, Q;

bool calcRealPoint3d(std::vector<std::vector<cv::Point3f>> &realPoints)
{
    std::vector<cv::Point3f> imgPoints;
    for (int rowIndex = 0; rowIndex < boardWidth; ++rowIndex)
    {
        for (int colIndex = 0; colIndex < boardWidth; ++colIndex)
        {
            imgPoints.emplace_back(cv::Point3f(rowIndex * squareSize, colIndex * squareSize, 0));
        }
    }
    for (int imgIndex = 0; imgIndex < imgNums; ++imgIndex)
    {
        realPoints.emplace_back(imgPoints);
    }
    return true;
}

bool outputCameraParam()
{
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "cameraMatrixL: " << cameraMatrixL
           << "cameraDiscoeffsL: " << distCoeffsL
           << "cameraMatrixR: " << cameraMatrixR
           << "cameraDiscoeffsR: " << distCoeffsR;
        fs.release();
        std::cout << "cameraMatrixL = " << std::endl
                  << cameraMatrixL << std::endl
                  << "cameraDiscoeffsL = " << std::endl
                  << distCoeffsL << std::endl
                  << "cameraMatrixR = " << std::endl
                  << cameraMatrixR << std::endl
                  << "cameraDiscoeffsR = " << std::endl
                  << distCoeffsL << std::endl;
    }
    else
    {
        std::cout << "Error: cannot save the intrinsics!" << std::endl;
    }
    fs.open("extrinsics.yml", cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "R: " << R << "T: " << T
           << "Rl: " << Rl << "Rr: " << Rr
           << "Pl: " << Pl << "Pr: " << Pr
           << "Q: " << Q;
        std::cout << "R = " << std::endl
                  << R << std::endl
                  << "T = " << std::endl
                  << T << std::endl
                  << "Rl = " << std::endl
                  << Rl << std::endl
                  << "Rr = " << std::endl
                  << Rr << std::endl
                  << "Pl = " << std::endl
                  << Pl << std::endl
                  << "Pr = " << std::endl
                  << Pr << std::endl
                  << "Q: " << std::endl
                  << Q << std::endl;
    }
    else
    {
        std::cout << "Error : cannot save extrinsic parameters" << std::endl;
    }
    return true;
}

int main(int argc, char *argv[])
{
    // cv::Mat img;
    std::vector<cv::String> paths_L;
    std::vector<cv::String> paths_R;
    cv::glob(pathL, paths_L);
    cv::glob(pathR, paths_R);
    std::vector<std::vector<cv::Point2f>> imgPointL, imgPointR; //all corners
    std::vector<std::vector<cv::Point3f>> objRealPoints;

    cv::Rect validROIL, validROIR;
    cv::Mat mapLx, mapLy, mapRx, mapRy;

    // std::cout << paths_L.size() << " " << paths_R.size() << std::endl;
    if (paths_L.size() != paths_R.size())
    {
        std::cerr << "cannot calibration!" << std::endl;
        return -1;
    }
    for (int i = 0; i < paths_R.size(); ++i)
    {
        std::cout << "mathing " << paths_L[i] << " & " << paths_R[i] << std::endl;
        std::vector<cv::Point2f> cornerL, cornerR;
        cv::Mat rgbImgL = cv::imread(paths_L[i], CV_LOAD_IMAGE_COLOR);
        cv::Mat rgbImgR = cv::imread(paths_R[i], CV_LOAD_IMAGE_COLOR);
        cv::Mat grayImgL, grayImgR;
        cv::cvtColor(rgbImgL, grayImgL, CV_BGR2GRAY);
        cv::cvtColor(rgbImgR, grayImgR, CV_BGR2GRAY);

        bool isFindL = cv::findChessboardCorners(rgbImgL, patternSize, cornerL);
        bool isFindR = cv::findChessboardCorners(rgbImgR, patternSize, cornerR);
        if (isFindL && isFindR)
        {
            cv::cornerSubPix(grayImgL, cornerL, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
            cv::drawChessboardCorners(rgbImgL, patternSize, cornerL, isFindL);
            // cv::imshow("chessboardL", rgbImgL);
            imgPointL.emplace_back(cornerL);
            cv::cornerSubPix(grayImgR, cornerR, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
            cv::drawChessboardCorners(rgbImgR, patternSize, cornerR, isFindR);
            // cv::imshow("chessboardR", rgbImgR);
            imgPointR.emplace_back(cornerR);
        }
        cv::waitKey(0);
    }

    //calculate reality point
    if (!calcRealPoint3d(objRealPoints))
    {
        std::cout << "calculation 3d reality point falied!" << std::endl;
        return -1;
    }

    double rms = cv::stereoCalibrate(objRealPoints, imgPointL, imgPointR,
                                     cameraMatrixL, distCoeffsL,
                                     cameraMatrixR, distCoeffsR,
                                     cv::Size(imgWidth, imgHeight), R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 80, 1e-5));
    std::cout << "Stereo Calibration done with RMS error = " << rms << std::endl;
    cv::stereoRectify(cameraMatrixL, distCoeffsL,
                      cameraMatrixR, distCoeffsR,
                      imgSize, R, T,
                      Rl, Rr, Pl, Pr, Q,
                      cv::CALIB_ZERO_DISPARITY, -1, imgSize,
                      &validROIL, &validROIR);

    cv::initUndistortRectifyMap(cameraMatrixL, distCoeffsR,
                                Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
    cv::initUndistortRectifyMap(cameraMatrixR, distCoeffsR,
                                Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);

    for (int i = 0; i < paths_R.size(); ++i)
    {
        cv::Mat rectifyImgL, rectifyImgR;
        cv::Mat imgL, imgR;
        //befor rectify
        imgL = cv::imread(paths_L[i]);
        imgR = cv::imread(paths_R[i]);
        cv::remap(imgL, rectifyImgL, mapLx, mapLy, cv::INTER_LINEAR);
        cv::remap(imgR, rectifyImgR, mapRx, mapRy, cv::INTER_LINEAR);
        // cv::imshow("rectifyImgL", rectifyImgL);
        // cv::imshow("rectifyImgR", rectifyImgR);
        if (!outputCameraParam())
        {
            std::cout << "cannot output calibration result" << std::endl;
        }
    }
    cv::waitKey(0);

    return 0;
}