#include "CameraCalibrator.h"

template<typename _Tp> static  cv::Mat toMat(const std::vector<std::vector<_Tp> > vecIn) {
    cv::Mat_<_Tp> matOut(vecIn.size(), vecIn.at(0).size());
    for (int i = 0; i < matOut.rows; ++i) {
        for (int j = 0; j < matOut.cols; ++j) {
            matOut(i, j) = vecIn.at(i).at(j);
        }
    }
    return matOut;
}

//cv::Mat CameraCalibrator::calibrate(std::vector<std::vector<float>>objectPoints, std::vector<std::vector<float>>imagePoints)
//{
//    cv::Mat objectPointsMat = toMat<float>(objectPoints);
//    cv::transpose(objectPointsMat, objectPointsMat);
//    
//    cv::Mat imagePointsMat = toMat<float>(imagePoints);
//    cv::transpose(imagePointsMat, imagePointsMat);
//
//    cv::Mat cam1Mat;
//    cv::Mat projected;
//    cv::Mat invertObjectPoints;
//    try {
//        cv::invert(objectPointsMat, invertObjectPoints, cv::DECOMP_SVD);
//    }
//    catch (std::exception e)
//    {
//        std::cout << e.what() << std::endl;
//    }
//    cam1Mat = imagePointsMat * invertObjectPoints;
//    return cam1Mat;
//}


cv::Mat CameraCalibrator::calibrate(std::vector<std::vector<double>> objectPoints, std::vector<cv::Point2d> imagePoints)
{
    cv::Mat objectPointsMat = toMat<double>(objectPoints);
    cv::transpose(objectPointsMat, objectPointsMat);

    cv::Mat imagePointsMat = cv::Mat_<double>(imagePoints.size(), 3, CV_64F);
    for (int i = 0; i < imagePointsMat.rows; ++i) {

        imagePointsMat.at<double>(i, 0) = imagePoints[i].x;
        imagePointsMat.at<double>(i, 1) = imagePoints[i].y;
        imagePointsMat.at<double>(i, 2) = 1;

    }

    cv::transpose(imagePointsMat, imagePointsMat);

    cv::Mat cam1Mat;
    cv::Mat projected;
    cv::Mat invertObjectPoints;
    try {
        cv::invert(objectPointsMat, invertObjectPoints, cv::DECOMP_SVD);
    }
    catch (std::exception e)
    {
        std::cout << e.what() << std::endl;
    }
    cam1Mat = imagePointsMat * invertObjectPoints;
    return cam1Mat;
}
