#include "iostream"
#include <opencv2/opencv.hpp>
#include "PointProjector.h"
#include "Camera.h"

int main(){
    tdr::Camera cam1(0);
    cam1.width = 640;
    cam1.height = 480;
    cam1.fovx = 50;
    cv::Mat t1(3, 1, CV_64F);
    t1.at<double>(0, 0) = -10;
    t1.at<double>(1, 0) = 0;
    t1.at<double>(2, 0) = 0;
    cam1.tvec = t1;
    cv::Mat r1(4, 1, CV_64F);
    t1.at<double>(0, 0) = 0.7071;
    t1.at<double>(1, 0) = 0;
    t1.at<double>(2, 0) = 0.7071;
    t1.at<double>(3, 0) = 0;
    cam1.rvec = cam1.toRotVec(r1);
    cam1.compCamParams();
    

    std::cout << "XD" << std::endl;
    return 0;
}