#include "iostream"
#include <opencv2/opencv.hpp>

int main(){

    cv::Mat i(1,3,CV_64F);

    i.at<double>(0,0) = 2;
    std::cout << i.at<double>(0,0) << std::endl;

    return 0;
}