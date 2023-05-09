#pragma once
#include <opencv2\opencv.hpp>
#include <vector>
class CameraCalibrator
{
public:

	//cv::Mat calibrate(std::vector<std::vector<float>>, std::vector<std::vector<float>>);
	cv::Mat calibrate(std::vector<std::vector<double>>, std::vector<cv::Point2d>);

};

