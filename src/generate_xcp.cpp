#include <iostream>
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <fstream>

std::vector<std::pair<cv::Mat, cv::Mat>> getData(const std::string& path){
    std::vector<std::pair<cv::Mat, cv::Mat>> result;

    std::ifstream file(path);
    std::string line, token;

    while (std::getline(file, line)){
        if(line.find("cam") != std::string::npos){
            continue;
        }

        std::istringstream iss(line);
        std::vector<double> seperatedLine;

        while(std::getline(iss, token, ',')) {
            seperatedLine.push_back(std::stod(token));
        }

        cv::Mat tvec = (cv::Mat_<double>(3, 1) << seperatedLine[1], seperatedLine[2], seperatedLine[3]);
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << seperatedLine[4], seperatedLine[5], seperatedLine[6]);
        result.push_back({tvec, rvec});
    }

    return result;
}

inline float SIGN(float x) { 
	return (x >= 0.0f) ? +1.0f : -1.0f; 
}

inline float NORM(float a, float b, float c, float d) { 
	return sqrt(a * a + b * b + c * c + d * d); 
}

cv::Mat mRot2Quat(const cv::Mat& m) {
	float r11 = m.at<float>(0, 0);
	float r12 = m.at<float>(0, 1);
	float r13 = m.at<float>(0, 2);
	float r21 = m.at<float>(1, 0);
	float r22 = m.at<float>(1, 1);
	float r23 = m.at<float>(1, 2);
	float r31 = m.at<float>(2, 0);
	float r32 = m.at<float>(2, 1);
	float r33 = m.at<float>(2, 2);
	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = cv::sqrt(q0);
	q1 = cv::sqrt(q1);
	q2 = cv::sqrt(q2);
	q3 = cv::sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	cv::Mat res = (cv::Mat_<float>(4, 1) << q0, q1, q2, q3);
	return res;
}

int main(){
    auto data = getData("./dataset/R02_D1/stationary_camera_data.csv");

    for(const auto& d : data){
        cv::Mat rmat;
		cv::Rodrigues(d.second, rmat);
        cv::Mat pos = -rmat.inv() * d.first;

        std::cout << "Position: " << pos.at<double>(0, 0) << " " << pos.at<double>(1, 0) << " " << pos.at<double>(2, 0) << std::endl;
        std::cout << rmat << std::endl;
        std::cout << d.second << std::endl;
        // std::cout << "Rotatoin: " << mRot2Quat(rmat) << std::endl;
    }

}