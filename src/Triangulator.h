#pragma once
#include <iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<queue>
#include "Camera.h"

class Triangulator{
protected:
	std::vector<cv::Mat> projection_matrices;//< vector of projection matrices in mocap system
	std::vector<const tdr::Camera*> cameras;
public:
	struct CamPointPair
	{
		const tdr::Camera* camera;
		cv::Point2d point;
		cv::Mat projectionMatrix() { return camera->cameraPerspectiveMatrix; };
	};

	std::vector<const tdr::Camera*> getCameras(){
		return cameras;
	}

	virtual std::pair<cv::Point3d, double> triangulatePoint(vector<CamPointPair> images) = 0;

	/*! \brief method to 3D reconstruction based on vecotr of 2D positions
	*/
	virtual std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) = 0;
};