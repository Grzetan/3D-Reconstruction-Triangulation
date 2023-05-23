#pragma once
#include <iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<queue>
#include "Camera.h"

class Triangulator
{

private:
	std::vector<const tdr::Camera*> cameras;
public:
	struct CamPointPair
	{
		const tdr::Camera* camera;
		cv::Point2d point;
		cv::Mat projectionMatrix() { return camera->cameraPerspectiveMatrix; };
	};
	virtual std::pair<cv::Point3d, double> triangulatePoint(vector<CamPointPair> images) = 0;

	/*! \brief method to 3D reconstruction based on vecotr of 2D positions
	*/
	virtual std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) = 0;
	virtual const tdr::Camera* getCamera(int camera) = 0;
};