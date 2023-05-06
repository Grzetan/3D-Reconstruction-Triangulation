#pragma once
#include <iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include "Camera.h"


struct CamPointPair
{
	cv::Mat projectionMatrix;
	cv::Point2d point;
};

class HasiecTriangulator
{
private:
	std::vector<cv::Mat> projection_matrices;//< vector of projection matrices in mocap system


	void construct(std::vector<cv::Mat> projection_matrices);

	cv::Point3d triangulatePoint(vector<CamPointPair> images);

public:
	
	/*! \brief constructor
	*/
	HasiecTriangulator(std::vector<const tdr::Camera*> cameras_);

	/*! \brief method to 3D reconstruction based on vecotr of 2D positions
	*/
	std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points);
};

