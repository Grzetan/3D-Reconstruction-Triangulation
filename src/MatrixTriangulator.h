#include<opencv2/opencv.hpp>
#include<vector>
#include "Camera.h"
#include "Triangulator.h"


class MatrixTriangulator : public Triangulator{
public:
	
	/*! \brief method to 3D reconstruction based on vector pairs - camera and point
	*/
	std::pair<cv::Point3d, double> triangulatePoint(vector<CamPointPair> images) override;
	
	/*! \brief constructor
	*/
	MatrixTriangulator(std::vector<const tdr::Camera*> cameras_);

	/*! \brief method to 3D reconstruction based on vecotr of 2D positions
	*/
	std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) override;
	
};
