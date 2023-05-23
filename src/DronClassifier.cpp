#include "DronClassifier.h"
#include <queue>
void DronClassifier2::triangulateMultipleDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones)
{
	std::priority_queue<Combination> combinationQueue();
	//for
}

//DronClassifier::Combination& DronClassifier::Combination::operator++()
//{
//	//for (int i = this->combination_.size() - 1; i >= 0; i++)
//	//{
//	//	this->combination_[i] = (this->combination_[i] + 1) % this->combination_.size();
//	//}
//}
