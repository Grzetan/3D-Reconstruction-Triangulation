#pragma once
#include <vector>
#include<opencv2/opencv.hpp>
#include "Triangulator.h"
class DroneClassifier2
{
    Triangulator* triangulator;
	public:
		struct Combination {
            std::vector<size_t> combination_;
            cv::Point3d point;
            double error;
            std::vector<int> sizes;
            int min_drones;
            int drones;

            bool operator<(const Combination& c) const;

            bool operator>(const Combination& c) const;

            bool isCombinationUnique(std::vector<Combination>& combinations);

            Combination(std::vector<size_t> combination_,cv::Point3d point,double error);
            Combination(int min_drones, std::vector<int> sizes);
            bool increment();

            
	    };
        DroneClassifier2 (Triangulator* triangulator) : triangulator(triangulator) {};

    void classifyDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones);
};