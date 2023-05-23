#pragma once
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>
#include "Triangulator.h"

# define MAX_ERROR 120 // This value can be different for different errors used in solver. For now it's 70 which means 7cm which is more or less drone size
# define MIN_CAMERAS 3

class DroneClassifier{
private:
    Triangulator* triangulator_;

    struct Combination {
        std::vector<size_t> combination_;
        cv::Point3d point;
        double error;
        // size_t closestPath;

        bool operator<(const Combination& c) const;

        bool operator>(const Combination& c) const;

        bool isCombinationUnique(std::vector<Combination>& combinations);

        static bool increment(std::vector<size_t>& combination, std::vector<size_t>& sizes);
    };

public:
    DroneClassifier(Triangulator* triangulator) : triangulator_(triangulator){};

	void classifyDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones);
};