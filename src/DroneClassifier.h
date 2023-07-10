#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Triangulator.h"
#include "DetectionsContainer.h"

# define MAX_ERROR_MATRIX 1e+5//120 // This value can be different for different errors used in solver. For now it's 70 which means 7cm which is more or less drone size
# define MAX_ERROR_RAY 120
# define MAX_STEP 15 // Max difference between drone positions between frames
# define MIN_CAMERAS 3 // Minimum number of cameras for valid combination
# define PATH_TAIL 3 // Length of path tail. Used in classyfing drones to paths

class DroneClassifier{
private:
    Triangulator* triangulator_;
    size_t n_drones_;

    double error_;

    struct Combination {
        std::vector<int> combination_;
        cv::Point3d point;
        double error;

        bool operator<(const Combination& c) const;

        bool operator>(const Combination& c) const;

        bool isCombinationUnique(std::vector<Combination>& combinations);
    };

    struct CombinationPath{
        size_t combination;
        size_t path;
        double error;

        bool operator > (const CombinationPath& elem) const;
    };

    class Iterator{
        std::vector<int> combination_;
        bool skipNext = false;
        const std::vector<int> sizes_;
    public:
        Iterator(std::vector<int> sizes);

        bool increment();

        bool cut();

        std::vector<int> getCombination();
    };

    std::vector<Combination> pickBestCombinations(const DetectionsContainer& container, int frame);

    Combination triangulateWithLastPos(cv::Point3d pos, const DetectionsContainer& container, std::vector<Combination> usedCombinations, int frame);

public:
    DroneClassifier(Triangulator* triangulator, size_t n_drones);

	void classifyDrones(const DetectionsContainer& container, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones);
};