#pragma once
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

#include "DetectionsContainer.h"
#include "Triangulator.h"
#include "utils.h"

#define MAX_ERROR_MATRIX 1e+5  // Max error for matrix triangulator
#define MAX_ERROR_RAY 120      // Max error for ray triangulator
#define MAX_STEP \
  200  // Max difference between drone positions between frames (used in new
       // algo)
#define MIN_CAMERAS 2  // Minimum number of cameras for valid combination
#define PATH_TAIL 3  // Length of path tail. Used in classyfing drones to paths

class DroneClassifier {
 public:
  struct Combination {
    std::vector<int> combination_;
    cv::Point3d point;
    double error;

    bool operator<(const Combination& c) const;

    bool operator>(const Combination& c) const;

    bool isCombinationUnique(const std::vector<Combination>& combinations);
  };

  struct CombinationPath {
    size_t combination;
    size_t path;
    double error;

    bool operator>(const CombinationPath& elem) const;
  };

 private:
  Triangulator* triangulator_;
  size_t n_drones_;

  double error_;

  class Iterator {
    std::vector<int> combination_;
    bool skipNext = false;
    const std::vector<int> sizes_;

   public:
    Iterator(std::vector<int> sizes);

    bool increment();

    bool cut();

    std::vector<int> getCombination();
  };

  void fillCombinationQueue(const DetectionsContainer& container, int frame,
                            std::priority_queue<Combination>& pq);

  std::vector<Combination> pickBestCombinations(
      const DetectionsContainer& container, int frame,
      const std::vector<Combination>& usedCombinations);

  Combination triangulateWithLastPos(cv::Point3d pos,
                                     const DetectionsContainer& container,
                                     std::vector<Combination> usedCombinations,
                                     int frame);

  bool isDetectionInCombinations(int cam, int det,
                                 const std::vector<Combination>& combinations);

  void classifyPaths(const std::vector<Combination>& finalCombinations,
                     std::vector<std::vector<cv::Point3d>>& triangulatedPoints,
                     std::vector<int>& processedPaths,
                     std::vector<std::vector<int>>& emptyFrames, int frame);

 public:
  DroneClassifier(Triangulator* triangulator, size_t n_drones);

  void classifyDrones(
      const DetectionsContainer& container,
      std::vector<std::vector<cv::Point3d>>& triangulatedPoints);
};