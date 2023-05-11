#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "PointTriangulator.h"
#include "HasiecTriangulator.h"
#include "utils.h"

int main(int argc, const char** argv){
    if(argc != 3 && argc != 4) throw std::runtime_error("Input paths must be provided");

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(argv[1]);

    // First dim = n_camera, second_dim = n_points
    std::vector<std::vector<cv::Point2d>> dronePoints;
    loadPointsOneDrone(argv[2], dronePoints);

    auto start = std::chrono::high_resolution_clock::now();
    // HasiecTriangulator projector(cameras);
    // std::vector<cv::Point3d> triangulatedPoints = projector.triangulatePoints(dronePoints);

    PointTriangulator projector(cameras);
    std::vector<cv::Point3d> triangulatedPoints = projector.triangulatePoints(dronePoints);
    auto stop = std::chrono::high_resolution_clock::now();
    
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

    const char* path = (argc == 4) ? argv[3] : "output.ply";
    writeOutputFile(path, triangulatedPoints);
}