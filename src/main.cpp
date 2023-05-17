#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "PointTriangulator.h"
#include "HasiecTriangulator.h"
#include "utils.h"

int main(int argc, const char** argv){
    if(argc != 3 && argc != 4) throw std::runtime_error("Input paths must be provided");

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(argv[1]);

    // First dim = n_camera, second_dim = n_frames
    // std::vector<std::vector<cv::Point2d>> dronePoints;
    // loadPointsOneDrone(argv[2], dronePoints);

    PointTriangulator triangulator(cameras);

    // // First dim = n_cameras, second_dim = n_frames, third_dim = n_drones
    std::vector<std::vector<std::vector<cv::Point2d>>> dronePoints;
    loadPointsMultipleDrones(argv[2], dronePoints);

    // // First dim = n_drones, second dim = n_cameras, third_dim = n_frames
    std::vector<std::vector<std::vector<cv::Point2d>>> classifiedDronePoints;
    triangulator.classifyDrones(dronePoints, classifiedDronePoints, 2);

    // auto start = std::chrono::high_resolution_clock::now();
    // // HasiecTriangulator triangulator(cameras);
    // // std::vector<cv::Point3d> triangulatedPoints = triangulator.triangulatePoints(dronePoints);

    // PointTriangulator triangulator(cameras);
    // std::vector<cv::Point3d> triangulatedPoints = triangulator.triangulatePoints(dronePoints);
    // auto stop = std::chrono::high_resolution_clock::now();
    
    // auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

    // const char* path = (argc == 4) ? argv[3] : "output.ply";
    // writeOutputFile(path, triangulatedPoints);
}