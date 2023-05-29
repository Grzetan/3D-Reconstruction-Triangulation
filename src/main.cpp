#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "RayTriangulator.h"
#include "MatrixTriangulator.h"
#include "DroneClassifier.h"
#include "utils.h"

int main(int argc, const char** argv){
    if(argc != 3 && argc != 4) throw std::runtime_error("Input paths must be provided");

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(argv[1]);

    // First dim = n_camera, second_dim = n_frames
    // std::vector<std::vector<cv::Point2d>> dronePoints;
    // loadPointsOneDrone(argv[2], dronePoints);

    Triangulator* triangulator = new RayTriangulator(cameras);

    DroneClassifier classifier(triangulator);

    // Film (6 ekranow i kazdy dron osobny ekran z sciezka 3d)
    // Zmienic blad na blad kwadratowy w matrixTriangulator

    // // First dim = n_cameras, second_dim = n_frames, third_dim = n_drones
    std::vector<std::vector<std::vector<cv::Point2d>>> dronePoints;
    loadPointsMultipleDrones(argv[2], dronePoints, 1, 7);
    
    std::cout << dronePoints[0].size() << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    // First dim = n_drones, second dim = n_cameras, third_dim = n_frames
    std::vector<std::vector<cv::Point3d>> triangulatedPoints;
    classifier.classifyDrones(dronePoints, triangulatedPoints, 2);

    // auto start = std::chrono::high_resolution_clock::now();
    // // HasiecTriangulator triangulator(cameras);
    // // std::vector<cv::Point3d> triangulatedPoints = triangulator.triangulatePoints(dronePoints);

    // PointTriangulator triangulator(cameras);
    // std::vector<cv::Point3d> triangulatedPoints = triangulator.triangulatePoints(dronePoints);
    auto stop = std::chrono::high_resolution_clock::now();
    
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;
    
    writeOutputFile("dron1H_G.ply", triangulatedPoints[0]);
    writeOutputFile("dron2H_G.ply", triangulatedPoints[1]);
    // const char* path = (argc == 4) ? argv[3] : "output.ply";
    // writeOutputFile(path, triangulatedPoints);

    // Free memory
    delete triangulator;
    for(const auto& cam : cameras)
        delete cam;
}