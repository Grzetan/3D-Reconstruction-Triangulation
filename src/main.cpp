#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "argparse.hpp"
#include "RayTriangulator.h"
#include "MatrixTriangulator.h"
#include "DroneClassifier.h"
#include "DetectionsContainer.h"
#include "utils.h"
#include <filesystem>

std::string OUTPUT_DIR = "./results/";

int main(int argc, const char** argv){
    argparse::ArgumentParser args("3D-Reconstruction-Triangulation"); 

    args.add_argument("cameras_path")
    .help("Path to the file with camera data")
    .required();

    args.add_argument("data_path")
    .help("Path to the folder woth detections on each camera")
    .required();

    args.add_argument("--n_drones")
    .help("Number of drones in the scene")
    .scan<'i', int>()
    .default_value(1);

    args.add_argument("--triangulator")
    .help("Which triangulator should be used")
    .default_value<std::string>("matrix");

    try {
        args.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    if(std::filesystem::exists(OUTPUT_DIR)){
        std::filesystem::remove_all(OUTPUT_DIR);
    }

    std::filesystem::create_directory(OUTPUT_DIR);

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(args.get("cameras_path").c_str());

    int n_drones = args.get<int>("n_drones");

    if(n_drones > 1){
        Triangulator* triangulator;
        if(args.get<std::string>("--triangulator") == "matrix"){
            triangulator = new MatrixTriangulator(cameras);
        }else if(args.get<std::string>("--triangulator") == "ray"){
            triangulator = new RayTriangulator(cameras);
        }else{
            throw std::runtime_error("Invalid --triangulator argument. Allowed options are \'matrix\' and \'ray\'");
        }

        DroneClassifier classifier(triangulator, n_drones);

        DetectionsContainer container(args.get("data_path").c_str(), 0, 7);

        auto start = std::chrono::high_resolution_clock::now();

        std::vector<std::vector<cv::Point3d>> triangulatedPoints;
        classifier.classifyDrones(container, triangulatedPoints);

        auto stop = std::chrono::high_resolution_clock::now();
    
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

        for(int i=1; i<=n_drones; i++){
            std::string name = OUTPUT_DIR + "drone" + std::to_string(i) + ".ply";
            writeOutputFile(name.c_str(), triangulatedPoints[i-1]);
        }

        delete triangulator;
    }else if(n_drones == 1){
        Triangulator* triangulator;
        if(args.get("triangulator") == "matrix"){
            triangulator = new MatrixTriangulator(cameras);
        }else if(args.get("triangulator") == "ray"){
            triangulator = new RayTriangulator(cameras);
        }else{
            throw std::runtime_error("Invalid --triangulator argument. Allowed options are \'matrix\' and \'ray\'");
        }

        std::vector<std::vector<cv::Point2d>> dronePoints;
        loadPointsOneDrone(args.get("data_path").c_str(), dronePoints);

        auto start = std::chrono::high_resolution_clock::now();

        std::vector<cv::Point3d> triangulatedPoints = triangulator->triangulatePoints(dronePoints);

        auto stop = std::chrono::high_resolution_clock::now();

        auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

        std::string name = OUTPUT_DIR + std::string("drone.ply");
        writeOutputFile(name.c_str(), triangulatedPoints);

        delete triangulator;
    }

    // Free memory
    for(const auto& cam : cameras)
        delete cam;
}