#include <iostream>
#include "argparse.hpp"
#include <opencv2/opencv.hpp>
#include "RayTriangulator.h"
#include "MatrixTriangulator.h"
#include "DroneClassifier.h"
#include "DetectionsContainer.h"
#include <filesystem>
#include <fstream>
namespace fs = std::filesystem;

// Kontakt z panem michalem odnosnie FOVa, czy pozycja kamery sie zgadza z obiektywem, wziac z aruco kalibracje, w ktorym miejscu przeskok jest
// przetestowac sciezki jak drony sa na srodku w miare
// przetestowac algo z roznymi ilosciami dronow: 1, 2, 4 itd

int main(int argc, const char** argv){
    argparse::ArgumentParser args("3D-Reconstruction-Triangulation"); 
    
    args.add_argument("path")
    .help("Path to the directory with data")
    .required();

    try {
        args.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    int n_drones = 0;
    std::string path = args.get("path");

    // Generate detections from labels
    if(true){
        std::vector<std::string> files;

        for (const auto & entry : fs::directory_iterator(path)){
            if(entry.path().string().find("mask") != std::string::npos){
                n_drones++;
                for (const auto & entry2 : fs::directory_iterator(entry)){
                    files.push_back(entry2.path().string());
                }
            }
        }

        std::vector<std::string> cameras = {};

        std::sort(files.begin(), files.end(), [&cameras](const std::string &a, const std::string &b){   
            std::string file_a = a.substr(a.find_last_of("/\\") + 1);
            std::string file_b = b.substr(b.find_last_of("/\\") + 1);

            int frame_a = std::stoi(file_a.substr(file_a.find_last_of("_") + 1));
            int frame_b = std::stoi(file_b.substr(file_b.find_last_of("_") + 1));

            std::string cam_a = file_a.substr(0, file_a.find_first_of("_"));
            std::string cam_b = file_b.substr(0, file_b.find_first_of("_"));

            if(std::find(cameras.begin(), cameras.end(), cam_a) == cameras.end()) cameras.push_back(cam_a);
            if(std::find(cameras.begin(), cameras.end(), cam_b) == cameras.end()) cameras.push_back(cam_b);

            if(frame_a < frame_b){
                return true;
            }else if(frame_a == frame_b){
                if(cam_a < cam_b) return true;
                else return false;
            }else{
                return false;
            }
        });

        std::vector<std::ofstream*> writers;

        if(!fs::exists(path + "referenceBB")){
            fs::create_directory(path + "referenceBB");
        }

        for(int i=0; i<cameras.size(); i++){
            std::string filename = path + "referenceBB/cam" + std::to_string(i) + ".csv";
            std::ofstream* file = new std::ofstream(filename);
            (*file) << "frame_id; x1; y1; width; height; cx; cy; prob" << std::endl;
            if(i==0)
                *(file) << "0;";
            writers.push_back(file);
        }

        int n_cameras = cameras.size();
        int i=0;
        int camIdx = 0;
        for(int fIdx=0; fIdx<files.size(); fIdx++){
            std::cout << files[fIdx] << std::endl;

            cv::Mat src, thresh;
            src = cv::imread(files[fIdx], 0);
            cv::threshold(src, thresh, 254, 255, cv::THRESH_BINARY);

            cv::Mat points;
            cv::findNonZero(thresh, points);
            cv::Rect r = cv::boundingRect(points);

            if(r.width != 0 && r.height != 0)
                (*writers[camIdx%n_cameras]) << r.x << ";" << r.y << ";" << r.width << ";" << r.height << ";" << r.x + r.width/2 << ";" << r.y + r.height/2 << ";1;";
        
            if(++i%n_drones == 0){
                i=0;
                (*writers[camIdx%n_cameras]) << std::endl;
                camIdx++;
                if(fIdx != files.size()-1)
                    (*writers[camIdx%n_cameras]) << camIdx / n_cameras << ";";
            }
        }

        for(auto& w : writers){
            w->close();
        }
    }else{
        for (const auto & entry : fs::directory_iterator(path)){
            if(entry.path().string().find("mask") != std::string::npos){
                n_drones++;
            }
        }
    }

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(std::string(path + "cameras.xml").c_str());
    Triangulator* triangulator = new RayTriangulator(cameras);

    if(n_drones==1){
        DetectionsContainer container(std::string(path + "referenceBB").c_str(), 1, 7);

        auto start = std::chrono::high_resolution_clock::now();

        auto data = container.getDataForTriangulation();

        std::vector<cv::Point3d> triangulatedPoints = triangulator->triangulatePoints(data);

        auto stop = std::chrono::high_resolution_clock::now();

        auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

        writeOutputFile("drone.ply", triangulatedPoints);
    }else{
        DroneClassifier classifier(triangulator, n_drones);

        DetectionsContainer container(std::string(path + "referenceBB_8drones").c_str(), 1, 7);

        auto start = std::chrono::high_resolution_clock::now();

        std::vector<std::vector<cv::Point3d>> triangulatedPoints;
        classifier.classifyDrones(container, triangulatedPoints);

        auto stop = std::chrono::high_resolution_clock::now();

        auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

        for(int i=1; i<=n_drones; i++){
            std::string name = "drone" + std::to_string(i) + ".ply";
            writeOutputFile(name.c_str(), triangulatedPoints[i-1]);
        }
    }

    delete triangulator;
}