#include <iostream>
#include "argparse.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
namespace fs = std::filesystem;

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
    
    // Generate detections from labels
    if(true){
        std::string path = args.get("path");
        std::vector<std::string> files;

        int n_drones = 0;

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

            if(frame_a < frame_b){
                return true;
            }else if(frame_a == frame_b){
                std::string cam_a = file_a.substr(0, file_a.find_first_of("_"));
                std::string cam_b = file_b.substr(0, file_b.find_first_of("_"));

                if(std::find(cameras.begin(), cameras.end(), cam_a) == cameras.end()) cameras.push_back(cam_a);
                if(std::find(cameras.begin(), cameras.end(), cam_b) == cameras.end()) cameras.push_back(cam_b);

                if(cam_a < cam_b) return true;
                else return false;
            }else{
                return false;
            }
        });

        std::vector<std::ofstream*> writers;

        for(int i=0; i<cameras.size(); i++){
            std::string filename = path + "cam" + std::to_string(i) + ".csv";
            std::ofstream* file = new std::ofstream(filename);
            (*file) << "frame_id; x1; y1; width; height; cx; cy; prob" << std::endl << "0;";
            writers.push_back(file);
        }

        int n_cameras = cameras.size();
        int i=0;
        int camIdx = 0;
        for(const auto& f : files){
            std::cout << f << std::endl;

            std::cout << i << ", " << camIdx << std::endl;

            cv::Mat src, thresh;
            src = cv::imread(f, 0);
            cv::threshold(src, thresh, 254, 255, cv::THRESH_BINARY);

            cv::Mat points;
            cv::findNonZero(thresh, points);
            cv::Rect r = cv::boundingRect(points);

            (*writers[camIdx%n_cameras]) << r.x << ";" << r.y << ";" << r.width << ";" << r.height << ";" << r.x + r.width/2 << ";" << r.y + r.height/2 << ";1;";
        
            if(++i%8 == 0){
                i=0;
                (*writers[camIdx%n_cameras]) << std::endl;
                camIdx++;
                (*writers[camIdx%n_cameras]) << camIdx / n_cameras << ";";
            }
        }

        for(auto& w : writers){
            w->close();
        }
    }
}