#include <iostream>
#include "argparse.hpp"
#include <opencv4/opencv2/imgcodecs.hpp>
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
                n_cameras++;
                for (const auto & entry2 : fs::directory_iterator(entry)){
                    files.push_back(entry2.path().string());
                }
            }
        }

        std::sort(files.begin(), files.end(), [](const std::string &a, const std::string &b){   
            std::string file_a = a.substr(a.find_last_of("/\\") + 1);
            std::string file_b = b.substr(b.find_last_of("/\\") + 1);

            std::string frame_a = file_a.substr(file_a.find_last_of("_") + 1);
            std::string frame_b = file_b.substr(file_b.find_last_of("_") + 1);

            if(frame_a > frame_b){
                return true;
            }else if(frame_a == frame_b){
                std::string cam_a = file_a.substr(0, file_a.find_first_of("_"));
                std::string cam_b = file_b.substr(0, file_b.find_first_of("_"));
                if(cam_a > cam_b) return true;
                else return false;
            }else{
                return false;
            }
        });

        std::vector<std::ofstream*> writers;

        for(int i=0; i<n_cameras; i++){
            std::string filename = path + "cam" + std::to_string(i) + ".csv";
            std::ofstream* file = new std::ofstream(filename);
            (*file) << "frame_id; x1; y1; width; height; cx; cy; prob" << std::endl;
            writers.push_back(file);
        }

        int i=0;
        int camIdx = 0;
        for(const auto& f : files){
            if(i%8 == 0){
                camIdx++;
            }

            std::cout << f << std::endl;
            cv::Mat src, thresh;
            src = cv::imread(f);
            cv::threshold(src, thresh, 254, 255, cv::THRESH_BINARY_INV);

            cv::Mat points;
            cv::findNonZero(thresh, points);
            cv::Rect r = cv::boundingRect(points);

            (*writers[i%n_drones]) << r.x << ";" << r.y << ";" << r.width << ";" << r.height << ";" << r.x + r.width/2 << ";" << r.y + r.height/2 << ";1;" << std::endl;
        }

        for(auto& w : writers){
            w->close();
        }
    }
}