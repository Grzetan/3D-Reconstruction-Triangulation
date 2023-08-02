#include <iostream>
#include "argparse.hpp"
#include <filesystem>
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
        for (const auto & entry : fs::directory_iterator(path)){
            if(entry.path().string().find("mask") != std::string::npos)
                std::cout << entry.path().string() << std::endl;
        }
    }
}