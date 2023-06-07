#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "argparse.hpp"
#include "happly.h"
#include "utils.h"

int main(int argc, const char** argv){
    argparse::ArgumentParser args("3D-Reconstruction-Triangulation"); 

    args.add_argument("pred_path")
    .help("Path to the PLY file with predictions")
    .required();

    args.add_argument("label_path")
    .help("Path to the CSV file from Vicon")
    .required();

    args.add_argument("--frequency")
    .help("How many lines in label file correspond to one frame")
    .scan<'i', int>()
    .default_value(1);

    args.add_argument("--output")
    .help("Name of the optional output file")
    .default_value<std::string>("None");

    try {
        args.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    happly::PLYData inputPLY(args.get("pred_path").c_str());
    auto predPathHapply = inputPLY.getVertexPositions();
    Path predPath;
    for(const auto& p : predPathHapply){
        predPath.push_back({p[0], p[1], p[2]});
    }

    Path labelPath;
    readInputCSV(args.get("label_path").c_str(), labelPath, args.get<int>("--frequency"));

    double error = calculateError(labelPath, predPath);
    std::cout << "Error: " << error << std::endl;

    if(args.get<std::string>("--output") != "None"){
        writeOutputFile(args.get<std::string>("--output").c_str(), labelPath);
    }
}