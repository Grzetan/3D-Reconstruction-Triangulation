#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "happly.h"
#include "utils.h"

int main(int argc, const char** argv){
    if(argc != 3) throw std::runtime_error("Input paths must be provided");

    // happly::PLYData inputPLY(argv[1]);
    // auto predPathHapply = inputPLY.getVertexPositions();
    // Path predPath;
    // for(const auto& p : predPathHapply){
    //     predPath.push_back({p[0], p[1], p[2]});
    // }

    Path labelPath;
    readInputCSV(argv[2], labelPath);

    // double error = calculateError(labelPath, predPath);
    // std::cout << error << std::endl;

    // Use this to save label points to PLY file
    writeOutputFile("dronT02_label.ply", labelPath);
}