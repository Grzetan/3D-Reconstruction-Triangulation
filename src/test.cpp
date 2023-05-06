#include <iostream>
#include <fstream>
#include "happly.h"

void readInputCSV(const char* dir, std::vector<std::array<double, 3>>& path, int offset = 2){
    std::ifstream file(dir);
    std::string line, token;
    std::array<double, 3> point;
    int i=0;
    
    while (std::getline(file, line)){
        if(offset > i++) continue; // Skip first `offset` lines
        std::istringstream iss(line);
        std::vector<double> seperatedLine;

        while(std::getline(iss, token, ',')) {
            seperatedLine.push_back(std::stod(token));
        }

        if(seperatedLine.size() != 12){
            throw std::runtime_error("Error at CSV file");
        }

        point[0] = seperatedLine[0];
        point[1] = seperatedLine[1];
        point[2] = seperatedLine[2];
        path.push_back(point);
    }
}

int main(int argc, const char** argv){
    if(argc != 3) throw std::runtime_error("Input paths must be provided");

    happly::PLYData inputPLY(argv[1]);
    std::vector<std::array<double, 3>> predPath = inputPLY.getVertexPositions();
    std::vector<std::array<double, 3>> labelPath;
    readInputCSV(argv[2], labelPath);
    std::cout << labelPath[0][0] << ", " << labelPath[0][1] << ", " << labelPath[0][2] << std::endl;
    std::cout << labelPath.size();
}