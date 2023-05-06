#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include "PointTriangulator.h"
#include "Camera.h"
#include "pugixml.hpp"
#include "HasiecTriangulator.h"
using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

const tdr::Camera* createCamera(int id, size_t width, size_t height, double focalLength, cv::Mat translation, cv::Mat rotation){
    tdr::Camera* cam = new tdr::Camera(id);
    cam->width = width;
    cam->height = height;
    cam->fx = focalLength;
    cam->tvec = translation;
    cam->rvec = cam->toRotVec(rotation);
    cam->rquat = rotation;
    cam->compCamParams();

    return cam;
}

std::vector<const tdr::Camera*> loadCamerasXML(const char* path){
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path);
    if (!result)
        throw std::runtime_error("Cannot open camera XML file");

    std::vector<const tdr::Camera*> cameras;

    for (pugi::xml_node camera : doc.child("Cameras").children("Camera")){
        pugi::xml_node controlFrame = camera.child("ControlFrames").child("ControlFrame");

        if(!controlFrame) continue;

        // Read camera's id
        int id = camera.attribute("DEVICEID").as_int();

        // Read focal length
        double focalLength = controlFrame.attribute("FOCAL_LENGTH").as_double();

        // Read width and height
        std::stringstream princialPoint = std::stringstream(controlFrame.attribute("PRINCIPAL_POINT").value());
        int width, height;
        princialPoint >> width >> height;
        width *= 2;
        height *= 2;

        // Read position
        std::stringstream position = std::stringstream(controlFrame.attribute("POSITION").value());
        double x, y, z;
        position >> x >> y >> z;

        // Read orientation
        std::stringstream orientation = std::stringstream(controlFrame.attribute("ORIENTATION").value());
        double w, i, j, k;
        orientation >> i >> j >> k >> w;

        const tdr::Camera* cam = createCamera(id, width, height, focalLength, (cv::Mat_<double>(3, 1) << x, y, z), (cv::Mat_<double>(4, 1) << w, -i, -j, -k));
        cameras.push_back(cam);
    }

    return cameras;
}

void load2DPoints(const char* path, std::vector<std::vector<cv::Point2d>>& points, int offset = 1){
    std::vector<std::string> files;
    for (const auto& dirEntry : recursive_directory_iterator(path)){
        std::string directory = dirEntry.path().u8string();
        if(directory.find(".csv") != std::string::npos && directory.find(".avi") == std::string::npos){
            files.push_back(directory);
        }
    }

    std::sort(files.begin(), files.end());
    std::string line, token;
    int frame, x1, y1, x2, y2;

    for(auto& f : files){
        int i=0;
        std::ifstream file(f);
        points.push_back({}); // Add new vector for current camera

        // int n_lines = 200;

        while (std::getline(file, line)){
            if(offset > i++) continue; // Skip first `offset` lines
            std::istringstream iss(line);
            std::vector<int> seperatedLine;

            while(std::getline(iss, token, ',')) {
                seperatedLine.push_back(std::stoi(token));
            }

            cv::Point2d point;
            if(seperatedLine.size() < 5){
                point.x = -1;
                point.y = -1;
            }else{
                point.x = seperatedLine[1] + (seperatedLine[3] - seperatedLine[1]) / 2; 
                point.y = seperatedLine[2] + (seperatedLine[4] - seperatedLine[2]) / 2;
            }
            points[points.size()-1].push_back(point);
        }
    }
}

void writeOutputFile(const char* path, const std::vector<cv::Point3d>& triangulatedPoints, const std::vector<const tdr::Camera*>& cameras = {}){
    std::ofstream out(path);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << triangulatedPoints.size() + cameras.size() * 3 << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << cameras.size() << "\n";
    out << "property list uchar int vertex_index\n";
    out << "end_header\n";

    double camSize = 0.5;
    double scale = 0.01;

    // Visualize cameras
    for(const auto& cam : cameras){
        double x = cam->camPos.at<double>(0,0) * scale;
        double y = cam->camPos.at<double>(1,0) * scale;
        double z = cam->camPos.at<double>(2,0) * scale;

        out << x - camSize << " " << y - camSize << " " << z - camSize << "\n";
        out << x + camSize << " " << y + camSize << " " << z + camSize << "\n";
        out << "0 0 0" << "\n";
    }

    // Visualize paths
    for(const auto& p : triangulatedPoints){
        out << p.x * scale << " " << p.y * scale << " " << p.z * scale << "\n";
    }

    for(int i=0; i<cameras.size(); i++){
        out << "3 " << i*3 << " " << i*3 + 1 << " " << i*3 + 2 << "\n";
    }
}

int main(int argc, const char** argv){
    if(argc != 3 && argc != 4) throw std::runtime_error("Input paths must be provided");

    std::vector<const tdr::Camera*> cameras = loadCamerasXML(argv[1]);

    // First dim = n_camera, second_dim = n_points
    std::vector<std::vector<cv::Point2d>> dronePoints;
    load2DPoints(argv[2], dronePoints);
    HasiecTriangulator projector2(cameras);
   // PointTriangulator projector(cameras);
    std::vector<cv::Point3d> triangulatedPoints2 = projector2.triangulatePoints(dronePoints);
   // std::vector<cv::Point3d> triangulatedPoints = projector.triangulatePoints(dronePoints);
    
    const char* path = (argc == 4) ? argv[3] : "output.ply";
    writeOutputFile(path, triangulatedPoints2);
    std::string pathPrefix = "hasiec_";
    writeOutputFile(pathPrefix.append(path).c_str(), triangulatedPoints2);
}