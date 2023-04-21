#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include "PointTriangulator.h"
#include "Camera.h"
#include "pugixml.hpp"

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

        const tdr::Camera* cam = createCamera(id, width, height, focalLength, (cv::Mat_<double>(3, 1) << x, y, z), (cv::Mat_<double>(4, 1) << w, i, j, k));
        cameras.push_back(cam);
    }

    return cameras;
}

void load2DPoints(const char* path, std::vector<std::vector<cv::Point2d>>& points, int offset = 5){
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

int main(){
    // tdr::Camera* cam1 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));
    // tdr::Camera* cam2 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 200, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));
    // tdr::Camera* cam3 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 10, -3, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));

    // tdr::Camera* cam1 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(4, 1) << 0.854, -0.146, 0.354, 0.354));
    // tdr::Camera* cam2 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 20, 0, 0), (cv::Mat_<double>(4, 1) << 0.354, -0.354, 0.146, 0.854));
    // tdr::Camera* cam3 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 20, 20, 0), (cv::Mat_<double>(4, 1) << -0.354, -0.354, -0.146, 0.854));
    // tdr::Camera* cam4 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 20, 0), (cv::Mat_<double>(4, 1) << 0.854, 0.146, 0.354, -0.354));

    // PointTriangulator projector({cam1, cam2});

    // std::vector<std::vector<cv::Point2d>> points = { { {639, 240}, {0, 240} } };


    // std::vector<cv::Point3d> triangulated = projector.triangulatePoints(points);

    // for(const auto& p : triangulated){
    //     std::cout << "Triangulated point: " << p << std::endl;
    // }

    std::vector<const tdr::Camera*> cameras = loadCamerasXML("Dron T02.xcp");

    // First dim = n_camera, second_dim = n_points
    // std::vector<std::vector<cv::Point2d>> drones2D;
    // load2DPoints("./referenceBB", drones2D);

    // for(int i=0; i<5; i++){
    //     for(int j=0; j<drones2D.size(); j++){
    //         std::cout << drones2D[j][i] << ", ";
    //     }
    //     std::cout << std::endl;
    // }

    for(const auto& cam : cameras){
        std::cout << cam->getCamId() << std::endl << "Position: \n" << cam->tvec << std::endl << "\nOrientation: \n" << cam->rquat << std::endl << "\n\n\n";
    }


    // Second and third camera is inversed???
    PointTriangulator projector(cameras);

    std::vector<std::vector<cv::Point2d>> points = { { {962, 541} }, { {962, 541} }, { {962, 541} }, { {962, 541} } };

    std::vector<cv::Point3d> triangulated = projector.triangulatePoints(points);

    for(const auto& p : triangulated){
        std::cout << "Triangulated point: " << p << std::endl;
    }


    return 0;
}