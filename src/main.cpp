#include "iostream"
#include <opencv2/opencv.hpp>
#include "PointTriangulator.h"
#include "Camera.h"
#include "pugixml.hpp"

tdr::Camera* createCamera(int id, size_t width, size_t height, double focalLength, cv::Mat translation, cv::Mat rotation){
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

std::vector<tdr::Camera*> loadCamerasXML(const char* path){
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path);
    if (!result)
        throw std::runtime_error("Cannot open camera XML file");

    std::vector<tdr::Camera*> cameras;

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
        orientation >> w >> i >> j >> k;

        tdr::Camera* cam = createCamera(id, width, height, focalLength, (cv::Mat_<double>(3, 1) << x, y, z), (cv::Mat_<double>(4, 1) << w, i, j, k));
        cameras.push_back(cam);
    }

    return cameras;
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

    std::vector<tdr::Camera*> cameras = loadCamerasXML("Dron T02.xcp");
    for(const auto& cam : cameras){
        std::cout << cam->fovx << ", " << cam->fovy << ", " << std::endl;
    }

    return 0;
}