#include "iostream"
#include <opencv2/opencv.hpp>
#include "PointTriangulator.h"
#include "Camera.h"
#include "pugixml.hpp"

tdr::Camera* createCamera(size_t width, size_t height, double fovx, cv::Mat translation, cv::Mat rotation){
    tdr::Camera* cam = new tdr::Camera(0);
    cam->width = width;
    cam->height = height;
    cam->fovx = fovx;
    cam->tvec = translation;
    cam->rvec = cam->toRotVec(rotation);
    cam->rquat = rotation;
    cam->compCamParams();

    return cam;
}

int main(){
    tdr::Camera* cam1 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));
    tdr::Camera* cam2 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 200, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));
    // tdr::Camera* cam3 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 10, -3, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));

    // tdr::Camera* cam1 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(4, 1) << 0.854, -0.146, 0.354, 0.354));
    // tdr::Camera* cam2 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 20, 0, 0), (cv::Mat_<double>(4, 1) << 0.354, -0.354, 0.146, 0.854));
    // tdr::Camera* cam3 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 20, 20, 0), (cv::Mat_<double>(4, 1) << -0.354, -0.354, -0.146, 0.854));
    // tdr::Camera* cam4 = createCamera(640, 480, 70, (cv::Mat_<double>(3, 1) << 0, 20, 0), (cv::Mat_<double>(4, 1) << 0.854, 0.146, 0.354, -0.354));

    PointTriangulator projector({cam1, cam2});

    std::vector<std::vector<cv::Point2d>> points = { { {639, 240}, {0, 240} } };


    std::vector<cv::Point3d> triangulated = projector.triangulatePoints(points);

    for(const auto& p : triangulated){
        std::cout << "Triangulated point: " << p << std::endl;
    }

    return 0;
}