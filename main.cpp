#include "iostream"
#include <opencv2/opencv.hpp>
#include "PointTriangulator.h"
#include "Camera.h"

tdr::Camera* createCamera(size_t width, size_t height, double fovx, cv::Mat translation, cv::Mat rotation){
    tdr::Camera* cam = new tdr::Camera(0);
    cam->width = 640;
    cam->height = 480;
    cam->fovx = 50;
    cam->tvec = translation;
    cam->rvec = cam->toRotVec(rotation);
    cam->rquat = rotation;
    cam->compCamParams();

    return cam;
}

int main(){
    tdr::Camera* cam1 = createCamera(640, 480, 50, (cv::Mat_<double>(3, 1) << -10, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));
    tdr::Camera* cam2 = createCamera(640, 480, 50, (cv::Mat_<double>(3, 1) << -20, 0, 0), (cv::Mat_<double>(4, 1) << 0.7071, 0, 0.7071, 0));

    PointTriangulator projector({cam1, cam2});

    std::vector<std::vector<cv::Point2d>> points = { { {20, 100}, {600, 100} } };

    std::vector<cv::Point3d> triangulated = projector.triangulatePoints(points);

    return 0;
}