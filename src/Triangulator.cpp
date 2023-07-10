#include "Triangulator.h"

double Triangulator::distToRay(const Ray& r, const cv::Vec3d& p, bool squared){
    cv::Point3d result = r.dir.cross(p - cv::Vec3d(r.origin));
    if(squared) return result.x*result.x + result.y*result.y + result.z*result.z;
    return std::sqrt(result.x*result.x + result.y*result.y + result.z*result.z);
}

double Triangulator::distToRay_(const Ray& r, const cv::Vec3d p, bool squared){
    return distToRay(r, p, squared);
}

cv::Vec3d Triangulator::rotatePointByQuaternion(cv::Vec3d point, const cv::Mat quat){
    cv::Vec4d quaternion(quat.at<double>(0,0), quat.at<double>(1,0), quat.at<double>(2,0), quat.at<double>(3,0));
    cv::Vec4d point_quaternion(0, point[0], point[1], point[2]);
    cv::Vec4d rotated_point_quaternion = quaternion * point_quaternion * quaternion.conj();

    return {rotated_point_quaternion[1], rotated_point_quaternion[2], rotated_point_quaternion[3]};
}

cv::Vec3d Triangulator::calculateRayDirectionForPixel(const tdr::Camera* cam, const cv::Point2d& point){
    // Add 0.5 to get center of pixel
    cv::Point2d p(point.x, point.y);
    p.x += 0.5;
    p.y += 0.5; 

    double d = 1 / std::tan(cam->fovy*0.0174533 / 2);
    cv::Vec3d ray;
    // We have to adjust coordinate space so it fits identity quaternion
    double x = ((double)cam->width / (double)cam->height) * ((2 * p.x / (double)cam->width) - 1);
    double y = (2 * p.y / (double)cam->height) - 1;
    ray[0] = x;
    ray[1] = y;
    ray[2] = d;
    return cv::normalize(ray);
}

Triangulator::Ray Triangulator::createRayForPoint(const tdr::Camera* cam, const cv::Point2d& point){
    cv::Vec3d pixelDir = calculateRayDirectionForPixel(cam, point);

    cv::Point3d origin;
    origin.x = cam->tvec.at<double>(0);
    origin.y = cam->tvec.at<double>(1);
    origin.z = cam->tvec.at<double>(2);
    return {origin, rotatePointByQuaternion(pixelDir, cam->rquat)};
}

double Triangulator::getDistFromRay(CamPointPair pair, cv::Point3d point){
    Ray ray = createRayForPoint(pair.camera, pair.point);

    return distToRay(ray, point);
}

