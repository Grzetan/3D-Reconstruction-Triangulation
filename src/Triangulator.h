#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

#include "Camera.h"

class Triangulator {
 public:
  struct CamPointPair {
    const tdr::Camera* camera;
    cv::Point2d point;
    cv::Mat projectionMatrix() { return camera->cameraPerspectiveMatrix; };
  };

  struct Ray {
    cv::Point3d origin;
    cv::Point3d dir;
  };

 protected:
  std::string type_;
  std::vector<cv::Mat>
      projection_matrices;  //< vector of projection matrices in mocap system
  std::vector<const tdr::Camera*> cameras;

  static double distToRay(const Ray& r, const cv::Vec3d& p,
                          bool squared = false);

  static double distToRay_(const Ray& r, const cv::Vec3d p,
                           bool squared = false);

  static cv::Vec3d rotatePointByQuaternion(cv::Vec3d point, const cv::Mat quat);

  // https://computergraphics.stackexchange.com/questions/8479/how-to-calculate-ray
  static cv::Vec3d calculateRayDirectionForPixel(const tdr::Camera* cam,
                                                 const cv::Point2d& point);

  static Ray createRayForPoint(const tdr::Camera* cam,
                               const cv::Point2d& point);

 public:
  std::vector<const tdr::Camera*> getCameras() { return cameras; }

  virtual std::pair<cv::Point3d, double> triangulatePoint(
      vector<CamPointPair> images) = 0;

  /*! \brief method to 3D reconstruction based on vecotr of 2D positions
   */
  virtual std::vector<cv::Point3d> triangulatePoints(
      std::vector<std::vector<cv::Point2d>> points) = 0;

  const tdr::Camera* getCamera(int camera) { return cameras[camera]; }

  std::string getType() { return type_; };

  static double getDistFromRay(CamPointPair pair, cv::Point3d point);
};