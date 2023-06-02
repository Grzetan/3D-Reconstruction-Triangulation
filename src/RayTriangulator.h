#pragma once

#include <iostream>
#include <queue>
#include "Camera.h"
#include "Triangulator.h"

# define THRESHOLD 1e-4 //5e-13 for squared
# define MAX_ITERATIONS 1000
# define SQUARED false

class RayTriangulator : public Triangulator{
    struct Ray {
        cv::Point3d origin;
        cv::Point3d dir;
    };

    class RayClosestPoint : public cv::LMSolver::Callback {
    public:
        explicit RayClosestPoint(const std::vector<Ray>& rays, const double epsilon=THRESHOLD, const bool squared=SQUARED) : rays_(rays), epsilon_(epsilon), squared_(squared) {}

        bool compute(cv::InputArray params, cv::OutputArray err, cv::OutputArray J) const override;

        double getError();

    private:
        std::vector<Ray> rays_;
        const double epsilon_;
        const bool squared_;
        mutable double error;

        static double distToRay(const Ray& r, const cv::Vec3d& p, bool squared=false);

        static double distToRay_(const Ray& r, const cv::Vec3d p, bool squared=false);
    };

    cv::Vec3d rotatePointByQuaternion(cv::Vec3d point, const cv::Mat quat);

    // https://computergraphics.stackexchange.com/questions/8479/how-to-calculate-ray
    cv::Vec3d calculateRayDirectionForPixel(const tdr::Camera* cam, const cv::Point2d& point);

    Ray createRayForPoint(const tdr::Camera* cam, const cv::Point2d& point);

    bool increment(std::vector<size_t>& combination, std::vector<size_t>& sizes);

public:
	RayTriangulator(std::vector<const tdr::Camera*> cameras_);

	std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) override;

    std::pair<cv::Point3d, double> triangulatePoint(std::vector<CamPointPair> images) override;
      
};
