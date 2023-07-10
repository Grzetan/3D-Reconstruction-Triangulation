#pragma once

#include <iostream>
#include <queue>
#include "Camera.h"
#include "Triangulator.h"

# define THRESHOLD 1e-4 //5e-13 for squared
# define MAX_ITERATIONS 1000
# define SQUARED false

class RayTriangulator : public Triangulator{
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
    };

public:
	RayTriangulator(std::vector<const tdr::Camera*> cameras_);

	std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) override;

    std::pair<cv::Point3d, double> triangulatePoint(std::vector<CamPointPair> images) override;
};
