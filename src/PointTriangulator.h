#pragma once

#include <iostream>
#include "Camera.h"

# define PI 3.14159265358979323846

struct Ray {
    cv::Point3d origin;
    cv::Point3d dir;
};

class RayClosestPoint : public cv::LMSolver::Callback {
public:
    explicit RayClosestPoint(const std::vector<Ray>& rays, const double epsilon=1e-14) : rays_(rays), epsilon_(epsilon) {}

    bool compute(cv::InputArray params, cv::OutputArray err, cv::OutputArray J) const override {
        cv::Mat paramMat = params.getMat().reshape(1, 1);
        const double x = paramMat.at<double>(0, 0);
        const double y = paramMat.at<double>(0, 1);
        const double z = paramMat.at<double>(0, 2);
        const cv::Vec3d currentPoint(x, y, z);

        err.create(static_cast<int>(rays_.size()), 1, CV_64FC1);
        cv::Mat errMat = err.getMat();
        errMat.forEach<double>([&](double& e, const int* position) {
            e = distToRay(rays_[position[0]], currentPoint);
        });

        if (J.needed())
        {
            J.create(errMat.rows, paramMat.cols, CV_64FC1);
            cv::Mat Jmat = J.getMat();
            for(int row = 0 ; row<Jmat.rows ; ++row){
                double* pJ = Jmat.ptr<double>(row);

                *pJ++ = (distToRay_(rays_[row], {x+epsilon_, y, z}) - distToRay_(rays_[row], {x-epsilon_, y, z})) / (2*epsilon_);
                *pJ++ = (distToRay_(rays_[row], {x, y+epsilon_, z}) - distToRay_(rays_[row], {x, y-epsilon_, z})) / (2*epsilon_);
                *pJ++ = (distToRay_(rays_[row], {x, y, z+epsilon_}) - distToRay_(rays_[row], {x, y, z-epsilon_})) / (2*epsilon_);
            }
        }

        return true;
    }

private:
    std::vector<Ray> rays_;
    const double epsilon_;

    static double distToRay(const Ray& r, const cv::Vec3d& p, bool squared=true){
        cv::Point3d result = r.dir.cross(p - cv::Vec3d(r.origin));
        if(squared) return result.x*result.x + result.y*result.y + result.z*result.z;
        return std::sqrt(result.x*result.x + result.y*result.y + result.z*result.z);
    }

    static double distToRay_(const Ray& r, const cv::Vec3d p, bool squared=true){
        return distToRay(r, p, squared);
    }
};

class PointTriangulator {
	std::vector<const tdr::Camera*> cameras;

    cv::Point3d triangulatePoint(std::vector<Ray>& rays) {
        cv::Point3d initGuess;

        for (const auto& ray : rays) {
            initGuess += ray.origin;
        }

        initGuess.x /= rays.size();
        initGuess.y /= rays.size();
        initGuess.z /= rays.size();

        cv::Ptr<cv::LMSolver> solver = cv::LMSolver::create(cv::Ptr<RayClosestPoint>(new RayClosestPoint(rays)), 1000);
        std::vector<double> params = { initGuess.x, initGuess.y, initGuess.z };
        int r = solver->run(params);

        return { params[0], params[1], params[2] };
    }

    // Not sure if it works
    cv::Vec3d calculateRayDirectionForPoint(cv::Mat perspectiveMatrix, cv::Point2d point) {
        cv::Mat p(1, 3, CV_64F);
        p.at<double>(0, 0) = point.x;
        p.at<double>(0, 1) = point.y;
        p.at<double>(0, 2) = 1;

        cv::Mat homoPoint;

        std::cout << perspectiveMatrix.rows << ", " << perspectiveMatrix.cols << std::endl;
        cv::Mat inversedPerspectiveMatrix = perspectiveMatrix.inv();
        // cv::invert(perspectiveMatrix, inversedPerspectiveMatrix);

        cv::perspectiveTransform(p, homoPoint, inversedPerspectiveMatrix);

        cv::Vec3d rayDir = {
            homoPoint.at<double>(0, 0) / homoPoint.at<double>(0, 3),
            homoPoint.at<double>(0, 1) / homoPoint.at<double>(0, 3),
            homoPoint.at<double>(0, 2) / homoPoint.at<double>(0, 3)
        };

        return rayDir;
    }

    cv::Vec3d rotatePointByQuaternion(cv::Vec3d point, const cv::Mat quat)
    {

        cv::Vec4d quaternion(quat.at<double>(0,0), quat.at<double>(1,0), quat.at<double>(2,0), quat.at<double>(3,0));
        cv::Vec4d point_quaternion(0, point[0], point[1], point[2]);
        cv::Vec4d rotated_point_quaternion = quaternion * point_quaternion * quaternion.conj();

        return {rotated_point_quaternion[1], rotated_point_quaternion[2], rotated_point_quaternion[3]};
    }

    // https://computergraphics.stackexchange.com/questions/8479/how-to-calculate-ray
    cv::Vec3d calculateRayDirectionForPixel(const tdr::Camera* cam, const cv::Point2d& point){
        // Add 0.5 to get center of pixel
        cv::Point2d p(point.x, point.y);
        p.x += 0.5;
        p.y += 0.5; 

        double d = 1 / std::tan(cam->fovx*0.0174533 / 2);
        cv::Vec3d ray;
        // We have to adjust coordinate space so it fits identity quaternion
        double x = ((double)cam->width / (double)cam->height) * ((2 * p.x / (double)cam->width) - 1);
        double y = (2 * (cam->height - p.y) / (double)cam->height) - 1;
        ray[0] = d;
        ray[1] = y;
        ray[2] = x;
        return cv::normalize(ray);
    }

    Ray createRayForPoint(const tdr::Camera* cam, const cv::Point2d& point){
        cv::Vec3d pixelDir = calculateRayDirectionForPixel(cam, point);

        cv::Point3d origin;
        origin.x = cam->tvec.at<double>(0);
        origin.y = cam->tvec.at<double>(1);
        origin.z = cam->tvec.at<double>(2);
        return {origin, rotatePointByQuaternion(pixelDir, cam->rquat)};
    }


public:
	PointTriangulator(std::vector<const tdr::Camera*> cameras_) : cameras(cameras_) {};

	// first dim = n_points
	// second dim = n_cameras
	std::vector<cv::Point3d> triangulatePoints(std::vector<std::vector<cv::Point2d>> points) {
		std::vector<cv::Point3d> result;

		for (const auto& p : points) {
            std::vector<Ray> rays;
            for (int i = 0; i < p.size(); i++) {
                if (p[i].x == -1 || p[i].y == -1) continue;

                rays.push_back(createRayForPoint(cameras[i], p[i]));
            }

            for(const auto& ray : rays){
                std::cout << ray.origin << " = " << ray.origin + 50*ray.dir << std::endl;
            }
            
            if (rays.size() < 2) {
                throw std::runtime_error("Too few rays are found");
            }

            result.push_back(triangulatePoint(rays));
		}

        return result;
	}
};