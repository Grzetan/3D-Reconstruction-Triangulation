#pragma once

#include <iostream>
#include "Camera.h"

struct Ray {
    cv::Point3d dir;
    cv::Point3d origin;
};

class RayClosestPoint : public cv::LMSolver::Callback {
public:
    explicit RayClosestPoint(const std::vector<Ray>& rays) : rays_(rays) {}

    bool compute(cv::InputArray _x, cv::OutputArray _f, cv::OutputArray _jacobian = cv::noArray()) const override {
        cv::Vec3d x = _x.getMat().ptr<cv::Vec3d>()[0];
        cv::Mat1d f(rays_.size(), 1);

        //cv::Mat1d jacobian(rays_.size(), 3);

        //for (size_t i = 0; i < rays_.size(); i++) {
        //    cv::Vec3d origin = rays_[i].origin;
        //    cv::Vec3d direction = rays_[i].dir;
        //    cv::Vec3d ray_direction = direction / cv::norm(direction);

        //    cv::Vec3d origin_to_point = x - origin;
        //    double distance = origin_to_point.dot(ray_direction);
        //    cv::Vec3d closest_point = origin + distance * ray_direction;

        //    f(i, 0) = cv::norm(closest_point - x);

        //    jacobian.row(i) = (closest_point - x).t();
        //}

        //_f.assign(f);
        //_jacobian.assign(jacobian);

        for (size_t i = 0; i < rays_.size(); i++) {
            cv::Point3d p = rays_[i].dir.cross(x - cv::Vec3d(rays_[i].origin));

            f(i, 0) = std::pow(cv::norm(p), 2);
        }

        _f.assign(f);

        return true;
    }

private:
    std::vector<Ray> rays_;
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

    std::string type2str(int type) {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }

    cv::Vec3d calculateRayDirectionForPoint(cv::Mat perspectiveMatrix, cv::Point2d point) {
        cv::Mat p(1, 3, CV_64F);
        p.at<double>(0, 0) = point.x;
        p.at<double>(0, 1) = point.y;
        p.at<double>(0, 2) = 1;

        cv::Mat homoPoint;

        std::cout << perspectiveMatrix.rows << ", " << perspectiveMatrix.cols << std::endl;
        // cv::Mat inversedPerspectiveMatrix = perspectiveMatrix.inv();
        // cv::invert(perspectiveMatrix, inversedPerspectiveMatrix);

        // cv::perspectiveTransform(p, homoPoint, inversedPerspectiveMatrix);

        cv::Vec3d rayDir = {
            homoPoint.at<double>(0, 0) / homoPoint.at<double>(0, 3),
            homoPoint.at<double>(0, 1) / homoPoint.at<double>(0, 3),
            homoPoint.at<double>(0, 2) / homoPoint.at<double>(0, 3)
        };

        return rayDir;
    }

    // https://computergraphics.stackexchange.com/questions/8479/how-to-calculate-ray
    cv::Vec3d calculateRayDirectionForPoint2(const tdr::Camera* cam, cv::Point2d point){
        // Add 0.5 to get center of pixel
        point.x += 0.5;
        point.y += 0.5; 

        double d = 1 / std::tan(cam->fovx / 2);
        cv::Vec3d ray;
        ray[0] = ((double)cam->width / (double)cam->height) * (2 * point.x / (double)cam->width) - 1;
        ray[1] = (2 * point.y / (double)cam->height) - 1;
        ray[2] = d;
        ray = ray / cv::norm(ray);

        return ray;
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
                // cv::Mat perspectiveMatrix = cameras[i]->cameraPerspectiveMatrix;

                cv::Point3d origin;
                origin.x = cameras[i]->camPos.at<double>(0);
                origin.y = cameras[i]->camPos.at<double>(0);
                origin.z = cameras[i]->camPos.at<double>(0);

                rays.push_back({ origin, calculateRayDirectionForPoint2(cameras[i], p[i]) });
            }

            if (rays.size() < 2) {
                throw std::runtime_error("Too few rays are found");
            }

            // result.push_back(triangulatePoint(rays));
		}

        return result;
	}
};