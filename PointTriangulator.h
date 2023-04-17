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

    // Find orientation of camera by multiplying 1, 0, 0 by its roatation quaternion
    // Calculate rotation between camera rotation and 0, 0, -1
    // Rotate pixel ray by this rotation

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
        ray[0] = ((double)cam->width / (double)cam->height) * ((2 * p.x / (double)cam->width) - 1);
        ray[1] = (2 * (cam->height - p.y) / (double)cam->height) - 1;
        ray[2] = -d;
        return cv::normalize(ray);
    }

    // Find camera orientation by multiplying rotation quaternion with default vector 1, 0, 0
    cv::Mat findRotationQuaternion(const cv::Mat& cameraQuat){
        cv::Vec3d defaultVec(1, 0, 0);
        cv::Vec3d cameraOrientation = rotatePointByQuaternion(defaultVec, cameraQuat);

        cv::Mat rotQuat(4, 1, CV_64F);
        cv::Vec3d defaultCameraOrientation(0, 0, -1);

        cv::Vec3d n = defaultCameraOrientation.cross(cameraOrientation);
        double c = defaultCameraOrientation.dot(cameraOrientation);

        rotQuat.at<double>(0, 0) = std::sqrt((1 + c) / 2.0);
        rotQuat.at<double>(1, 0) = n[0] + std::sqrt((1 - c) / 2.0);
        rotQuat.at<double>(2, 0) = n[1] + std::sqrt((1 - c) / 2.0);
        rotQuat.at<double>(3, 0) = n[2] + std::sqrt((1 - c) / 2.0);

        rotQuat = rotQuat / cv::norm(rotQuat);
        return rotQuat;
    }

    Ray createRayForPoint(const tdr::Camera* cam, const cv::Point2d& point){
        cv::Vec3d pixelDir = calculateRayDirectionForPixel(cam, point);
        cv::Mat rotQuat = findRotationQuaternion(cam->rquat);

        cv::Point3d origin;
        origin.x = cam->tvec.at<double>(0);
        origin.y = cam->tvec.at<double>(1);
        origin.z = cam->tvec.at<double>(2);
        return {origin, rotatePointByQuaternion(pixelDir, rotQuat)};
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

            if (rays.size() < 2) {
                throw std::runtime_error("Too few rays are found");
            }

            for(const auto& ray : rays){
                std::cout << ray.origin << " : " << ray.dir << std::endl;
            }


            // cv::Mat mat = tdr::Camera::toRotMatrix(cameras[0]->rquat);
            // cv::Vec3d pt(1, 0, 0);
            // std::cout << mat * pt << std::endl;

            // std::cout << rotatePointByQuaternion(pt, cameras[0]->rquat) << std::endl;
            // std::cout << findRotationQuaternion(cameras[0]->rquat) << std::endl;
            // std::cout << rotateVectorByQuaternion({1, 0, 0}, cameras[0]->rquat) << std::endl;
            // std::cout << "HALO: " << rotateVectorByQuaternion(r, cameras[0]->rquat) << std::endl;
            // result.push_back(triangulatePoint(rays));
		}

        return result;
	}
};