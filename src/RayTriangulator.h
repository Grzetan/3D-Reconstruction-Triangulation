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

    // void triangulateMultipleDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones){
    //     // Add new vector for every drone
    //     for(int i=0; i<n_drones; i++){
    //         triangulatedPoints.push_back({});
    //     }
        
    //     for(int frame=0; frame<points[0].size(); frame++){
    //         std::cout << frame << std::endl;
    //         std::vector<size_t> combination(points.size());
    //         std::vector<size_t> n_detections(points.size());
    //         for(int i=0; i<points.size(); i++){
    //             n_detections[i] = points[i][frame].size() + 1; // Plus one for no detection
    //         }

    //         std::priority_queue<Combination> combinationsQueue;

    //         do{
    //             // Skip combination if number of cameras is not enough
    //             int count = std::count_if(combination.begin(), combination.end(), [&](size_t &i) {
    //                 return i > 0;
    //             });
    //             if(count < MIN_CAMERAS) continue;

    //             // Create rays for every bounding box in this combination
    //             std::vector<Ray> rays;
    //             for(int i=0; i<combination.size(); i++){
    //                 if(combination[i] == 0) continue; // First index means no detection so we can skip it
    //                 rays.push_back(createRayForPoint(cameras[i], points[i][frame][combination[i]-1]));
    //             }

    //             std::pair<cv::Point3d, double> pointWithError = triangulatePoint(rays);
    //             if(frame == 0)
    //                 combinationsQueue.push({combination, pointWithError.first, pointWithError.second});
    //             else{
    //                 size_t bestPath = 0;
    //                 double bestDist = 1e+6;
    //                 for(int j=0; j<n_drones; j++){
    //                     double dist = cv::norm(triangulatedPoints[j].back() - pointWithError.first);
    //                     if(dist < bestDist){
    //                         bestDist = dist;
    //                         bestPath = j;
    //                     }
    //                 }
    //                 combinationsQueue.push({combination, pointWithError.first, pointWithError.second, bestPath});
    //             }
    //         }while(increment(combination, n_detections));

    //         // Pick n_drones best combinations
    //         std::vector<Combination> finalCombinations;
    //         while(!combinationsQueue.empty() && finalCombinations.size() < n_drones){
    //             Combination c = combinationsQueue.top();
    //             if(c.isCombinationUnique(finalCombinations) && c.error < MAX_ERROR) finalCombinations.push_back(c);
    //             combinationsQueue.pop();
    //         }

    //         // Pick best drone path for each new point
    //         if(frame == 0){
    //             for(int i=0; i<finalCombinations.size(); i++){
    //                 triangulatedPoints[i].push_back(finalCombinations[i].point);
    //             }
    //         }else{
    //             // Classify every new point to path
    //             for(int i=0; i<finalCombinations.size(); i++){
    //                 triangulatedPoints[finalCombinations[i].closestPath].push_back(finalCombinations[i].point);
    //             }
    //         }
    //     }
    // }
};
