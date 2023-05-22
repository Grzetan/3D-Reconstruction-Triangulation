#pragma once
#include <vector>
#include<opencv2/opencv.hpp>
class DronClassifier
{
	public:
		struct Combination {
            std::vector<size_t> combination_;
            cv::Point3d point;
            double error;

            bool operator<(const Combination& c) const {
                int count1 = std::count(combination_.begin(), combination_.end(), 0);
                int count2 = std::count(c.combination_.begin(), c.combination_.end(), 0);
                if (count1 != count2) {
                    return count1 > count2;
                }

                return error > c.error;
            }

            bool operator>(const Combination& c) const {
                int count1 = std::count(combination_.begin(), combination_.end(), 0);
                int count2 = std::count(c.combination_.begin(), c.combination_.end(), 0);
                if (count1 != count2) {
                    return count1 < count2;
                }

                return error < c.error;
            }

            bool isCombinationUnique(std::vector<Combination>& combinations) {
                for (const auto& comb : combinations) {
                    for (int i = 0; i < combination_.size(); i++) {
                        if (combination_[i] == comb.combination_[i] && combination_[i] != 0) return false;
                    }
                }
                return true;
            }
            Combination& operator++();
            //Combination operator++(int);
	    };
	void triangulateMultipleDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones);
};