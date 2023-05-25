#include "DroneClassifier.h"

bool DroneClassifier::Combination::operator<(const Combination& c) const {
    int count1 = std::count(combination_.begin(), combination_.end(), 0);
    int count2 = std::count(c.combination_.begin(), c.combination_.end(), 0);
    if (count1 != count2) {
        return count1 > count2;
    }

    return error > c.error;
}

bool DroneClassifier::Combination::operator>(const Combination& c) const {
    int count1 = std::count(combination_.begin(), combination_.end(), 0);
    int count2 = std::count(c.combination_.begin(), c.combination_.end(), 0);
    if (count1 != count2) {
        return count1 < count2;
    }

    return error < c.error;
}

bool DroneClassifier::Combination::isCombinationUnique(std::vector<Combination>& combinations) {
    for (const auto& comb : combinations) {
        for (int i = 0; i < combination_.size(); i++) {
            if (combination_[i] == comb.combination_[i] && combination_[i] != 0) return false;
        }
    }
    return true;
}

bool DroneClassifier::Combination::increment(std::vector<size_t>& combination, std::vector<size_t>& sizes){
    for(int i=combination.size() - 1; i>=0; i--){
        if (combination[i] < sizes[i] - 1) {
            combination[i]++;
            return true;
        } else {
            combination[i] = 0;
        }
    }
    return false;
}

bool DroneClassifier::CombinationPath::operator > (const CombinationPath& elem) const{
    return (error < elem.error);
}

void DroneClassifier::classifyDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones){
    // Add new vector for every drone
    for(int i=0; i<n_drones; i++){
        triangulatedPoints.push_back({});
    }
    
    for(int frame=0; frame<points[0].size(); frame++){
        std::cout << frame << std::endl;
        std::vector<size_t> combination(points.size());
        std::vector<size_t> n_detections(points.size());
        for(int i=0; i<points.size(); i++){
            n_detections[i] = points[i][frame].size() + 1; // Plus one for no detection
        }

        std::priority_queue<Combination> combinationsQueue;

        do{
            // Skip combination if number of cameras is not enough
            int count = std::count_if(combination.begin(), combination.end(), [&](size_t &i) {
                return i > 0;
            });
            if(count < MIN_CAMERAS) continue;

            // Create rays for every bounding box in this combination
            std::vector<Triangulator::CamPointPair> images;
            for(int i=0; i<combination.size(); i++){
                if(combination[i] == 0) continue; // First index means no detection so we can skip it
                points[i][frame][combination[i]-1];
                images.push_back({triangulator_->getCameras()[i], points[i][frame][combination[i]-1]});
            }

            std::pair<cv::Point3d, double> pointWithError = triangulator_->triangulatePoint(images);
            combinationsQueue.push({combination, pointWithError.first, pointWithError.second});
        }while(Combination::increment(combination, n_detections));

        // Pick n_drones best combinations
        std::vector<Combination> finalCombinations;
        while(!combinationsQueue.empty() && finalCombinations.size() < n_drones){
            Combination c = combinationsQueue.top();
            if(c.isCombinationUnique(finalCombinations) && c.error < MAX_ERROR) finalCombinations.push_back(c);
            combinationsQueue.pop();
        }

        // Pick best drone path for each new point
        if(frame == 0){
            for(int i=0; i<finalCombinations.size(); i++){
                triangulatedPoints[i].push_back(finalCombinations[i].point);
            }
        }else{
            // Classify every new point to path
            std::vector<CombinationPath> combinationPathVec; // Tuple -> combination, path, error
            for(int i=0; i<finalCombinations.size(); i++){
                size_t bestPath = 0;
                double bestDist = -1;
                for(int j=0; j<n_drones; j++){
                    // Calculate average error from tail of each path
                    int nPointsToCheck = std::min(triangulatedPoints[j].size(), (size_t)PATH_TAIL);
                    if(nPointsToCheck == 0) continue;

                    double dist = 0;
                    for(int tail = triangulatedPoints[j].size() - nPointsToCheck; tail<triangulatedPoints[j].size(); tail++){
                        dist += cv::norm(triangulatedPoints[j][tail] - finalCombinations[i].point);
                    }
                    dist /= (double) nPointsToCheck;

                    if(dist < bestDist || bestDist == -1){
                        bestDist = dist;
                        bestPath = j;
                    }
                }

                combinationPathVec.push_back({(size_t)i, bestPath, bestDist});
            }
            
            std::sort(combinationPathVec.begin(), combinationPathVec.end(), greater<CombinationPath>());

            std::vector<size_t> usedPaths;
            for(const auto& c : combinationPathVec){
                // If best path is taken or closest path is a path that doesn't exists
                if(std::find(usedPaths.begin(), usedPaths.end(), c.path) != usedPaths.end()){
                    int emptyPath = -1;
                    for(int i=0; i < triangulatedPoints.size(); i++){
                        if(triangulatedPoints[i].empty()){
                            emptyPath = i;
                            break;
                        }
                    }
                    if(emptyPath != -1){
                        triangulatedPoints[emptyPath].push_back(finalCombinations[c.combination].point);
                    }
                    continue;
                };
                triangulatedPoints[c.path].push_back(finalCombinations[c.combination].point);
                usedPaths.push_back(c.path);
            }
        }
    }
}

