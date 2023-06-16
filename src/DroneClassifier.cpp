#include "DroneClassifier.h"

DroneClassifier::DroneClassifier(Triangulator* triangulator) : triangulator_(triangulator){
    if(triangulator->getType() == "matrix"){
        error_ = MAX_ERROR_MATRIX;
    }else if(triangulator->getType() == "ray"){
        error_ = MAX_ERROR_RAY;
    }
}

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

DroneClassifier::Iterator::Iterator(std::vector<int> sizes): sizes_(sizes){
    combination_ = std::vector<int>(sizes.size(), -1);
}

bool DroneClassifier::Iterator::increment(){
    // Check if combination was cut
    if(skipNext){
        skipNext = false;
        return true;
    }

    // Increment first -1 bit
    for(int i=0; i<combination_.size(); i++){
        if (combination_[i] == -1) {
            combination_[i]++;
            return true;
        }
    }
    // If there are no -1 bits, increment last bit
    for(int i=combination_.size() - 1; i>=0; i--){
        if(combination_[i] < sizes_[i] - 1){
            combination_[i]++;
            return true;
        }else{
            combination_[i] = -1;
        }
    }

    return false;
}


bool DroneClassifier::Iterator::cut(){
    for(int i=combination_.size() - 1; i>=0; i--){
        if(combination_[i] != -1 && combination_[i] < sizes_[i] - 1){
            combination_[i]++;
            skipNext = true;
            return true;
        }else{
            combination_[i] = -1;
        }
    }
    return false;
}

std::vector<int> DroneClassifier::Iterator::getCombination(){
    return combination_;
}

bool DroneClassifier::CombinationPath::operator>(const CombinationPath& elem) const{
    return (error < elem.error);
}

void DroneClassifier::classifyDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones){
    std::vector<std::vector<int>> emptyFrames;

    // Add new vector for every drone
    for(int i=0; i<n_drones; i++){
        triangulatedPoints.push_back({});
        emptyFrames.push_back({});
    }
    
    for(int frame=0; frame<points[0].size(); frame++){
        std::cout << frame << " / " << points[0].size() << std::endl;
        std::vector<int> n_detections(points.size());
        for(int i=0; i<points.size(); i++){
            n_detections[i] = points[i][frame].size() + 1; // Plus one for no detection
        }

        Iterator iterator(n_detections);
        std::priority_queue<Combination> combinationsQueue;

        while(iterator.increment()){
            std::vector<int> combination = iterator.getCombination();

            // Skip combination if number of cameras is not enough
            int count = std::count_if(combination.begin(), combination.end(), [&](int &i) {
                return i > 0;
            });
            if(count < 2) continue;

            // Create rays for every bounding box in this combination
            std::vector<Triangulator::CamPointPair> images;
            for(int i=0; i<combination.size(); i++){
                if(combination[i] <= 0) continue; // Index 0 or -1 means no detection or camera isn't taken into account so we can skip it
                images.push_back({triangulator_->getCameras()[i], points[i][frame][combination[i]-1]});
            }

            std::pair<cv::Point3d, double> pointWithError = triangulator_->triangulatePoint(images);

            // If at some point combination's error is too big, cut the rest of combinations
            if(pointWithError.second > error_){
                if(!iterator.cut()) break;
            }
            // Add combination to queue only if all of the cameras are taken into account
            else if(std::find(combination.begin(), combination.end(), -1) == combination.end() && 
                    std::count_if(combination.begin(), combination.end(), [&](int &i) {
                        return i > 0;
                    }) >= MIN_CAMERAS){
                combinationsQueue.push({combination, pointWithError.first, pointWithError.second});
            }
        }

        // Pick n_drones best combinations
        std::vector<Combination> finalCombinations;
        while(!combinationsQueue.empty() && finalCombinations.size() < n_drones){
            Combination c = combinationsQueue.top();
            if(c.isCombinationUnique(finalCombinations)) finalCombinations.push_back(c);
            combinationsQueue.pop();
        }

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

        // Add point [0,0,0] to unused paths to keep frame count
        for(int i=0; i<n_drones; i++){
            if(std::find(usedPaths.begin(), usedPaths.end(), i) == usedPaths.end()){
                emptyFrames[i].push_back(frame);
            }
        }
    }

    // Add [0,0,0] to empty detections
    for(int i=0; i<emptyFrames.size(); i++){
        for(int j=0; j<emptyFrames[i].size(); j++){
            triangulatedPoints[i].insert(triangulatedPoints[i].begin() + emptyFrames[i][j], cv::Point3d(0,0,0));
        }
    }
}

