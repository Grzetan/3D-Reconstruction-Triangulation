#include "DroneClassifier.h"

DroneClassifier::DroneClassifier(Triangulator* triangulator, size_t n_drones) : triangulator_(triangulator), n_drones_(n_drones){
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

void DroneClassifier::classifyDrones(const DetectionsContainer& container, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones){
    // Add new vector for every drone
    for(int i=0; i<n_drones; i++){
        triangulatedPoints.push_back({});
    }

    int n_frames = container.getFrameCount();

    for(int frame=0; frame<n_frames; frame++){
        std::cout << frame << " / " << n_frames << std::endl;

        std::vector<int> processedPaths;
        std::vector<Combination> usedCombinations;

        // First process all paths with some existing points
        for(int n_path=0; n_path<triangulatedPoints.size(); n_path++){
            const std::vector<cv::Point3d>& currPath = triangulatedPoints[n_path];
            // TODO We maybe can get last valid pos from path to speed up algo
            if(currPath.size() == 0 || true) continue;

            if(currPath.size() != 0 && currPath.back() != cv::Point3d(0,0,0)){
                processedPaths.push_back(n_path);
                Combination bestPointForPath = triangulateWithLastPos(currPath.back(), container, usedCombinations, frame);
                if(bestPointForPath.combination_.size() > 0) // If combination was found, add it to used combinations
                    usedCombinations.push_back(bestPointForPath);
                triangulatedPoints[n_path].push_back(bestPointForPath.point); // Add point to path
            }
        }

        if(processedPaths.size() == triangulatedPoints.size()) continue;

        std::cout << "OLD ALGO" << std::endl;
        // Create container with not used detections
        DetectionsContainer tmpContainer(container.getCamCount());
        tmpContainer.addEmptyFrame();

        for(int cam=0; cam<container.getCamCount(); cam++){
            for(int det=0; det<container.detCountForCam(cam, frame); det++){
                if(!isDetectionInCombinations(det, cam, usedCombinations)){
                    tmpContainer.addDetectionToCamera(container.getRecord(cam, frame, det), cam);
                }
            }
        }

        std::vector<Combination> finalCombinations = pickBestCombinations(tmpContainer, 0, triangulatedPoints.size() - processedPaths.size());

        // Classify every new point to path
        std::vector<CombinationPath> combinationPathVec;
        for(int i=0; i<finalCombinations.size(); i++){
            size_t bestPath = 0;
            double bestDist = -1;
            for(int j=0; j<n_drones; j++){
                if(std::find(processedPaths.begin(), processedPaths.end(), j) != processedPaths.end()) continue;

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

        for(const auto& c : combinationPathVec){
            // If best path is taken or closest path is a path that doesn't exists
            if(std::find(processedPaths.begin(), processedPaths.end(), c.path) != processedPaths.end()){
                int emptyPath = -1;
                for(int i=0; i < triangulatedPoints.size(); i++){
                    if(triangulatedPoints[i].empty()){
                        emptyPath = i;
                        break;
                    }
                }
                if(emptyPath != -1){
                    triangulatedPoints[emptyPath].push_back(finalCombinations[c.combination].point);
                    processedPaths.push_back(emptyPath);
                }
            }else{
                triangulatedPoints[c.path].push_back(finalCombinations[c.combination].point);
                processedPaths.push_back(c.path);
            }
        }

        // Add point [0,0,0] to unused paths to keep frame count
        for(int i=0; i<n_drones; i++){
            if(std::find(processedPaths.begin(), processedPaths.end(), i) == processedPaths.end()){
                triangulatedPoints[i].push_back(cv::Point3d(0,0,0));
            }
        }
    }
}

void DroneClassifier::fillCombinationQueue(const DetectionsContainer& container, int frame, std::priority_queue<Combination>& pq){
    std::vector<int> n_detections = container.getDetectionsCount(frame);
    Iterator iterator(n_detections);

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
            images.push_back({triangulator_->getCameras()[i], container.getRecord(i, frame, combination[i]-1)});
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
            pq.push({combination, pointWithError.first, pointWithError.second});
        }
    }
}


std::vector<DroneClassifier::Combination> DroneClassifier::pickBestCombinations(const DetectionsContainer& container, int frame, int detectionsNeeded){
    std::priority_queue<Combination> combinationsQueue;
    fillCombinationQueue(container, frame, combinationsQueue);

    // Pick n_drones best combinations
    std::vector<Combination> finalCombinations;
    while(!combinationsQueue.empty() && finalCombinations.size() < detectionsNeeded){
        Combination c = combinationsQueue.top();
        if(c.isCombinationUnique(finalCombinations)) finalCombinations.push_back(c);
        combinationsQueue.pop();
    }

    return finalCombinations;
}

DroneClassifier::Combination DroneClassifier::triangulateWithLastPos(cv::Point3d pos, const DetectionsContainer& container, std::vector<Combination> usedCombinations, int frame){
    // Create empty container that will later have only one frame with valid detections
    DetectionsContainer tmpContainer(container.getCamCount());
    tmpContainer.addEmptyFrame();

    // Get detections that are relativly close to the last pos in path
    for(int cam=0; cam<container.getCamCount(); cam++){
        for(int det=0; det<container.detCountForCam(cam, frame); det++){
            cv::Point2d record = container.getRecord(cam, frame, det);
            if(Triangulator::getDistFromRay({triangulator_->getCamera(cam), record}, pos) < 20){
                tmpContainer.addDetectionToCamera(record, cam);
            }
        }
    }

    std::cout << "HALO: ";
    for(const auto& i : tmpContainer.getDetectionsCount(0)){
        std::cout << i << ", ";
    }
    std::cout << std::endl;

    std::priority_queue<Combination> combinationsQueue;
    fillCombinationQueue(tmpContainer, 0, combinationsQueue);

    while(!combinationsQueue.empty()){
        Combination c = combinationsQueue.top();
        if(c.isCombinationUnique(usedCombinations) && c.error < error_) return c;
        combinationsQueue.pop();
    }
    
    return {{}, {0,0,0}};
}

bool DroneClassifier::isDetectionInCombinations(int cam, int det, const std::vector<Combination>& combinations){
    for(const auto& c : combinations){
        for(int cam=0; cam<c.combination_.size(); cam++){
            if(cam==cam && c.combination_[cam]==det) return true;
        }
    }
    return false;
}


