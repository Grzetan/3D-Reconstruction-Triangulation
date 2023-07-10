#include "DetectionsContainer.h"

DetectionsContainer::DetectionsContainer(const char* path, int offset, int recordSize, int startFrame, int endFrame){
    std::vector<std::string> files = getFiles(path);
    readFiles(files, offset, recordSize, startFrame, endFrame);
}

void DetectionsContainer::readFiles(const std::vector<std::string>& files, int offset, int recordSize, int startFrame, int endFrame){
    std::string line, token;

    for(auto& f : files){
        int n_line=0, frame=-1;
        std::ifstream file(f);
        data.push_back({}); // Add new vector for current camera

        while (std::getline(file, line)){
            if(offset > n_line++) continue; // Skip first `offset` lines
            std::istringstream iss(line);
            std::vector<int> seperatedLine;

            while(std::getline(iss, token, ',')) {
                seperatedLine.push_back(std::stoi(token));
            }

            if((seperatedLine[0] <= startFrame || seperatedLine[0] > endFrame) && startFrame != endFrame){
                frame = seperatedLine[0];
                continue;
            };
            
            for(int i=0; i<seperatedLine[0]-frame-1; i++){
                data.back().push_back({});
            }
            frame = seperatedLine[0];

            if((seperatedLine.size() - 1) % recordSize != 0)
                throw std::runtime_error("Invalid CSV file!");

            data.back().push_back({}); // Add vector for new frame

            for(int j=0; j<seperatedLine.size() / recordSize; j++){
                cv::Point2d point;
                point.x = seperatedLine[j*recordSize+5]; 
                point.y = seperatedLine[j*recordSize+6];
                data.back().back().push_back(point);
            }
        }
    }

    if(data.size() < 2) throw std::runtime_error("There must be at least 2 cameras");

    for(int i=1; i<data.size(); i++){
        if(data[i-1].size() != data[i].size())
            throw std::runtime_error("Number of frames on all cameras must be the same");
    }

    n_frames = data[0].size();
    n_cameras = data.size();
}

std::vector<std::string> DetectionsContainer::getFiles(const char* path){
    // Get CSV files in order
    std::vector<std::string> files;
    for (const auto& dirEntry : recursive_directory_iterator(path)){
        std::string directory = dirEntry.path().u8string();
        if(directory.find(".csv") != std::string::npos && directory.find(".avi") == std::string::npos){
            files.push_back(directory);
        }
    }

    std::sort(files.begin(), files.end());

    return files;
}

std::vector<Frames> DetectionsContainer::getFrame(int i) const{
    std::vector<Frames> frame;
    for(int j=0; j<data.size(); j++){
        frame.push_back(data[j][i]);
    }

    return frame;
}

int DetectionsContainer::getFrameCount() const{
    return n_frames;
}

std::vector<int> DetectionsContainer::getDetectionsCount(int frame) const{
    std::vector<int> n_detections(n_cameras);
    for(int i=0; i<n_cameras; i++){
        n_detections[i] = data[i][frame].size() + 1; // Plus one for no detection
    }

    return n_detections;
}

cv::Point2d DetectionsContainer::getRecord(int camera, int frame, int detection) const{
    return data[camera][frame][detection];
}