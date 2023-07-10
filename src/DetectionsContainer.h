#pragma once
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::Point2d> Detections;
typedef std::vector<Detections> Cameras;

using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

class DetectionsContainer{
public:
    int n_frames, n_cameras;

    std::vector<Cameras> data;

    std::vector<std::string> getFiles(const char* path);

    void readFiles(const std::vector<std::string>& files, int offset, int recordSize, int startFrame, int endFrame);

public:
    /**
     * @brief Loads 2D pixel data from CSV file
     * @param path Path to file
     * @param points Output argument, 3D Vector of points. First dim is for cameras, second is for number of frames and third dimention is for drones.
     * @param offset How many lines should we skip at the beggining.
     * @param recordSize How many numbers each detection has
     * @param startFrame From which frame should path be created
     * @param endFrame On which frame should path be ended
     */
    DetectionsContainer(const char* path, int offset, int recordSize, int startFrame=0, int endFrame=0);

    DetectionsContainer(int camCount);

    std::vector<Detections> getFrame(int i) const;

    int getFrameCount() const;

    int getCamCount() const;

    std::vector<int> getDetectionsCount(int frame) const;

    int detCountForCam(int cam, int frame) const;

    cv::Point2d getRecord(int camera, int frame, int detection) const;

    void addEmptyFrame();

    void addDetectionToCamera(cv::Point2d det, int cam);
};