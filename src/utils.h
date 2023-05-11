#pragma once
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "pugixml.hpp"
#include "Camera.h"

using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

void loadPointsOneDrone(const char* path, std::vector<std::vector<cv::Point2d>>& points, int offset = 1);

std::vector<const tdr::Camera*> loadCamerasXML(const char* path);

const tdr::Camera* createCamera(int id, size_t width, size_t height, double focalLength, cv::Mat translation, cv::Mat rotation);

void writeOutputFile(const char* path, const std::vector<cv::Point3d>& triangulatedPoints);
