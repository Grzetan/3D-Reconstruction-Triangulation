#pragma once
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "pugixml.hpp"
#include "Camera.h"

using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

/**
 * @brief Loads 2D pixel data from CSV file
 * @param path Path to file
 * @param points Output argument, 2D Vector of points. First dim is for cameras and second dim is for number of points.
 * @param offset How many lines should we skip at the beggining.
 */
void loadPointsOneDrone(const char* path, std::vector<std::vector<cv::Point2d>>& points, int offset = 1);

/**
 * @brief Loads 2D pixel data from CSV file
 * @param path Path to file
 * @param points Output argument, 3D Vector of points. First dim is for cameras, second is for number of frames and third dimention is for drones.
 * @param offset How many lines should we skip at the beggining.
 * @param recordSize How many numbers each detection has
 */
void loadPointsMultipleDrones(const char* path, std::vector<std::vector<std::vector<cv::Point2d>>>& points, int offset = 1, int recordSize = 7);

std::vector<const tdr::Camera*> loadCamerasXML(const char* path);

const tdr::Camera* createCamera(int id, size_t width, size_t height, double focalLength, cv::Mat translation, cv::Mat rotation);

void writeOutputFile(const char* path, const std::vector<cv::Point3d>& triangulatedPoints, bool addFaces=true);
