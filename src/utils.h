#pragma once
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Camera.h"
#include "DroneClassifier.h"
#include "pugixml.hpp"

using recursive_directory_iterator =
    std::filesystem::recursive_directory_iterator;

typedef std::vector<cv::Point3d> Path;

/**
 * @brief Loads 2D pixel data from CSV file
 * @param path Path to file
 * @param points Output argument, 2D Vector of points. First dim is for cameras
 * and second dim is for number of points.
 * @param offset How many lines should we skip at the beggining.
 */
void loadPointsOneDrone(const char *path,
                        std::vector<std::vector<cv::Point2d>> &points,
                        int offset = 1);

std::vector<const tdr::Camera *> loadCamerasXML(const char *path);

const tdr::Camera *createCamera(int id, size_t width, size_t height,
                                double focalLength, cv::Mat translation,
                                cv::Mat rotation);

void writeOutputFile(const char *path,
                     const std::vector<cv::Point3d> &triangulatedPoints);

double calculateError(Path &labelPath, Path &predPath);

double calculateMedian(Path &labelPath, Path &predPath);

double calculateStd(Path &labelPath, Path &predPath, double mean);

/** Funkcja oblicza globalne współrzędne punku podanego we współrzenych drona
 * @param[in] cross wektor 5 punktów ramion krzyża w postaci [1. punkt ramienia
 * x,2. punkt ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt
 * przecięcia]
 * @param[in] punkt w lokalnym układnie współrzędnych związanych z ramionami x i
 * y
 * @return punkt w globalnym układnie współrzędnych
 */
cv::Point3d convert2global(std::vector<cv::Point3d> cross,
                           cv::Point3d localPoint);

/** Funkcja oblicza punkt przecięcia dla ramion krzyża w 3d z uwzględnieniem, że
 * jeden z 4 punktów może nie być współpaszczyznowy z pozostałymi  trzema
 * @param[in] four_points wektor 4 punktów ramion krzyża w postaci niekoniecznie
 * uporządkowanej
 * @param[in] arms wektor dwóch wartości stanowiących długości ramion krzyża
 * @param[in] err maksymalny dopuszczalny błąd między faktyczną długością
 * ramienia, a długością obliczoną na podstawie punktów (w jednostkach tych
 * samych co pozycje markerów ramion)
 * @return wektor 5 punktów krzyża w postaci [1. punkt ramienia x,2. punkt
 * ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt przecięcia]
 */
std::vector<cv::Point3d> cross3d(
    std::vector<cv::Point3d> four_points,
    std::vector<double> arms = std::vector<double>{9, 6}, float err = 1000);

void readInputCSV(const char *dir, std::vector<Path> &path, int frequency,
                  int startFrame = 0, int endFrame = 0);