#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "argparse.hpp"
#include "happly.h"
#include "utils.h"

std::string OUTPUT_DIR = "./labels/";

int main(int argc, const char **argv) {
  argparse::ArgumentParser args("3D-Reconstruction-Triangulation");

  args.add_argument("pred_path")
      .help("Path to the folder with PLY predictions")
      .required();

  args.add_argument("label_path")
      .help("Path to the CSV file from Vicon")
      .required();

  args.add_argument("--frequency")
      .help("How many lines in label file correspond to one frame")
      .scan<'i', int>()
      .default_value(1);

  args.add_argument("--output")
      .help("Name of the optional output file")
      .default_value<std::string>("None");

  try {
    args.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << args;
    std::exit(1);
  }

  if (std::filesystem::exists(OUTPUT_DIR)) {
    std::filesystem::remove_all(OUTPUT_DIR);
  }

  std::filesystem::create_directory(OUTPUT_DIR);

  // Read label paths from vicon system
  std::vector<Path> labelPaths;
  readInputCSV(args.get("label_path").c_str(), labelPaths,
               args.get<int>("--frequency"));

  // Write label paths to PLY for visualization
  for (int i = 0; i < labelPaths.size(); i++) {
    std::string name = OUTPUT_DIR + "label_" + std::to_string(i + 1) + ".ply";
    writeOutputFile(name.c_str(), labelPaths[i]);
  }

  for (const auto &entry :
       std::filesystem::directory_iterator(args.get("pred_path"))) {
    happly::PLYData inputPLY(entry.path());
    auto predPathHapply = inputPLY.getVertexPositions();
    Path predPath;
    for (const auto &p : predPathHapply) {
      predPath.push_back({p[0], p[1], p[2]});
    }

    double minAvg = 1e+8;
    double minMedian = 1e+8;
    double minStd = 1e+8;
    double minQStd = 1e+8;
    std::string labelPath = "";
    for (int i = 0; i < labelPaths.size(); i++) {
      double error = calculateError(labelPaths[i], predPath);
      double median = calculateMedian(labelPaths[i], predPath);
      double std = calculateStd(labelPaths[i], predPath, error);
      double qstd = calcualteQuarterDeviation(labelPaths[i], predPath);
      if (error < minAvg) {
        minAvg = error;
        minMedian = median;
        minStd = std;
        minQStd = qstd;
        labelPath = OUTPUT_DIR + "label_" + std::to_string(i + 1) + ".ply";
      }
    }

    std::cout << entry.path() << " with " << labelPath
              << ": average error: " << minAvg << " median error: " << minMedian
              << " std: " << minStd << " qstd: " << minQStd << std::endl;
  }
}