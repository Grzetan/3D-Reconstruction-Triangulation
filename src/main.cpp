#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "argparse.hpp"
#include "RayTriangulator.h"
#include "MatrixTriangulator.h"
#include "DroneClassifier.h"
#include "utils.h"

int main(int argc, const char** argv){
    // argparse::ArgumentParser args("3D-Reconstruction-Triangulation"); 

    // args.add_argument("cameras_path")
    // .help("Path to the file with camera data")
    // .required();

    // args.add_argument("data_path")
    // .help("Path to the folder woth detections on each camera")
    // .required();

    // args.add_argument("--n_drones")
    // .help("Number of drones in the scene")
    // .scan<'i', int>()
    // .default_value(1);

    // args.add_argument("--triangulator")
    // .help("Which triangulator should be used")
    // .default_value<std::string>("matrix");

    // try {
    //     args.parse_args(argc, argv);
    // }
    // catch (const std::runtime_error& err) {
    //     std::cerr << err.what() << std::endl;
    //     std::cerr << args;
    //     std::exit(1);
    // }

    // std::vector<const tdr::Camera*> cameras = loadCamerasXML(args.get("cameras_path").c_str());

    // int n_drones = args.get<int>("n_drones");

    // if(n_drones > 1){
    //     Triangulator* triangulator;
    //     if(args.get<std::string>("--triangulator") == "matrix"){
    //         triangulator = new MatrixTriangulator(cameras);
    //     }else if(args.get<std::string>("--triangulator") == "ray"){
    //         triangulator = new RayTriangulator(cameras);
    //     }else{
    //         throw std::runtime_error("Invalid --triangulator argument. Allowed options are \'matrix\' and \'ray\'");
    //     }

    //     DroneClassifier classifier(triangulator);

    //     std::vector<std::vector<std::vector<cv::Point2d>>> dronePoints;
    //     loadPointsMultipleDrones(args.get("data_path").c_str(), dronePoints, 1, 7);

    //     auto start = std::chrono::high_resolution_clock::now();

    //     std::vector<std::vector<cv::Point3d>> triangulatedPoints;
    //     classifier.classifyDrones(dronePoints, triangulatedPoints, n_drones);

    //     auto stop = std::chrono::high_resolution_clock::now();
    
    //     auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //     std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

    //     for(int i=1; i<=n_drones; i++){
    //         std::string name = "drone" + std::to_string(i) + ".ply";
    //         writeOutputFile(name.c_str(), triangulatedPoints[i-1]);
    //     }

    //     delete triangulator;
    // }else if(n_drones == 1){
    //     Triangulator* triangulator;
    //     if(args.get("triangulator") == "matrix"){
    //         triangulator = new MatrixTriangulator(cameras);
    //     }else if(args.get("triangulator") == "ray"){
    //         triangulator = new RayTriangulator(cameras);
    //     }else{
    //         throw std::runtime_error("Invalid --triangulator argument. Allowed options are \'matrix\' and \'ray\'");
    //     }

    //     std::vector<std::vector<cv::Point2d>> dronePoints;
    //     loadPointsOneDrone(args.get("data_path").c_str(), dronePoints);

    //     auto start = std::chrono::high_resolution_clock::now();

    //     std::vector<cv::Point3d> triangulatedPoints = triangulator->triangulatePoints(dronePoints);

    //     auto stop = std::chrono::high_resolution_clock::now();

    //     auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //     std::cout << "Execution time: " << time.count() * 1e-6 << "s" << std::endl;

    //     writeOutputFile("dron.ply", triangulatedPoints);

    //     delete triangulator;
    // }

    // // Free memory
    // for(const auto& cam : cameras)
    //     delete cam;

    std::vector<cv::Point3d> points;

    // Quat to calculate pos: w,-y,-z,-x from CSV
    // Quat to visualize rotation: w,y,z,x from CSV


    // https://www.omnicalculator.com/math/quaternion = 0.707107, -0.707107, 0, 0 * quaternion 

    tdr::Camera cam1(1);
    // cam1.rquat = (cv::Mat_<double>(4, 1) << 0.913008391857147, -0.1603030264377594, 0.3691321015357971, -0.0667838528752327);
    cam1.rvec = (cv::Mat_<double>(3,1) << -1.8030127723051588, -0.7605790719287683, -0.5374602997814084);
    cam1.tvec = (cv::Mat_<double>(3, 1) << -2.4211251541298466, 3.9287887957504046, 10.254812764947367);
    cam1.compCamPos();
    points.push_back({cam1.camPos.at<double>(0, 0), cam1.camPos.at<double>(0, 1), cam1.camPos.at<double>(0, 2)});

    tdr::Camera cam2(2);
    // cam2.rquat = (cv::Mat_<double>(4, 1) << 0.9098438620567322, -0.16042986512184143, -0.37686944007873535, 0.06645218282938004);
    cam2.rvec = (cv::Mat_<double>(3,1) << -0.8660440468144118, -2.094135985389056, -1.4673246615497073);
    cam2.tvec = (cv::Mat_<double>(3, 1) << 2.157460405969535, 3.9092256350612558, 10.227343803772223);
    cam2.compCamPos();
    points.push_back({cam2.camPos.at<double>(0, 0), cam2.camPos.at<double>(0, 1), cam2.camPos.at<double>(0, 2)});
    
    tdr::Camera cam3(3);
    // cam3.rquat = (cv::Mat_<double>(4, 1) << 0.3768696188926697, -0.06645228713750839, 0.9098437428474426, -0.16042999923229218);
    cam3.rvec = (cv::Mat_<double>(3,1) << -1.806610350691817, 0.7472598228142185, 0.523158523665988);
    cam3.tvec = (cv::Mat_<double>(3, 1) << -0.27561110998351457, 3.1928205678003097, 8.263012010406923);
    cam3.compCamPos();
    points.push_back({cam3.camPos.at<double>(0, 0), cam3.camPos.at<double>(0, 1), cam3.camPos.at<double>(0, 2)});

    tdr::Camera cam4(4);
    // cam4.rquat = (cv::Mat_<double>(4, 1) << 0.3768696188926697, -0.06645228713750839, -0.9098437428474426, 0.16042999923229218);
    cam4.rvec = (cv::Mat_<double>(3,1) << -0.8659750700278613, 2.0940255780976638, 1.4674326363599184);
    cam4.tvec = (cv::Mat_<double>(3, 1) << -2.174767468200178, 3.8122882802506757, 9.956303419436846);
    cam4.compCamPos();
    points.push_back({cam4.camPos.at<double>(0, 0), cam4.camPos.at<double>(0, 1), cam4.camPos.at<double>(0, 2)});

    tdr::Camera cam5(5);
    // cam5.rquat = (cv::Mat_<double>(4, 1) << 1.7609735891710443e-07, -3.105067492015223e-08, 0.9848077297210693, -0.1736479550600052);
    cam5.rvec = (cv::Mat_<double>(3,1) << -1.4630369426947496, 1.4622020392989297, 1.0219376860740386);
    cam5.tvec = (cv::Mat_<double>(3, 1) << 4.214448939995925e-05, 2.4016679874206965, 6.140393526172339);
    cam5.compCamPos();
    points.push_back({cam5.camPos.at<double>(0, 0), cam5.camPos.at<double>(0, 1), cam5.camPos.at<double>(0, 2)});

    tdr::Camera cam6(6);
    // cam6.rquat = (cv::Mat_<double>(4, 1) << 0.9848077893257141, -0.17364801466464996, -0.0, 0.0);
    cam6.rvec = (cv::Mat_<double>(3,1) << -1.4611732308714729, -1.4616228965778795, -1.0236873574123597);
    cam6.tvec = (cv::Mat_<double>(3, 1) << -0.007164718155340565, 2.4232925966047834, 6.150333550064927);
    cam6.compCamPos();
    points.push_back({cam6.camPos.at<double>(0, 0), cam6.camPos.at<double>(0, 1), cam6.camPos.at<double>(0, 2)});

    tdr::Camera cam7(7);
    // cam7.rquat = (cv::Mat_<double>(4, 1) << 0.6963645219802856, -0.12278773635625839, -0.6963640451431274, 0.12278764694929123);
    cam7.rvec = (cv::Mat_<double>(3,1) << 4.005582029757298e-05, -2.573320556829324, -1.802144194275057);
    cam7.tvec = (cv::Mat_<double>(3, 1) << -0.007981329846330943, 2.884304261438658, 7.426958437181165);
    cam7.compCamPos();
    points.push_back({cam7.camPos.at<double>(0, 0), cam7.camPos.at<double>(0, 1), cam7.camPos.at<double>(0, 2)});

    tdr::Camera cam8(8);
    // cam8.rquat = (cv::Mat_<double>(4, 1) << 0.6963642239570618, -0.12278764694929123, 0.6963642835617065, -0.12278765439987183);
    cam8.rvec = (cv::Mat_<double>(3,1) << -1.9195098750605122, -0.0006901907402259471, -0.000654196915076301);
    cam8.tvec = (cv::Mat_<double>(3, 1) << 0.0015480959721941771, 3.501003907840777, 9.122243560621474);
    cam8.compCamPos();
    points.push_back({cam8.camPos.at<double>(0, 0), cam8.camPos.at<double>(0, 1), cam8.camPos.at<double>(0, 2)});

    writeOutputFile("test.ply", points);
}