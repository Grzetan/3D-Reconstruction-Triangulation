#include "MatrixTriangulator.h"

std::pair<cv::Point3d, double> MatrixTriangulator::triangulatePoint(vector<CamPointPair> images){


    cv::Mat cooficients;// A matrix in equation AX=B
    cv::Mat bias;// B matrix in equation AX=B
    cooficients = cv::Mat(2 * images.size(), 3, CV_64FC1);
    bias = cv::Mat(2 * images.size(), 1, CV_64FC1);

    for (int n_cam_iterator = 0; n_cam_iterator < images.size(); n_cam_iterator++) {
        if ((images[n_cam_iterator].point.x == -1 || images[n_cam_iterator].point.y == -1)) continue;


        cooficients.at<double>(0 + n_cam_iterator * 2, 0) = images[n_cam_iterator].projectionMatrix().at<double>(0, 0) - images[n_cam_iterator].point.x * images[n_cam_iterator].projectionMatrix().at<double>(2, 0);
        cooficients.at<double>(0 + n_cam_iterator * 2, 1) = images[n_cam_iterator].projectionMatrix().at<double>(0, 1) - images[n_cam_iterator].point.x * images[n_cam_iterator].projectionMatrix().at<double>(2, 1);
        cooficients.at<double>(0 + n_cam_iterator * 2, 2) = images[n_cam_iterator].projectionMatrix().at<double>(0, 2) - images[n_cam_iterator].point.x * images[n_cam_iterator].projectionMatrix().at<double>(2, 2);


        cooficients.at<double>(1 + n_cam_iterator * 2, 0) = images[n_cam_iterator].projectionMatrix().at<double>(1, 0) - images[n_cam_iterator].point.y * images[n_cam_iterator].projectionMatrix().at<double>(2, 0);
        cooficients.at<double>(1 + n_cam_iterator * 2, 1) = images[n_cam_iterator].projectionMatrix().at<double>(1, 1) - images[n_cam_iterator].point.y * images[n_cam_iterator].projectionMatrix().at<double>(2, 1);
        cooficients.at<double>(1 + n_cam_iterator * 2, 2) = images[n_cam_iterator].projectionMatrix().at<double>(1, 2) - images[n_cam_iterator].point.y * images[n_cam_iterator].projectionMatrix().at<double>(2, 2);

        bias.at<double>(0 + n_cam_iterator * 2, 0) = images[n_cam_iterator].point.x * images[n_cam_iterator].projectionMatrix().at<double>(2, 3) - images[n_cam_iterator].projectionMatrix().at<double>(0, 3);
        bias.at<double>(1 + n_cam_iterator * 2, 0) = images[n_cam_iterator].point.y * images[n_cam_iterator].projectionMatrix().at<double>(2, 3) - images[n_cam_iterator].projectionMatrix().at<double>(1, 3);

    }

    cv::Mat inv_L;
    cv::invert(cooficients, inv_L, cv::DecompTypes::DECOMP_SVD);
    cv::Mat X = inv_L * bias;
    cv::Mat error = (cooficients * X - bias);
    cv::Mat errorT;
    cv::transpose(error, errorT);
    error = (errorT * error);
    double err = error.at<double>(0);
    return std::pair<cv::Point3d, double> {cv::Point3d(X.at<double>(0), X.at<double>(1), X.at<double>(2)),err};

}


MatrixTriangulator::MatrixTriangulator(std::vector<const tdr::Camera*> cameras_){
    cameras = cameras_;
}


std::vector<cv::Point3d> MatrixTriangulator::triangulatePoints(std::vector<std::vector<cv::Point2d>> points){
    for (int i = 0; i < points.size() - 1; i++) {
        if (points[i].size() != points[i + 1].size())
            throw std::runtime_error("Every camera should have the same number of points");
    }

    std::vector<cv::Point3d> result;

    for (int n_point = 0; n_point < points[0].size(); n_point++) { // For every point
        int numberOfCameras = 0;
        std::vector<CamPointPair> frame{};
        for (int n_cam = 0; n_cam < min(points.size(), this->cameras.size()); n_cam++) {
            if (!(points[n_cam][n_point].x == -1 || points[n_cam][n_point].y == -1)) numberOfCameras += 1;
            frame.push_back(CamPointPair{ this->cameras[n_cam],points[n_cam][n_point] });
        }


        if (frame.size() < 2) {
            throw std::runtime_error("Too few rays are found");
        }

        result.push_back(triangulatePoint(frame).first);
    }

    return result;
}