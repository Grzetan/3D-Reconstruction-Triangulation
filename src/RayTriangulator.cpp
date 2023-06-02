# include "RayTriangulator.h"

RayTriangulator::RayTriangulator(std::vector<const tdr::Camera*> cameras_){
    cameras = cameras_;
    type_ = "ray";
}


bool RayTriangulator::RayClosestPoint::compute(cv::InputArray params, cv::OutputArray err, cv::OutputArray J) const{
    cv::Mat paramMat = params.getMat().reshape(1, 1);
    const double x = paramMat.at<double>(0, 0);
    const double y = paramMat.at<double>(0, 1);
    const double z = paramMat.at<double>(0, 2);
    const cv::Vec3d currentPoint(x, y, z);

    double summedError = 0;

    err.create(static_cast<int>(rays_.size()), 1, CV_64FC1);
    cv::Mat errMat = err.getMat();
    errMat.forEach<double>([&](double& e, const int* position) {
        e = distToRay(rays_[position[0]], currentPoint);
        summedError += e;
    });

    error = summedError / (double) rays_.size();

    if (J.needed())
    {
        J.create(errMat.rows, paramMat.cols, CV_64FC1);
        cv::Mat Jmat = J.getMat();
        for(int row = 0 ; row<Jmat.rows ; ++row){
            double* pJ = Jmat.ptr<double>(row);

            *pJ++ = (distToRay_(rays_[row], {x+epsilon_, y, z}, squared_) - distToRay_(rays_[row], {x-epsilon_, y, z}, squared_)) / (2*epsilon_);
            *pJ++ = (distToRay_(rays_[row], {x, y+epsilon_, z}, squared_) - distToRay_(rays_[row], {x, y-epsilon_, z}, squared_)) / (2*epsilon_);
            *pJ++ = (distToRay_(rays_[row], {x, y, z+epsilon_}, squared_) - distToRay_(rays_[row], {x, y, z-epsilon_}, squared_)) / (2*epsilon_);
        }
    }

    return true;
}

double RayTriangulator::RayClosestPoint::getError(){
    return error;
}

double RayTriangulator::RayClosestPoint::distToRay(const Ray& r, const cv::Vec3d& p, bool squared){
    cv::Point3d result = r.dir.cross(p - cv::Vec3d(r.origin));
    if(squared) return result.x*result.x + result.y*result.y + result.z*result.z;
    return std::sqrt(result.x*result.x + result.y*result.y + result.z*result.z);
}

double RayTriangulator::RayClosestPoint::distToRay_(const Ray& r, const cv::Vec3d p, bool squared){
    return distToRay(r, p, squared);
}

cv::Vec3d RayTriangulator::rotatePointByQuaternion(cv::Vec3d point, const cv::Mat quat){
    cv::Vec4d quaternion(quat.at<double>(0,0), quat.at<double>(1,0), quat.at<double>(2,0), quat.at<double>(3,0));
    cv::Vec4d point_quaternion(0, point[0], point[1], point[2]);
    cv::Vec4d rotated_point_quaternion = quaternion * point_quaternion * quaternion.conj();

    return {rotated_point_quaternion[1], rotated_point_quaternion[2], rotated_point_quaternion[3]};
}

cv::Vec3d RayTriangulator::calculateRayDirectionForPixel(const tdr::Camera* cam, const cv::Point2d& point){
    // Add 0.5 to get center of pixel
    cv::Point2d p(point.x, point.y);
    p.x += 0.5;
    p.y += 0.5; 

    double d = 1 / std::tan(cam->fovy*0.0174533 / 2);
    cv::Vec3d ray;
    // We have to adjust coordinate space so it fits identity quaternion
    double x = ((double)cam->width / (double)cam->height) * ((2 * p.x / (double)cam->width) - 1);
    double y = (2 * p.y / (double)cam->height) - 1;
    ray[0] = x;
    ray[1] = y;
    ray[2] = d;
    return cv::normalize(ray);
}

RayTriangulator::Ray RayTriangulator::createRayForPoint(const tdr::Camera* cam, const cv::Point2d& point){
    cv::Vec3d pixelDir = calculateRayDirectionForPixel(cam, point);

    cv::Point3d origin;
    origin.x = cam->tvec.at<double>(0);
    origin.y = cam->tvec.at<double>(1);
    origin.z = cam->tvec.at<double>(2);
    return {origin, rotatePointByQuaternion(pixelDir, cam->rquat)};
}

bool RayTriangulator::increment(std::vector<size_t>& combination, std::vector<size_t>& sizes){
    for(int i=combination.size() - 1; i>=0; i--){
        if (combination[i] < sizes[i] - 1) {
            combination[i]++;
            return true;
        } else {
            combination[i] = 0;
        }
    }
    return false;
}

std::vector<cv::Point3d> RayTriangulator::triangulatePoints(std::vector<std::vector<cv::Point2d>> points){
    // Check if dims are correct
    for(int i=0; i<points.size() - 1; i++){
        if(points[i].size() != points[i+1].size()) 
            throw std::runtime_error("Every camera should have the same number of points");
    }
    
    std::vector<cv::Point3d> result;

    for(int n_point=0; n_point<points[0].size(); n_point++){ // For every point
        std::vector<CamPointPair> images;
        for (int n_cam = 0; n_cam < points.size(); n_cam++) {
            if (points[n_cam][n_point].x == -1 || points[n_cam][n_point].y == -1) continue;

            images.push_back({cameras[n_cam], points[n_cam][n_point]});
        }
        
        if (images.size() < 2) {
            throw std::runtime_error("Too few detections are found");
        }

        std::pair<cv::Point3d, double> pointWithError = triangulatePoint(images);
        result.push_back(pointWithError.first);
    }

    return result;
}

std::pair<cv::Point3d, double> RayTriangulator::triangulatePoint(std::vector<CamPointPair> images){
    std::vector<Ray> rays;
    for(const auto& p : images){
        rays.push_back(createRayForPoint(p.camera, p.point));
    }

    cv::Point3d initGuess;

    for (const auto& ray : rays) {
        initGuess += ray.origin;
    }

    initGuess.x /= rays.size();
    initGuess.y /= rays.size();
    initGuess.z /= rays.size();

    cv::Ptr solverCompare = cv::Ptr<RayClosestPoint>(new RayClosestPoint(rays));
    cv::Ptr<cv::LMSolver> solver = cv::LMSolver::create(solverCompare, MAX_ITERATIONS);
    std::vector<double> params = { initGuess.x, initGuess.y, initGuess.z };
    int r = solver->run(params);

    return { {params[0], params[1], params[2]}, solverCompare->getError() };
}