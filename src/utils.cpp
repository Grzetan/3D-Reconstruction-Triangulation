#include "utils.h"

void loadPointsOneDrone(const char* path, std::vector<std::vector<cv::Point2d>>& points, int offset){
    std::vector<std::string> files;
    for (const auto& dirEntry : recursive_directory_iterator(path)){
        std::string directory = dirEntry.path().u8string();
        if(directory.find(".csv") != std::string::npos && directory.find(".avi") == std::string::npos){
            files.push_back(directory);
        }
    }

    std::sort(files.begin(), files.end());
    std::string line, token;
    int frame, x1, y1, x2, y2;

    for(auto& f : files){
        int i=0;
        std::ifstream file(f);
        points.push_back({}); // Add new vector for current camera

        while (std::getline(file, line)){
            if(offset > i++) continue; // Skip first `offset` lines
            std::istringstream iss(line);
            std::vector<int> seperatedLine;

            while(std::getline(iss, token, ',')) {
                seperatedLine.push_back(std::stoi(token));
            }

            cv::Point2d point;
            if(seperatedLine.size() < 5){
                point.x = -1;
                point.y = -1;
            }else{
                point.x = seperatedLine[1] + (seperatedLine[3] - seperatedLine[1]) / 2; 
                point.y = seperatedLine[2] + (seperatedLine[4] - seperatedLine[2]) / 2;
            }
            points.back().push_back(point);
        }
    }
}

void loadPointsMultipleDrones(const char* path, std::vector<std::vector<std::vector<cv::Point2d>>>& points, int offset, int recordSize){
    std::vector<std::string> files;
    for (const auto& dirEntry : recursive_directory_iterator(path)){
        std::string directory = dirEntry.path().u8string();
        if(directory.find(".csv") != std::string::npos && directory.find(".avi") == std::string::npos){
            files.push_back(directory);
        }
    }

    std::sort(files.begin(), files.end());
    std::string line, token;
    int startFrame = 10000, endFrame = 12000;

    for(auto& f : files){
        int n_line=0, frame=-1;
        std::ifstream file(f);
        points.push_back({}); // Add new vector for current camera

        while (std::getline(file, line)){
            if(offset > n_line++) continue; // Skip first `offset` lines
            std::istringstream iss(line);
            std::vector<int> seperatedLine;

            while(std::getline(iss, token, ',')) {
                seperatedLine.push_back(std::stoi(token));
            }

            if(seperatedLine[0] <= startFrame || seperatedLine[0] > endFrame){
                frame = seperatedLine[0];
                continue;
            };
            
            for(int i=0; i<seperatedLine[0]-frame-1; i++){
                points.back().push_back({});
            }
            frame = seperatedLine[0];

            if((seperatedLine.size() - 1) % recordSize != 0)
                throw std::runtime_error("Invalid CSV file!");

            points.back().push_back({}); // Add vector for new frame

            for(int j=0; j<seperatedLine.size() / 6; j++){
                cv::Point2d point;
                point.x = seperatedLine[j*recordSize+5]; 
                point.y = seperatedLine[j*recordSize+6];
                points.back().back().push_back(point);
            }
        }
    }

    for(int i=1; i<points.size(); i++){
        if(points[i-1].size() != points[i].size())
            throw std::runtime_error("Number of frames on all cameras must be the same");
    }
}

std::vector<const tdr::Camera*> loadCamerasXML(const char* path){
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path);
    if (!result)
        throw std::runtime_error("Cannot open camera XML file");

    std::vector<const tdr::Camera*> cameras;

    for (pugi::xml_node camera : doc.child("Cameras").children("Camera")){
        pugi::xml_node controlFrame = camera.child("ControlFrames").child("ControlFrame");

        if(!controlFrame) continue;

        // Read camera's id
        int id = camera.attribute("DEVICEID").as_int();

        // Read focal length
        double focalLength = controlFrame.attribute("FOCAL_LENGTH").as_double();

        // Read width and height
        std::stringstream princialPoint = std::stringstream(controlFrame.attribute("PRINCIPAL_POINT").value());
        int width, height;
        princialPoint >> width >> height;
        width *= 2;
        height *= 2;

        // Read position
        std::stringstream position = std::stringstream(controlFrame.attribute("POSITION").value());
        double x, y, z;
        position >> x >> y >> z;

        // Read orientation
        std::stringstream orientation = std::stringstream(controlFrame.attribute("ORIENTATION").value());
        double w, i, j, k;
        orientation >> i >> j >> k >> w;

        const tdr::Camera* cam = createCamera(id, width, height, focalLength, (cv::Mat_<double>(3, 1) << x, y, z), (cv::Mat_<double>(4, 1) << w, -i, -j, -k));
        cameras.push_back(cam);
    }

    return cameras;
}

const tdr::Camera* createCamera(int id, size_t width, size_t height, double focalLength, cv::Mat translation, cv::Mat rotation){
    tdr::Camera* cam = new tdr::Camera(id);
    cam->width = width;
    cam->height = height;
    cam->fx = focalLength;
    cam->tvec = translation;
    cam->rvec = cam->toRotVec(rotation);
    cam->rquat = rotation;
    cam->compCamParams();

    return cam;
}

void writeOutputFile(const char* path, const std::vector<cv::Point3d>& triangulatedPoints, bool addFaces){
    std::ofstream out(path);
    out << "ply\n";
    out << "format ascii 1.0\n";
    int nVertex = addFaces ? triangulatedPoints.size() * 2 : triangulatedPoints.size();
    out << "element vertex " << nVertex << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    int nFaces = addFaces ? triangulatedPoints.size() : 0;
    out << "element face " << nFaces << "\n";
    out << "property list uchar int vertex_index\n";
    out << "end_header\n";

    double camSize = 0.5;
    double scale = 0.01; //0.01;
    double faceOffset = 0.1;

    // Visualize paths
    for(const auto& p : triangulatedPoints){
        cv::Point3d scaled = p * scale;
        out << scaled.x << " " << scaled.y << " " << scaled.z << "\n";
        if(addFaces)
            out << scaled.x + faceOffset << " " << scaled.y + faceOffset << " " << scaled.z + faceOffset << "\n";
    }

    if(addFaces){
        for(int i=0; i<triangulatedPoints.size(); i++){
            out << "3 " << i*2 << " " << i*2+1 << " " << i*2+2 << "\n";
        }
    }
}
