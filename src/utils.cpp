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

void loadPointsMultipleDrones(const char* path, std::vector<std::vector<std::vector<cv::Point2d>>>& points, int offset, int recordSize, int startFrame, int endFrame){
    std::vector<std::string> files;
    for (const auto& dirEntry : recursive_directory_iterator(path)){
        std::string directory = dirEntry.path().u8string();
        if(directory.find(".csv") != std::string::npos && directory.find(".avi") == std::string::npos){
            files.push_back(directory);
        }
    }

    std::sort(files.begin(), files.end());
    std::string line, token;

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

            if((seperatedLine[0] <= startFrame || seperatedLine[0] > endFrame) && startFrame != endFrame){
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

            for(int j=0; j<seperatedLine.size() / recordSize; j++){
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

void writeOutputFile(const char* path, const std::vector<cv::Point3d>& triangulatedPoints){
    std::ofstream out(path);
    out << "ply\n";
    out << "format ascii 1.0\n";
    int nVertex = triangulatedPoints.size();
    out << "element vertex " << nVertex << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    int nFaces = 0;
    out << "element face " << nFaces << "\n";
    out << "property list uchar int vertex_index\n";
    out << "end_header\n";

    double scale = 1; //0.01;

    // Visualize paths
    for(const auto& p : triangulatedPoints){
        cv::Point3d scaled = p * scale;
        out << scaled.x << " " << scaled.y << " " << scaled.z << "\n";
    }
}

double calculateError(Path& labelPath, Path& predPath){
    double sumError = 0;
    size_t size = std::min(labelPath.size(), predPath.size());

    for(int i=0; i<size; i++){
        if(predPath[i].x == 0 && predPath[i].y == 0 && predPath[i].z == 0) continue;
        double err = std::sqrt(
            std::pow(predPath[i].x - labelPath[i].x, 2) +
            std::pow(predPath[i].y - labelPath[i].y, 2) +
            std::pow(predPath[i].z - labelPath[i].z, 2)
        );
        sumError += err;
    }

    return sumError / (double) size;
}

void readInputCSV(const char* dir, Path& path, int frequency, int startFrame, int endFrame){
    std::ifstream file(dir);
    std::string line, token;
    int i=0;
    
    while (std::getline(file, line)){
        if(line.find("Drone") != std::string::npos){
            continue;
        }

        if(i%frequency != 0 || ((i/frequency < startFrame || i/frequency > endFrame) && startFrame < endFrame)){
            i++;
            continue;
        };
        i++;
        
        std::istringstream iss(line);
        std::vector<double> seperatedLine;

        while(std::getline(iss, token, ',')) {
            seperatedLine.push_back(std::stod(token));
        }

        if(seperatedLine.size() != 12){
            throw std::runtime_error("Error at CSV file");
        }

        std::vector<cv::Point3d> markerPos;
        for(int j=0; j<4; j++){
            markerPos.push_back({seperatedLine[j*3], seperatedLine[j*3+1], seperatedLine[j*3+2]});
        }

        std::vector<cv::Point3d> cross = cross3d(markerPos);
        if(cross.size() != 5) continue;

        // cv::Point3d center = convert2global(cross, {50, 0, -20});
        cv::Point3d center = convert2global(cross, {-180, 200, -450});
        path.push_back(center);
    }
}

std::vector<cv::Point3d> cross3d(std::vector<cv::Point3d> four_points, std::vector<double> arms, float err)
{
	//znalezienie które ramiona znajdują sie naprzeciwko siebie poprzez sprawdzenie odległości między każdym możliwym dopasowaniem punktów w pary
	std::vector<int> arms_order;
	
		for (auto j : std::vector< std::vector<int>>{ std::vector<int> {1,2,3},std::vector<int> {2,1,3},std::vector<int> {3,1,2} })
		{
			auto arm1 = four_points[0] - four_points[j[0]];
			auto arm2 = four_points[j[1]] - four_points[j[2]];
			if ((abs(sqrt(arm1.ddot(arm1)) - arms[0]) < err && abs(sqrt(arm2.ddot(arm2)) - arms[1]) < err) || (abs(sqrt(arm1.ddot(arm1)) - arms[1]) < err && abs(sqrt(arm2.ddot(arm2)) - arms[0]) < err))
			{
				arms_order = j;
				break;
			}
		}
		
		if (arms_order.empty()){
			return std::vector<cv::Point3d>{};
        }

		//poniższe obliczenia zostały wyprowadzone z warunku, że iloczyn skalarny między wektorem rozpiętym od punktu środkowego do jednego z ramion, a wektorem od punktu środkowego do końca ramienia prostopadłego powinien wynosić 0
		cv::Point3d delta_collinear = four_points[arms_order[0]] - four_points[0];
		cv::Point3d delta_non_collinear = four_points[arms_order[1]] - four_points[0];
		double denominator = (delta_collinear.x * delta_collinear.x + delta_collinear.y * delta_collinear.y + delta_collinear.z * delta_collinear.z);
		double t = (delta_collinear.x * delta_non_collinear.x + delta_collinear.y * delta_non_collinear.y + delta_collinear.z * delta_non_collinear.z) / denominator;
		
		cv::Point3d cross_point = four_points[0] + delta_collinear * t;
		return std::vector<cv::Point3d> { four_points[0], four_points[arms_order[0]], four_points[arms_order[1]], four_points[arms_order[2]], cross_point };
}

cv::Point3d convert2global(std::vector<cv::Point3d> cross, cv::Point3d localPoint)
{
	cv::Point3d versorX = cross[1] - cross[0];//obliczam wektor kierunkowy osi x
	versorX = versorX / sqrt(versorX.dot(versorX)); // normalizacja 
	cv::Point3d versorY = cross[3] - cross[2];//obliczam wektor kierunkowy osi y
	versorY = versorY / sqrt(versorY.dot(versorY));// normalizacja 

	cv::Point3d versorZ = versorX.cross(versorY);//obliczam wersor kierunkowy osi z jako ilczyn wektorowy wersorów x i y - będzie porostopadły
	//versorZ = versorZ / sqrt(versorZ.dot(versorZ)); 

	cv::Point3d globalPoint;
	
	//nowe współrzędne to współrzędne środka lokalnego układu plus lokalne współrzędne punktów przemnożone razy wersory osi lokalnego układu widziane w globalnym układzie
	globalPoint = (cross[4] + localPoint.x * versorX + localPoint.y * versorY + localPoint.z * versorZ);

	return globalPoint;
}
