#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "happly.h"

typedef std::vector<std::array<double, 3>> Path;

/** Funkcja oblicza punkt przecięcia dla ramion krzyża w 3d z uwzględnieniem, że jeden z 4 punktów może nie być współpaszczyznowy z pozostałymi  trzema
* @param[in] four_points wektor 4 punktów ramion krzyża w postaci niekoniecznie uporządkowanej
* @param[in] arms wektor dwóch wartości stanowiących długości ramion krzyża
* @param[in] err maksymalny dopuszczalny błąd między faktyczną długością ramienia, a długością obliczoną na podstawie punktów (w jednostkach tych samych co pozycje markerów ramion)
* @return wektor 5 punktów krzyża w postaci [1. punkt ramienia x,2. punkt ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt przecięcia]
*/
std::vector<cv::Point3d> cross3d(std::vector<cv::Point3d> four_points, std::vector<double> arms = std::vector<double>{9,6}, float err = 1000)
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
            std::cout << "HALO";
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

/** Funkcja oblicza globalne współrzędne punku podanego we współrzenych drona
* @param[in] cross wektor 5 punktów ramion krzyża w postaci [1. punkt ramienia x,2. punkt ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt przecięcia]
* @param[in] punkt w lokalnym układnie współrzędnych związanych z ramionami x i y
* @return punkt w globalnym układnie współrzędnych
*/
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

void readInputCSV(const char* dir, Path& path, int offset = 2, int frequency = 4){
    std::ifstream file(dir);
    std::string line, token;
    std::array<double, 3> point;
    int i=0;
    
    while (std::getline(file, line)){
        if(offset > i++ || (i+offset-1)%frequency != 0) continue; // Skip first `offset` lines and skip redundant frequencies
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

        cv::Point3d center = convert2global(cross, {50, 0, -20});

        point[0] = center.x;
        point[1] = center.y;
        point[2] = center.z;
        path.push_back(point);
    }
}

double calculateError(Path& labelPath, Path& predPath){
    double sumError = 0;
    size_t size = std::min(labelPath.size(), predPath.size());

    for(int i=0; i<size; i++){
        double err = std::sqrt(
            std::pow(predPath[i][0] - labelPath[i][0], 2) +
            std::pow(predPath[i][1] - labelPath[i][1], 2) +
            std::pow(predPath[i][2] - labelPath[i][2], 2)
        );
        // std::cout << err << std::endl;
        sumError += err;
    }

    return sumError / (double) size;
}

int main(int argc, const char** argv){
    if(argc != 3) throw std::runtime_error("Input paths must be provided");

    happly::PLYData inputPLY(argv[1]);
    Path predPath = inputPLY.getVertexPositions();
    Path labelPath;
    readInputCSV(argv[2], labelPath);

    double error = calculateError(labelPath, predPath);
    std::cout << error << std::endl;
}