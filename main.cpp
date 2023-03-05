#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <array>

// Frame -> Drone -> X, Y, Z
typedef std::vector<std::vector<std::array<double, 3>>> MarkerData;

struct Vec3{
    double x, y, z;

    Vec3 operator+(Vec3& v){
        return {x + v.x, y + v.y, z + v.z};
    }

    Vec3 operator-(Vec3& v){
        return {x - v.x, y - v.y, z - v.z};
    }

    Vec3 operator*(double v){
        return {x * v, y * v, z * v};
    }

    Vec3 operator/(double v){
        return {x / v, y / v, z / v};
    }

    Vec3 crossProduct(Vec3& v){
        return {
            y * v.z - z * v.y,
            x * v.z - z * v.x,
            x * v.y - y * v.x
        };
    }

    double dotProduct(Vec3& v){
        return x * v.x + y * v.y + z * v.z;
    }

    double magnitude(){
        return std::sqrt(x*x + y*y + z*z);
    }
};

struct Quaternion{
    double real, i, j, k;

    Quaternion operator*(Quaternion& q){
        return {
            real*q.real - i*q.i - j*q.j - k*q.k,
            i*q.real + real*q.i + j*q.k - k*q.j,
            real*q.j - i*q.k + j*q.real + k*q.i,
            real*q.k + i*q.j - j*q.i + k*q.real
        };
    }
};

Vec3 computePlaneNormal(Vec3 p1, Vec3 p2, Vec3 p3){
    Vec3 vec1 = p2 - p1;
    Vec3 vec2 = p3 - p1;

    return vec1.crossProduct(vec2);
}

Quaternion computeRotationQuaternion(Vec3 planeNormal1, Vec3 planeNormal2){
    Vec3 rotationAxis = planeNormal1.crossProduct(planeNormal2);
    double theta = std::acos(planeNormal1.dotProduct(planeNormal2) / (planeNormal1.magnitude() * planeNormal2.magnitude() + 1e-8));

    return {std::cos(theta / 2.0), rotationAxis.x, rotationAxis.y, rotationAxis.z};
}

void parseData(std::string path, MarkerData& data){
    std::ifstream file(path);
    std::string line;

    // Remove header and second line
    std::getline(file, line);
    std::getline(file, line);

    int count = 0;

    while(std::getline(file, line)){
        std::stringstream ss(line);
        std::string token;
        std::vector<std::array<double, 3>> frame;
        std::array<double, 3> drone;


        while(std::getline(ss, token, ',')) {
            double val = std::stod(token);
            if(count == 3){
                frame.push_back(drone);
                count = 0;
            }
            drone[count] = val;
            count++;
        }
        frame.push_back(drone); // Add last drone
        std::cout << frame.size() << std::endl;

        data.push_back(frame);
    }
}

int main(){

    MarkerData data;
    parseData("./Dron T02.csv", data);

    std::cout << data[0].size() << std::endl;
    // for(auto& x : data){
    //     for(auto& y : x){
    //         std::cout << y[0] << ", " << y[1] << ", " << y[2] << std::endl;
    //     }
    // }

    // Czy można założyc że te 4 punkty sa wspolnopłaszczyznowe?? Jeśli nie to które punkty wyznaczaja plaszczyzne?
    Vec3 p1 = {0, 0, 0};
    Vec3 p2 = {0, 4, 5};
    Vec3 p3 = {0, -2, 8};
    Vec3 p4 = {0, 9, -3};

    Vec3 planeNormal = computePlaneNormal(p1, p2, p3);

    // Zakładam że druga płasczyzna jest podana jako kwaternion wiec mamy już jej wektor normalny
    Vec3 groundPlane = {0,0,0};

    // Obliczam kwaternion rotacji między jedna a druga plaszczyzna
    Quaternion rotationQuaternion = computeRotationQuaternion(planeNormal, groundPlane);

    std::cout << "Rotation quaternion: r = " 
              << rotationQuaternion.real << ", i = " 
              << rotationQuaternion.i << ", j = " 
              << rotationQuaternion.j << ", k = " 
              << rotationQuaternion.k << std::endl;
}