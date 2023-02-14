#include <iostream>
#include <math.h>

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
        return {0,0,0,0};
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

int main(){

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