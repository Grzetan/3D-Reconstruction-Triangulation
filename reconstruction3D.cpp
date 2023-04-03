#include <iostream>
#include <opencv2/opencv.hpp>

struct Quaternion {
    double real, i, j, k;

    Quaternion operator*(Quaternion& q) {
        return {
            real * q.real - i * q.i - j * q.j - k * q.k,
            i * q.real + real * q.i + j * q.k - k * q.j,
            real * q.j - i * q.k + j * q.real + k * q.i,
            real * q.k + i * q.j - j * q.i + k * q.real
        };
    }

    void normalize() {
        double d = std::sqrt(real * real + i * i + j * j + k * k);
        real /= d;
        i /= d;
        j /= d;
        k /= d;
    }
};

cv::Mat constructIntrinsicsMatrix(double focal_length, cv::Point2d principalPoint) {
    cv::Mat matrix(3, 3, CV_64F);
    matrix.at<double>(0, 0) = focal_length;
    matrix.at<double>(0, 1) = 0;
    matrix.at<double>(0, 2) = principalPoint.x;

    matrix.at<double>(1, 0) = 0;
    matrix.at<double>(1, 1) = focal_length;
    matrix.at<double>(1, 2) = principalPoint.y;

    matrix.at<double>(2, 0) = 0;
    matrix.at<double>(2, 1) = 0;
    matrix.at<double>(2, 2) = 1;

    return matrix;
}

cv::Mat quaternionToRotationMatrix(Quaternion q) {
    cv::Mat r(3, 3, CV_64F);
    r.at<double>(0, 0) = 2 * (q.real * q.real + q.i * q.i) - 1;
    r.at<double>(0, 1) = 2 * (q.i * q.j - q.real * q.k);
    r.at<double>(0, 2) = 2 * (q.i * q.k + q.real * q.j);
    r.at<double>(1, 0) = 2 * (q.i * q.j + q.real * q.k);
    r.at<double>(1, 1) = 2 * (q.real * q.real + q.j * q.j) - 1;
    r.at<double>(1, 2) = 2 * (q.j * q.k - q.real * q.i);
    r.at<double>(2, 0) = 2 * (q.i * q.k - q.real * q.j);
    r.at<double>(2, 1) = 2 * (q.j * q.k + q.real * q.i);
    r.at<double>(2, 2) = 2 * (q.real * q.real + q.k * q.k) - 1;

    return r;
}

cv::Mat constructExtrinsicMatrix(cv::Point3d position, Quaternion orientation) {
    cv::Mat rotation_matrix = quaternionToRotationMatrix(orientation);

    cv::Mat extrinsicMatrix(3, 4, CV_64F);
    extrinsicMatrix.at<double>(0, 0) = rotation_matrix.at<double>(0, 0);
    extrinsicMatrix.at<double>(0, 1) = rotation_matrix.at<double>(0, 1);
    extrinsicMatrix.at<double>(0, 2) = rotation_matrix.at<double>(0, 2);
    extrinsicMatrix.at<double>(0, 3) = position.x;

    extrinsicMatrix.at<double>(1, 0) = rotation_matrix.at<double>(1, 0);
    extrinsicMatrix.at<double>(1, 1) = rotation_matrix.at<double>(1, 1);
    extrinsicMatrix.at<double>(1, 2) = rotation_matrix.at<double>(1, 2);
    extrinsicMatrix.at<double>(1, 3) = position.y;

    extrinsicMatrix.at<double>(2, 0) = rotation_matrix.at<double>(2, 0);
    extrinsicMatrix.at<double>(2, 1) = rotation_matrix.at<double>(2, 1);
    extrinsicMatrix.at<double>(2, 2) = rotation_matrix.at<double>(2, 2);
    extrinsicMatrix.at<double>(2, 3) = position.z;

    return extrinsicMatrix;
}

cv::Mat constructPerspectiveMatrix(cv::Mat intrinsicsMatrix, cv::Mat extrinsicMatrix) {
    return intrinsicsMatrix * extrinsicMatrix;
}

cv::Point3d calculateRayDirectionForPoint(cv::Mat perspectiveMatrix, cv::Point2d point) {
    cv::Mat p(1, 3, CV_64F);
    p.at<double>(0, 0) = point.x;
    p.at<double>(0, 1) = point.y;
    p.at<double>(0, 2) = 1;

    cv::Mat homoPoint;

    cv::Mat inversedPerspectiveMatrix;
    cv::invert(perspectiveMatrix, inversedPerspectiveMatrix);

    cv::perspectiveTransform(p, homoPoint, inversedPerspectiveMatrix);

    cv::Point3d rayDir = {
        homoPoint.at<double>(0, 0) / homoPoint.at<double>(0, 3),
        homoPoint.at<double>(0, 1) / homoPoint.at<double>(0, 3),
        homoPoint.at<double>(0, 2) / homoPoint.at<double>(0, 3)
    };

    return rayDir;
}

/*
1) Implement function that constructs perspective matrix od camera DONE
2) implement function that produces a vector for 2d point DONE
3) implement function that uses least squares to find closest 3d point to every vector

*/

int main()
{
    std::cout << "Hello World!\n";
}
