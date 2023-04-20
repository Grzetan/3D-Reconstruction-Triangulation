//**************************************************************************************************//
// Camera.h																							//
// Author: Tomasz Krzeszowski																		//
// email: tkrzeszo@prz.edu.pl																		//
// Version: 0.1																						//
// Date: 23.03.2023																					//
//																									//
//**************************************************************************************************//

#ifndef _Camera_H
#define _Camera_H

#pragma once

#include <cmath>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace std;

namespace tdr
{
	constexpr auto RAD_TO_DEG = 57.29577951308232087679; //!< Radians to degrees factor.
	constexpr auto DEG_TO_RAD = 0.01745329251994329576; //!< Degrees to radians factor.

	/*! \Class Camera
	 * \brief Camera model implementation.
	 *
	 * Class contains methods for projecting points using the opencv camera model.
	 */
	class Camera
	{
	private:
		int id;

	public:
		
		cv::Mat rvec; //!< Rotation vector.
		cv::Mat rmat; //!< Rotation 
		cv::Mat tvec; //!< translation vector.
		cv::Mat rquat; //!< Rotation quaternion.
		cv::Mat camPos; //!< Camera position.
		cv::Mat cameraMatrix; //!< Camera matrix.
		cv::Mat cameraExtrinsicMatrix; //!< Camera extrinsic matrix
		cv::Mat cameraPerspectiveMatrix; //!< Camera perspective matrix
		cv::Mat distCoeffs; //!< Distorsion coeficient vector.
		cv::Vec3d upVec; //!< Camera up vector.
		 
		int width; //!< Image width.
		int height; //!< Image height.
		int diagonal; //!< Image diagonal.
		int cx; //!< Cx parameter.
		int cy; //!< Cy parameter.
		double fx; //!< Focal length x.
		double fy; //!< Focal length y.
		double fovx; //!< Field of view x.
		double fovy; //!< Field of view y.
		int fps; //!< Camera fps.

		Camera(int id):width(0), height(0), diagonal(0), cx(0), cy(0), fx(0), fy(0), fovx(0), fovy(0), fps(0)
		{ this->id = id; }

		/*! \brief Compute cx and cy.
		 *
		 *	Image width and height must be set elier.
		 */
		void compCxCy()
		{
			if (width == 0 || height == 0) throw std::runtime_error("Set width and height first.");
			cx = (int)round(width / 2.0);
			cy = (int)round(height / 2.0);
		}

		/*! \brief Compute field of view y.
		 *
		 *	Field of view x must be set elier.
		 */
		void compFovy()
		{
			if (fovx == 0) throw std::runtime_error("Set fovx first.");
			fovy = 2.0 * atan(tan(fovx * 0.5 * DEG_TO_RAD) / (double(width) / double(height))) * RAD_TO_DEG;
		}

		/*! \brief Compute focal length x and focal length y.
		 *
		 *	Image width, image height, field of view x, and field of view y must be set elier.
		 */
		void compFxFy()
		{
			if (width == 0 || height == 0 || fovx == 0 || fovy ==0 ) throw std::runtime_error("Set width, height, fovx, and fovy first.");
			fx = (width / 2.0) / (tan((fovx / 2.0) * DEG_TO_RAD));
			fy = (height / 2.0) / (tan((fovy / 2.0) * DEG_TO_RAD));

		}

		/*! \brief Compute camera matrix.
		 *	
		 *	Focal length x, focal length y, cx, and cy must be set elier.
		 */
		void createCamMat()
		{
			if (fx == 0 || fy == 0 || cx == 0 || cy == 0) throw std::runtime_error("Set fx, fy, cx, and cy first.");
			this->cameraMatrix = (cv::Mat_<double>(3, 3) <<
				fx, 0, cx,
				0, fy, cy,
				0, 0, 1);
		}

		/*! \brief Compute camera matrix.
		 *
		 *	tquat, campos must be set earlier
		 */
		void createExtricsicMat() {
			cv::Mat rotation_matrix = toRotMatrix(rquat);

			cv::Mat extrinsicMatrix(3, 4, CV_64F);
			extrinsicMatrix.at<double>(0, 0) = rotation_matrix.at<double>(0, 0);
			extrinsicMatrix.at<double>(0, 1) = rotation_matrix.at<double>(0, 1);
			extrinsicMatrix.at<double>(0, 2) = rotation_matrix.at<double>(0, 2);
			extrinsicMatrix.at<double>(0, 3) = camPos.at<double>(0);

			extrinsicMatrix.at<double>(1, 0) = rotation_matrix.at<double>(1, 0);
			extrinsicMatrix.at<double>(1, 1) = rotation_matrix.at<double>(1, 1);
			extrinsicMatrix.at<double>(1, 2) = rotation_matrix.at<double>(1, 2);
			extrinsicMatrix.at<double>(1, 3) = camPos.at<double>(1);

			extrinsicMatrix.at<double>(2, 0) = rotation_matrix.at<double>(2, 0);
			extrinsicMatrix.at<double>(2, 1) = rotation_matrix.at<double>(2, 1);
			extrinsicMatrix.at<double>(2, 2) = rotation_matrix.at<double>(2, 2);
			extrinsicMatrix.at<double>(2, 3) = camPos.at<double>(2);

			cameraExtrinsicMatrix = extrinsicMatrix;
		}

		/*! \brief Compute camera perspective matrix.
		 *
		 *	cameraMatrix and cameraExtrinsicMatrix must be set earlier
		 */
		void createPerspectiveMat() {
			cameraPerspectiveMatrix = cameraMatrix * cameraExtrinsicMatrix;
		}

		/*! \brief Compute camera postion based on OpenCV rotation and translation vector.
		 *
		 */
		void compCamPos()
		{
			cv::Rodrigues(rvec, rmat); 

			cv::Mat R = rmat.t(); 
			camPos = -R * tvec; 
		}

		/*! \brief Compute camera parameters.
		 *
		 * Computing parameters: Cx, Cy, field of view y, focal_x, focal_y, camera matrix, and camera postion.
		 */
		void compCamParams()
		{
			compCxCy();
			compFovy();
			compFxFy();
			compCamPos();
			createCamMat();
			createExtricsicMat();
			createPerspectiveMat();
			diagonal = (int)sqrt(width * width + height * height);
		}

		int getCamId() { return id; }

		/*! \brief Project 3D point to image space.
		 *
		 */
		cv::Point2d projectPoint(cv::Vec3d p3d, bool useDistCoeffs = false)
		{
			vector<cv::Vec3d> ptmp(1, p3d);
			vector<cv::Point2d> p2d;
			cv::Mat tmpDistCoeffs;
			if (useDistCoeffs) tmpDistCoeffs = this->distCoeffs;

			cv::projectPoints(ptmp, rvec, tvec, cameraMatrix, tmpDistCoeffs, p2d);
			return p2d[0];

		}

		/*! \brief Project vector of 3D points to image space.
		 *
		 */
		vector<cv::Point2d> projectPoints(vector<cv::Vec3d> p3d, bool useDistCoeffs = false)
		{
			vector<cv::Point2d> p2d;
			cv::Mat tmpDistCoeffs;
			if (useDistCoeffs) tmpDistCoeffs = this->distCoeffs;

			cv::projectPoints(p3d, rvec, tvec, cameraMatrix, tmpDistCoeffs, p2d);
			return p2d;
		}

		/*! \brief Remove distortions from image.
		 *
		 */
		cv::Mat undistordImage(const cv::Mat& distImage)
		{
			cv::Mat undistImage;
			cv::undistort(distImage, undistImage, this->cameraMatrix, this->distCoeffs);
			return undistImage;
		}

		/*! \brief Compute quaternion norm.
		 *
		 */
		static double norm(cv::Mat quat)
		{
			double x = quat.at<double>(0, 0);
			double y = quat.at<double>(1, 0);
			double z = quat.at<double>(2, 0);
			double w = quat.at<double>(3, 0);

			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		/*! \brief Do quaternion normalization.
		 *
		 */
		inline static cv::Mat normalize(cv::Mat quat) 
		{
			double normVal = norm(quat);
			if (normVal < 1.e-6)
			{
				// throw std::runtime_error("Cannot normalize this quaternion: the norm is too small.");
			}
			double x = quat.at<double>(0, 0) / normVal;
			double y = quat.at<double>(1, 0) / normVal;
			double z = quat.at<double>(2, 0) / normVal;
			double w = quat.at<double>(3, 0) / normVal;
		
			return (cv::Mat_<double>(4, 1) << x, y, z, w);
		}

		/*! \brief Convert quaternion to OpenCV rotation vector.
		 *
		 */
		static cv::Mat toRotVec(cv::Mat quat) 
		{
			double x = quat.at<double>(0, 0);
			double y = quat.at<double>(1, 0);
			double z = quat.at<double>(2, 0);
			double w = quat.at<double>(3, 0);

			double angle = 2 * std::acos(w / norm(quat));
			const double sin_v = std::sin(angle * 0.5);

			double tmp = (norm(quat) * sin_v);
			return angle*(cv::Mat_<double>(3, 1) << x/tmp, y/tmp, z/tmp);	
		}

		/*! \brief Convert quaternion to OpenCV rotation matrix.
		 *
		 */
		static cv::Mat toRotMatrix(cv::Mat quat)
		{
			cv::Mat qTemp = normalize(quat);
			double a = quat.at<double>(0, 0); //a = qTemp.w;
			double b = quat.at<double>(1, 0); //b = qTemp.x;
			double c = quat.at<double>(2, 0); //c = qTemp.y;
			double d = quat.at<double>(3, 0); //d = qTemp.z;
			
			cv::Matx<double, 3, 3> R{
				1 - 2 * (c * c + d * d), 2 * (b * c - a * d)    , 2 * (b * d + a * c),
				2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b),
				2 * (b * d - a * c)    , 2 * (c * d + a * b)    , 1 - 2 * (b * b + c * c)
			};
			return cv::Mat(R);
		}

		/*! \brief Compute OpenCV tvec based on rotation vector nad camera position.
		 *
		 */
		static cv::Mat comptvec(cv::Mat rvec, cv::Mat camPos)
		{
			cv::Mat rmat;
			cv::Rodrigues(rvec, rmat);

			return -rmat * camPos;
		}
	};
}
#endif // !_Camera_H
