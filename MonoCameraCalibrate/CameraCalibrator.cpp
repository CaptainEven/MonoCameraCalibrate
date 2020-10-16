/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:
   Computer Vision Programming using the OpenCV Library.
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify,
   and distribute this source code, or portions thereof, for any purpose, without fee,
   subject to the restriction that the copyright notice may not be removed
   or altered from any source or altered source distribution.
   The software is released on an as-is basis and without any warranties of any kind.
   In particular, the software is not guaranteed to be fault-tolerant or free from failure.
   The author disclaims all warranties with regard to this software, any use,
   and any consequent failure, is purely the responsibility of the user.

   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include "CameraCalibrator.h"

// Open chessboard images and extract corner points
int CameraCalibrator::addChessboardPoints(
	const std::vector<std::string>& file_list,
	const cv::Size& board_size)
{
	// @param patternSize Number of inner corners per a chessboard row and column
	// (patternSize = cv::Size(points_per_row, points_per_colum) = cv::Size(columns, rows)).

	// the points on the chessboard
	std::vector<cv::Point2f> img_corners;
	std::vector<cv::Point3f> obj_corners;

	// 3D Scene Points:
	// Initialize the chessboard corners 
	// in the chessboard reference frame
	// The corners are at 3D location (X,Y,Z)= (i,j,0)
	for (int y = 0; y < board_size.height; ++y)  // rows
	{
		for (int x = 0; x < board_size.width; ++x)  // cols
		{
			obj_corners.push_back(cv::Point3f(float(x), float(y), 0.0f));
		}
	}

	// 2D Image points:
	cv::Mat image; // to contain chessboard image
	int successes = 0;

	// for all viewpoints
	for (int i = 0; i < file_list.size(); ++i)
	{
		// Open the image
		image = cv::imread(file_list[i], 0);

		// Get the chessboard corners
		bool found = cv::findChessboardCorners(image, board_size, img_corners);

		// Get subpixel accuracy on the corners
		cv::cornerSubPix(
			image,
			img_corners,
			cv::Size(5, 5),
			cv::Size(-1, -1),
			cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
				30,		// max number of iterations 
				0.1)    // min accuracy
		);

		// If we have a good board, add it to our data
		if (img_corners.size() == board_size.area())  // find all corners
		{
			// Add image and scene points from one view
			this->addPoints(img_corners, obj_corners);
			successes++;
		}

		//Draw the corners
		cv::drawChessboardCorners(image, board_size, img_corners, found);
		cv::imshow("Corners on Chessboard", image);
		cv::waitKey(100);
	}

	return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& img_corners,
	const std::vector<cv::Point3f>& obj_corners)
{

	// 2D image points from one view
	this->m_img_pts.push_back(img_corners);

	// corresponding 3D scene points
	this->m_obj_pts.push_back(obj_corners);
}

// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate(const cv::Size& img_size)
{
	// undistorter must be reinitialized
	must_init_undistort = true;

	//Output rotations and translations
	std::vector<cv::Mat> rvecs, tvecs;

	// start calibration
	return calibrateCamera(m_obj_pts, // the 3D points
		m_img_pts,  // the image points
		img_size,    // image size
		camera_matrix, // output camera matrix
		dist_coeffs,   // output distortion matrix
		rvecs, tvecs, // Rs, Ts 
		flag);        // set options
//						   cv::CALIB_USE_INTRINSIC_GUESS);

}

// remove distortion in an image (after calibration)
cv::Mat CameraCalibrator::remap(const cv::Mat &image)
{

	cv::Mat undistorted;

	if (must_init_undistort)
	{
		// called once per calibration

		cv::initUndistortRectifyMap(
			camera_matrix,  // computed camera matrix
			dist_coeffs,    // computed distortion matrix
			cv::Mat(),      // optional rectification (none) 
			cv::Mat(),      // camera matrix to generate undistorted
			cv::Size(640, 480),
			//            image.size(),  // size of undistorted
			CV_32FC1,      // type of output map
			map_1, map_2);   // the x and y mapping functions

		must_init_undistort = false;
	}

	// Apply mapping functions
	cv::remap(image, undistorted, map_1, map_2,
		cv::INTER_LINEAR);  // interpolation type

	return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

	// Set the flag used in cv::calibrateCamera()
	flag = 0;
	if (!tangentialParamEnabled) flag += cv::CALIB_ZERO_TANGENT_DIST;
	if (radial8CoeffEnabled) flag += cv::CALIB_RATIONAL_MODEL;
}

