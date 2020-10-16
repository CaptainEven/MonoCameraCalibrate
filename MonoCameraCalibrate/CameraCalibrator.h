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

#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraCalibrator 
{

	// input points
    std::vector<std::vector<cv::Point3f>> m_obj_pts;
    std::vector<std::vector<cv::Point2f>> m_img_pts;

    // output Matrices
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

	// flag to specify how calibration is done
	int flag;

	// used in image undistortion 
    cv::Mat map_1, map_2; 
	bool must_init_undistort;

  public:
	CameraCalibrator() : flag(0), must_init_undistort(true) {};

	// Open the chessboard images and extract corner points
	int addChessboardPoints(const std::vector<std::string>& file_list, const cv::Size& boardSize);

	// Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, 
		const std::vector<cv::Point3f>& objectCorners);

	// Calibrate the camera
	double calibrate(const cv::Size& imageSize);

    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);

	// Remove distortion in an image (after calibration)
	cv::Mat remap(const cv::Mat& image);

    // Getters
    cv::Mat getCameraMatrix() { return camera_matrix; }
    cv::Mat getDistCoeffs()   { return dist_coeffs; }
};

#endif // CAMERACALIBRATOR_H
