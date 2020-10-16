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

#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CameraCalibrator.h"

int main()
{

	cv::namedWindow("Image");

	cv::Mat image;
	std::vector<std::string> file_list;

	// generate list of chessboard image filename
	for (int i = 1; i <= 20; ++i)
	{

		std::stringstream str;
		str << "../chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
		std::cout << str.str() << std::endl;

		// collect images
		file_list.push_back(str.str());

		image = cv::imread(str.str(), 0);
		cv::imshow("Image", image);
		cv::waitKey(100);  // 0.1s
	}

	// Create calibrator object
	CameraCalibrator camera_calibrator;

	// add the corners from the chessboard
	cv::Size board_size(6, 4);  // (points_per_row, points_per_col): (cols, rows)
	camera_calibrator.addChessboardPoints(
		file_list,	// filenames of chessboard image
		board_size);	// size of chessboard
		// calibrate the camera
	//	cameraCalibrator.setCalibrationFlag(true, true);

	// Calibration
	cv::Size img_size = image.size();
	camera_calibrator.calibrate(img_size);

	// Image Undistortion
	image = cv::imread(file_list[6]);
	cv::Mat uImage = camera_calibrator.remap(image);

	// display camera matrix
	cv::Mat cameraMatrix = camera_calibrator.getCameraMatrix();
	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
	std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl;

	imshow("Original Image", image);
	imshow("Undistorted Image", uImage);

	cv::waitKey();
	return 0;
}

// reference: http://xilinx.eetrend.com/d6-xilinx/blog/2016-04/9958.html
