/*
 * camera.h
 *
 *  Created on: Nov 8, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_CAMERA_H_
#define INCLUDE_SLAM_MAIN_CAMERA_H_

#include <opencv2/core.hpp>

struct Camera {

	Camera(){
		focalLength = 164.25;
		principalPoint = cv::Point2f(213.51,118.43);
		intrinsic=(cv::Mat_<double>(3,3)<<
				focalLength,0,principalPoint.x,
				0,focalLength,principalPoint.y,
				0,0,1);
	}

	double focalLength;
	cv::Point2d principalPoint;
	cv::Mat intrinsic;
};



#endif /* INCLUDE_SLAM_MAIN_CAMERA_H_ */
