/*
 * visual_odometry.h
 *
 *  Created on: Nov 17, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_
#define INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_

class VisualOdometry {
public:
	void init(const Camera& camera);
	void run(const cv::Mat& left, const cv::Mat& right);
};


#endif /* INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_ */
