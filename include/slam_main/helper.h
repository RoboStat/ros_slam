/*
 * helper.h
 *
 *  Created on: Nov 14, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_HELPER_H_
#define INCLUDE_SLAM_MAIN_HELPER_H_

//std
#include <string>
#include <iostream>
//opencv
#include <opencv2/core.hpp>
//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

std::string fixedNum(int value, int digits = 3);

visualization_msgs::Marker createPathMarker();
visualization_msgs::Marker createPointMarker();

void inverseRT(cv::Mat& R, cv::Mat& T);
cv::Mat identityRT();
cv::Mat fullRT(const cv::Mat& R, const cv::Mat& T);

#endif /* INCLUDE_SLAM_MAIN_HELPER_H_ */
