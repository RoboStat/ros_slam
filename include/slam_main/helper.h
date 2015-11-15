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

std::string fixedNum(int value, int digits = 3) {
	std::string result;
	while (digits-- > 0) {
		result += ('0' + value % 10);
		value /= 10;
	}
	std::reverse(result.begin(), result.end());
	return result;
}

visualization_msgs::Marker createMarker() {

	visualization_msgs::Marker marker;
	// draw line b/w points
	uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type to our shape from earlier
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.scale.y = 0.05;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	return marker;
}

void inverseRT(cv::Mat& R, cv::Mat& T) {
	R = R.t();
	T = -R * T;
}

cv::Mat identityRT(){
	cv::Mat m(3,4,cv::DataType<float>::type,float(0));
	m.at<float>(0,0)=1;
	m.at<float>(1,1)=1;
	m.at<float>(2,2)=1;
	return m;
}

cv::Mat fullRT(const cv::Mat& R, const cv::Mat& T){
	cv::Mat RT(4,4,cv::DataType<float>::type, float(0));
	R.copyTo(RT.colRange(0,3).rowRange(0,3));
	T.copyTo(RT.col(3).rowRange(0,3));
	RT.at<float>(3,3)=1;
	return RT;
}
#endif /* INCLUDE_SLAM_MAIN_HELPER_H_ */
