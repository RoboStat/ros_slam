/*
 * helper.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: wenda
 */
#include <slam_main/helper.h>
//std
#include <math.h>

std::string fixedNum(int value, int digits) {
	std::string result;
	while (digits-- > 0) {
		result += ('0' + value % 10);
		value /= 10;
	}
	std::reverse(result.begin(), result.end());
	return result;
}

visualization_msgs::Marker createPathMarker() {

	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type to our shape from earlier
	marker.type = visualization_msgs::Marker::LINE_STRIP;

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
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	//marker.lifetime = ros::Duration();
	return marker;
}

visualization_msgs::Marker createPointMarker() {
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.action = visualization_msgs::Marker::ADD;

	marker.id = 1;
	marker.type = visualization_msgs::Marker::POINTS;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;

	return marker;
}

geometry_msgs::PoseStamped createPose(cv::Mat& R, cv::Mat& T) {
	geometry_msgs::PoseStamped poseStamp;
	poseStamp.header.frame_id = "/my_frame";
	poseStamp.header.stamp = ros::Time::now();

	geometry_msgs::Pose& pose = poseStamp.pose;
	pose.position.x = T.at<float>(0);
	pose.position.y = T.at<float>(1);
	pose.position.z = T.at<float>(2);

	// quaternian conversion
	float w = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1)
			+ R.at<float>(2,2)) / 2.0;
	float x = (R.at<float>(2,1)-R.at<float>(1,2))/(4.0*w);
	float y = (R.at<float>(0,2)-R.at<float>(2,0))/(4.0*w);
	float z = (R.at<float>(1,0)-R.at<float>(0,1))/(4.0*w);
	pose.orientation.w = w;
	pose.orientation.x = x;
	pose.orientation.y = y;
	pose.orientation.z = z;

	return poseStamp;
}

tf::Transform createTF(cv::Mat& R, cv::Mat& T) {
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(T.at<float>(0),
			T.at<float>(1),
			T.at<float>(2)));

	// quaternian conversion
	float w = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1)
				+ R.at<float>(2,2)) / 2.0;
	float x = (R.at<float>(2,1)-R.at<float>(1,2))/(4.0*w);
	float y = (R.at<float>(0,2)-R.at<float>(2,0))/(4.0*w);
	float z = (R.at<float>(1,0)-R.at<float>(0,1))/(4.0*w);
	tf::Quaternion q(x,y,z,w);
	transform.setRotation(q);

	return transform;
}


void inverseRT(cv::Mat& R, cv::Mat& T) {
	R = R.t();
	T = -R * T;
}

cv::Mat identityRT() {
	cv::Mat m(3, 4, cv::DataType<float>::type, float(0));
	m.at<float>(0, 0) = 1;
	m.at<float>(1, 1) = 1;
	m.at<float>(2, 2) = 1;
	return m;
}

cv::Mat fullRT(const cv::Mat& R, const cv::Mat& T) {
	cv::Mat RT(4, 4, cv::DataType<float>::type, float(0));
	R.copyTo(RT.colRange(0, 3).rowRange(0, 3));
	T.copyTo(RT.col(3).rowRange(0, 3));
	RT.at<float>(3, 3) = 1;
	return RT;
}

