#include <slam_main/visual_odometry.h>
#include <slam_main/helper.h>
//std
#include <iostream>
//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

//////////////////////main///////////////////////////
int main( int argc, char** argv ) {
	cv::namedWindow("SLAM", cv::WINDOW_NORMAL);
	cv::resizeWindow("SLAM",760,500);

	// video source
	int startFrame = 380;
	int endFrame = 4000;
	//string path = "/home/wenda/Developer/Autonomy/cmu_16662_p2/sensor_data/";
	string path = "/home/wenda/Developer/Autonomy/cmu_16662_p2/NSHLevel2_Images/";

	// visual odometry stuff
	Camera camera;
	VisualOdometry vo(camera);
	vo.setStartEndFrame(startFrame, endFrame);

	//rviz marker setup
	ros::init(argc, argv, "slam_traj");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	for (int i = startFrame; i < endFrame; i++) {
		//read image
		cv::Mat left_frame = cv::imread(path + "left" + fixedNum(i,4) + ".jpg");
		cv::Mat right_frame = cv::imread(path + "right" + fixedNum(i,4) + ".jpg");

		// run vo
		vo.run(left_frame,right_frame,i);

		//frame info
		cv::Mat curR,curT;
		if(vo.getRT(curR, curT,i)) {
			marker_pub.publish(vo.visualizeTraj());
			marker_pub.publish(vo.visualizeLandmark());
		}
		cout << "current pose:" << curR <<endl;
		cout << curT << endl;

		//visualize frame
		cv::Mat display;
		vo.getDisplayFrame(display);
		cv::imshow("SLAM", display);
		cv::waitKey(1);

	}

	return 0;
}
