/*
 * visual_odometry.h
 *
 *  Created on: Nov 17, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_
#define INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_

#include <slam_main/feature_associator.h>
#include <slam_main/feature_associator_nn.h>
#include <slam_main/factor_graph.h>
#include <slam_main/camera.h>
//std
#include <vector>
#include <map>

class VisualOdometry {
public:
	// init
	VisualOdometry(const Camera& camera);
	~VisualOdometry();
	void setStartEndFrame(int start, int end);

	// run
	long run(const cv::Mat& left_frame, const cv::Mat& right_frame, int i);

	// get result
	bool getRT(cv::Mat& R, cv::Mat& T, int i);
	void getDisplayFrame(cv::Mat& displayFrame);

private:
	void updateLandmark();
	void triangulate(const cv::Mat& pts1,
					 const cv::Mat& pts2,
					 cv::Mat& outpts);
	void confirmTrials();

	// camera info
	Camera camera;

	// all the tracked landmarks
	int landmarkID = 0;
	std::map<int, Landmark> landmarks;
	std::vector<Landmark> trials;
	std::vector<cv::KeyPoint> unmatched;

	// features associator
	unsigned int landmarkThre = 50;
	unsigned int trialThre = 300;
	FeatureAssociator* featureAssoc;

	// factor graph
	FactorGraph graph;

	// Pose Chain
	std::vector<cv::Mat> allR;
	std::vector<cv::Mat> allT;

	// states
	bool trialState = true;

	// start end frame
	int startFrame = 0;
	int endFrame = 1000;
};


#endif /* INCLUDE_SLAM_MAIN_VISUAL_ODOMETRY_H_ */
