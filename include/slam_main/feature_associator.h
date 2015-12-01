/*
 * feature_associator.h
 *
 *  Created on: Nov 15, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_H_
#define INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_H_

#include <slam_main/landmark.h>
//std
#include <vector>
#include <map>
#include <set>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

class FeatureAssociator {
public:
	FeatureAssociator();
	virtual ~FeatureAssociator();
	//normal tracking done in all the frames
	virtual void processImage(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int frameNum,
			map<int, Landmark>& landmarks,
			vector<Landmark>& trials,
			vector<cv::KeyPoint>& unmatched)=0;

	//first key frame,
	//find matching points between left and right image
	virtual void initTrials(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int frameNum,
			vector<Landmark>& trials)=0;

	//subsequent key frame
	//find matching between unmatched points in left to right
	//should be called after process image
	virtual void refreshTrials(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int framNum,
			vector<cv::KeyPoint>& newPts,
			vector<Landmark>& trials)=0;

	//visualize the tracking trace of landmarks across frames
	virtual void visualizeTrace(map<int, Landmark>& landmarks,
			const vector<Landmark>& trials);
	virtual void visualizePair(const vector<Landmark>& trials);

	//retrieve the display frame
	virtual cv::Mat getDisplayFrame();

protected:
	cv::Mat displayFrame;
};

extern const cv::Scalar RED;
extern const cv::Scalar BLUE;
extern const cv::Scalar GREEN;
extern const cv::Scalar YELLOW;

#endif /* INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_H_ */
