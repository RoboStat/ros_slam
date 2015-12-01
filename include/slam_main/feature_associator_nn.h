/*
 * feature_associator.h
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_NN_H_
#define INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_NN_H_

#include <slam_main/feature_associator.h>
#include <slam_main/nn_finder.h>
#include <slam_main/keypoint_filter.h>

using namespace std;

class FeatureAssociatorNN: public FeatureAssociator {
public:
	FeatureAssociatorNN();
	//normal tracking done in all the frames
	void processImage(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int frameNum,
			map<int, Landmark>& landmarks,
			vector<Landmark>& trials,
			vector<cv::KeyPoint>& unmatched);

	//first key frame,
	//find matching points between left and right image
	void initTrials(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int frameNum,
			vector<Landmark>& trials);

	//subsequent key frame
	//find matching between unmatched points in left to right
	//should be called after process image
	void refreshTrials(
			const cv::Mat& image1,
			const cv::Mat& image2,
			int frameNum,
			vector<cv::KeyPoint>& newPts,
			vector<Landmark>& trials);

protected:
	bool trackLandmark(
			Landmark& landmark,
			const cv::Mat& image1,
			const cv::Mat& image2,
			const vector<cv::KeyPoint>& kpts1,
			const vector<cv::KeyPoint>& kpts2,
			const NNFinder& nnFinder1,
			const NNFinder& nnFinder2,
			set<int>& matchPts1,
			set<int>& matchPts2);

	bool trackPoint(
			const cv::KeyPoint& point,
			const cv::Mat& descp,
			const NNFinder& nnFinder,
			const vector<cv::KeyPoint>& kpts,
			const cv::Mat& image,
			cv::KeyPoint& newPoint,
			cv::Mat& newDescp,
			int& matchID);

	void matchAdd(
			const vector<cv::KeyPoint>& kpts1,
			const vector<cv::KeyPoint>& kpts2,
			const cv::Mat& descp1,
			const cv::Mat& descp2,
			int frameNum,
			vector<Landmark>& trials);

	void evenlyDetect(const cv::Mat& frame, vector<cv::KeyPoint>& kpts);

	cv::Ptr<cv::ORB> ptExtractor;
	cv::Ptr<cv::DescriptorMatcher> ptMatcher;
	KeyPointFilter filter;
	cv::FlannBasedMatcher flann;

	float searchRad;
	float singleThre;
	float doubleRatio;
};

#endif /* INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_NN_H_ */
