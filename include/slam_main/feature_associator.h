/*
 * feature_associator.h
 *
 *  Created on: Nov 7, 2015
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
	//normal tracking done in all the frames
	void processImage(const cv::Mat& image,
				      int frameNum,
					  map<int, Landmark>& landmarks,
					  vector<Landmark>& trials,
					  vector<cv::KeyPoint>& unmatched);

	//first key frame,
	//find matching points between left and right image
	void initTrials(const cv::Mat& image1,
					const cv::Mat& image2,
					int frameNum,
					vector<Landmark>& trials);

	//subsequent key frame
	//find matching between unmatched points in left to right
	//should be called after process image
	void refreshTrials(const cv::Mat& image2,
					   vector<cv::KeyPoint>& newPts,
					   vector<Landmark>& trials);

	//visualize the tracking trace of landmarks across frames
	void visualizeTrace(const map<int,Landmark>& landmarks,
						const vector<Landmark>& trials);
	void visualizePair(const vector<Landmark>& trials);

	//retrieve the display frame
	cv::Mat getDisplayFrame();

private:
	bool trackLandmark(Landmark& landmark, const NNFinder& nnFinder);
	bool pairLandmark(Landmark& landmark, const NNFinder& nnFinder);
	void evenlyDetect(const cv::Mat& frame, vector<cv::KeyPoint>& kpts);

	cv::Ptr<cv::ORB> ptExtractor;
	cv::Ptr<cv::DescriptorMatcher> ptMatcher;

	float searchRad;
	float disparityRad;
	float singleThre;
	float doubleRatio;

	int frameNum;
	cv::Mat frame;
	cv::Mat displayFrame;

	vector<cv::KeyPoint> kpts;
	set<int> matchPts;
};



#endif /* INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_H_ */
