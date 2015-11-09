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
	void processImage(const cv::Mat& image,
				      int frameNum,
					  map<int, Landmark>& landmarks,
					  vector<Landmark>& trials,
					  vector<cv::KeyPoint>& unmatched);

	void initTrials(const cv::Mat& image,
					int frameNum,
					vector<Landmark>& trials);

	void refreshTrials(vector<cv::KeyPoint>& newPts,
					   vector<Landmark>& trials);

	void visualizeTrace(const map<int,Landmark>& landmarks,
						const vector<Landmark>& trials);
	cv::Mat getDisplayFrame();

private:
	vector<int>::iterator findNN(const cv::KeyPoint& point,
								 float rad, vector<int>& nn);
	bool trackLandmark(Landmark& landmark);
	void evenlyDetect(const cv::Mat& frame, vector<cv::KeyPoint>& kpts);

	cv::Ptr<cv::ORB> ptExtractor;
	cv::Ptr<cv::DescriptorMatcher> ptMatcher;

	float searchRad;
	float singleThre;
	float doubleRatio;

	int frameNum;
	cv::Mat frame;
	cv::Mat displayFrame;

	vector<cv::KeyPoint> kpts;
	set<int> matchPts;
	multimap<float, int> xind;
	multimap<float, int> yind;

};



#endif /* INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_H_ */
