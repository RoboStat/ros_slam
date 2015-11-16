/*
 * feat_assoc_nn2.cpp
 *
 *  Created on: Nov 15, 2015
 *      Author: wenda
 */
#include <slam_main/feat_assoc_nn2.h>
//std
#include <math.h>
#include <iostream>

FeatAssocNN2::FeatAssocNN2()
:flann(new cv::flann::LshIndexParams(20, 10, 2)) {
}

void FeatAssocNN2::initTrials(
		const cv::Mat& image1,
		const cv::Mat& image2,
		int frameNum,
		vector<Landmark>& trials) {

	using namespace cv;
	this->displayFrame = image1.clone();
	this->frameNum = frameNum;
	this->matchPts.clear();
	this->kpts.clear();

	// feature detection and description
	vector<KeyPoint> kpts1, kpts2;
	Mat descp1, descp2;
	evenlyDetect(image1, kpts1);
	evenlyDetect(image2, kpts2);
	ptExtractor->compute(image1, kpts1, descp1);
	ptExtractor->compute(image2, kpts2, descp2);

	matchAdd(kpts1,kpts2,descp1,descp2,trials);
}

void FeatAssocNN2::refreshTrials(
		const cv::Mat& image2,
		vector<cv::KeyPoint>& newPts,
		vector<Landmark>& trials) {

	using namespace cv;
	this->matchPts.clear();
	this->kpts.clear();
	trials.clear();
	// compute descriptor and keypoints
	vector<KeyPoint> kpts2;
	Mat descp1, descp2;
	evenlyDetect(image2, kpts2);
	ptExtractor->compute(frame, newPts, descp1);
	ptExtractor->compute(image2, kpts2, descp2);

	matchAdd(newPts,kpts2,descp1,descp2,trials);
}

void FeatAssocNN2::matchAdd(
		const vector<cv::KeyPoint>& kpts1,
		const vector<cv::KeyPoint>& kpts2,
		const cv::Mat& descp1,
		const cv::Mat& descp2,
		vector<Landmark>& trials) {

	using namespace cv;
	// match the point using flann
	vector<DMatch> matches;
	flann.match(descp1, descp2, matches);

	// pick good ones
	std::vector<DMatch> goodmatches;
	for (auto it = matches.begin(); it != matches.end(); it++) {
		if (it->distance < 60 &&
			abs(kpts1[it->queryIdx].pt.y - kpts2[it->trainIdx].pt.y) < 2 &&
			kpts1[it->queryIdx].pt.x - kpts2[it->trainIdx].pt.x < 50 &&
			kpts1[it->queryIdx].pt.x - kpts2[it->trainIdx].pt.x > 2)

			goodmatches.push_back(*it);
	}

	// add to the trials
	trials.clear();
	for (auto it = goodmatches.begin(); it != goodmatches.end(); it++) {
		Landmark landmark(kpts1[it->queryIdx], descp1.row(it->queryIdx).clone(), frameNum);
		landmark.setPair(kpts2[it->trainIdx]);
		trials.push_back(landmark);
	}

	std::cout << "pair detect:" << kpts1.size() << std::endl;
	std::cout << "flann match:" << goodmatches.size() << std::endl;
}

