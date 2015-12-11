/*
 * feature_associator.cpp
 *
 *  Created on: Nov 15, 2015
 *      Author: wenda
 */
#include <slam_main/feature_associator.h>

const cv::Scalar RED(0,0,255);
const cv::Scalar BLUE(255,0,0);
const cv::Scalar GREEN(0,255,0);
const cv::Scalar YELLOW(0,255,255);

FeatureAssociator::FeatureAssociator(){
}

FeatureAssociator::~FeatureAssociator(){
}

void FeatureAssociator::visualizeTrace(
		 const map<int, Landmark>& landmarks,
		 const vector<Landmark>& trials) {

	using namespace cv;

	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		if(it->second.isInlier()) {
			it->second.visualizeTrace(displayFrame, YELLOW);
		} else {
			it->second.visualizeTrace(displayFrame, BLUE);
		}
	}

	for (auto it = trials.begin(); it != trials.end(); it++) {
		it->visualizeTrace(displayFrame, GREEN);
	}
}

void FeatureAssociator::visualizePair(const vector<Landmark>& trials){
	using namespace cv;
	for(auto it=trials.begin(); it!=trials.end(); it++) {
		KeyPoint p1,p2;
		it->firstPointPair(p1,p2);
		line(displayFrame,p1.pt,p2.pt,Scalar(0,0,255));
	}
}

cv::Mat FeatureAssociator::getDisplayFrame() {
	return displayFrame;
}
