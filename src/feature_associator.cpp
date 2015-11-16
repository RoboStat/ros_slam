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
		 map<int, Landmark>& landmarks,
		 const vector<Landmark>& trials) {

	using namespace cv;

	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		if(it->second.isInlier()) {
			it->second.visualizeTrace(displayFrame, YELLOW);
			it->second.setInlier(false);
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
		line(displayFrame,it->firstPoint().pt,it->getPair().pt,Scalar(0,0,255));
	}
}

cv::Mat FeatureAssociator::getDisplayFrame() {
	return displayFrame;
}
