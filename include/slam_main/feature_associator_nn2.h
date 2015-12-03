/*
 * feature_associator_nn2.h
 *
 *  Created on: Dec 2, 2015
 *      Author: wenda
 */
#ifndef INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_NN2_H_
#define INCLUDE_SLAM_MAIN_FEATURE_ASSOCIATOR_NN2_H_

#include <slam_main/feature_associator_nn.h>
#include <slam_main/camera.h>

class FeatureAssociatorNN2 : public FeatureAssociatorNN {
public:
	FeatureAssociatorNN2(const Camera& camera){
		this->camera = camera;
	}
	~FeatureAssociatorNN2(){}

	//normal tracking done in all the frames
	void processImage(
				const cv::Mat& image1,
				const cv::Mat& image2,
				int frameNum,
				map<int, Landmark>& landmarks,
				vector<Landmark>& trials,
				vector<cv::KeyPoint>& unmatched);

private:
	bool trackLandmarkByPred(
				Landmark& landmark,
				const cv::Mat& image1,
				const cv::Mat& image2,
				const vector<cv::KeyPoint>& kpts1,
				const vector<cv::KeyPoint>& kpts2,
				const NNFinder& nnFinder1,
				const NNFinder& nnFinder2,
				set<int>& matchPts1,
				set<int>& matchPts2);

	Camera camera;

	float searchRad2 = 2;
	float singleThre2 = 80;
	float doubleRatio2 = 0.9;
};

#endif



