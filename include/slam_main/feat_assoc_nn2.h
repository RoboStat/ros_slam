/*
 * feat_assoc_nn2.h
 *
 *  Created on: Nov 15, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_SLAM_MAIN_FEAT_ASSOC_NN2_H_
#define INCLUDE_SLAM_MAIN_FEAT_ASSOC_NN2_H_

#include <slam_main/feature_associator_nn.h>

class FeatAssocNN2: public FeatureAssociatorNN {
public:
	FeatAssocNN2();
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

private:
	void matchAdd(
			const vector<cv::KeyPoint>& kpts1,
			const vector<cv::KeyPoint>& kpts2,
			const cv::Mat& descp1,
			const cv::Mat& descp2,
			vector<Landmark>& trials);

	cv::FlannBasedMatcher flann;
};

#endif /* INCLUDE_SLAM_MAIN_FEAT_ASSOC_NN2_H_ */
