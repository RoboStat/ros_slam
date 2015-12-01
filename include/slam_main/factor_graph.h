/*
 * factor_graph.h
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */

#ifndef INCLUDE_FACTOR_GRAPH_H_
#define INCLUDE_FACTOR_GRAPH_H_

#include <slam_main/camera.h>
//opencv
#include <opencv2/core.hpp>
//gtsam
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

class FactorGraph {
public:
	FactorGraph(const Camera& camera);

	void addFirstPose(int poseID);
	void addPose(int poseID, const cv::Mat& R, const cv::Mat& T);
	void advancePose(int poseID, cv::Mat R, cv::Mat T);
	void getPose(int poseID, cv::Mat& R, cv::Mat& T);

	void addLandMark(int landmarkID, const cv::Point3f& location);
	void getLandMark(int landmarkID, cv::Point3f& location);

	void addProjection(int poseID, int landmarkID, const cv::Point2f& location);
	void addStereo(int poseID, int landmarkID,
			const cv::Point2f& loc1, const cv::Point2f& loc2);

	void update();

	void printInitials();

private:
	gtsam::Cal3_S2::shared_ptr K;
	gtsam::Cal3_S2Stereo::shared_ptr KS;
	gtsam::noiseModel::Isotropic::shared_ptr measurementNoise;
	gtsam::noiseModel::Isotropic::shared_ptr stereoNoise;
	gtsam::ISAM2Params parameters;
	gtsam::ISAM2 isam;

	gtsam::NonlinearFactorGraph graph;
	gtsam::Values initial;
	gtsam::Values estimate;

	bool isFirstLandmark;
};



#endif /* INCLUDE_FACTOR_GRAPH_H_ */
