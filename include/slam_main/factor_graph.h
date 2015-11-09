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
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

class FactorGraph {
public:
	FactorGraph(const Camera& camera);

	void addFirstPose(int poseID);
	void advancePose(int poseID, const cv::Mat& R, const cv::Mat& T);
	void getPose(int poseID, cv::Mat& R, cv::Mat& T);

	void addLandMark(int landmarkID, const cv::Point3d& location);
	void getLandMark(int landmarkID, cv::Point3d& location);

	void addProjection(int poseID, int landmarkID, const cv::Point2d& location);

	void update();

	void printInitials();

private:
	gtsam::Cal3_S2::shared_ptr K;
	gtsam::noiseModel::Isotropic::shared_ptr measurementNoise;

	gtsam::ISAM2Params parameters;
	gtsam::ISAM2 isam;

	gtsam::NonlinearFactorGraph graph;
	gtsam::Values initial;
	gtsam::Values estimate;

	bool isFirstLandmark;
};



#endif /* INCLUDE_FACTOR_GRAPH_H_ */
