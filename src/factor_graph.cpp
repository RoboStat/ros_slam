/*
 * factor_graph.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */
#include <slam_main/factor_graph.h>
#include <slam_main/helper.h>
//std
#include <iostream>
#include <fstream>
//gtsam
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/BetweenFactor.h>

gtsam::Matrix4 convertFromCV(const cv::Mat& R, const cv::Mat& T) {
	// convert R,T to gtsam matrix
	gtsam::Matrix4 change;
	change.setZero();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			change(i, j) = R.at<double>(i, j);
		}
	}
	change(0, 3) = T.at<double>(0, 0);
	change(1, 3) = T.at<double>(1, 0);
	change(2, 3) = T.at<double>(2, 0);
	change(3, 3) = 1;

	return change;
}

FactorGraph::FactorGraph(const Camera& camera) {
	// camera parameters
	K = gtsam::Cal3_S2::shared_ptr(
			new gtsam::Cal3_S2(camera.focalLength, camera.focalLength,
					0.0, camera.principalPoint.x, camera.principalPoint.y));

	KS = gtsam::Cal3_S2Stereo::shared_ptr(
			new gtsam::Cal3_S2Stereo(camera.focalLength, camera.focalLength,
					0.0, camera.principalPoint.x, camera.principalPoint.y, camera.baseline));

	//observation noise just right
	measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
	stereoNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
	//prior noise bigger
	poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
						gtsam::Vector3::Constant(10.0), gtsam::Vector3::Constant(10.0)).finished());
	pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 10.0);
	//loop noise smaller
	loopNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
							gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());

	//isam parameters
	parameters.relinearizeThreshold = 0.01;
	parameters.relinearizeSkip = 1;
	isam = gtsam::ISAM2(parameters);

	//initial clean up
	initial.clear();
	estimate.clear();

	isFirstLandmark = true;
}

void FactorGraph::addFirstPose(int poseID) {
	using namespace gtsam;
	// Add a prior on pose x0
	// 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
	noiseModel::Diagonal::shared_ptr poseNoise =
			noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
	graph.push_back(PriorFactor<Pose3>(Symbol('x', poseID), Pose3(), poseNoise));

	initial.insert(symbol('x', poseID), Pose3());
	estimate.insert(symbol('x', poseID), Pose3());
}

void FactorGraph::addPose(int poseID, const cv::Mat& R, const cv::Mat& T) {
	cv::Mat invR = R.t();
	cv::Mat invT = -R * T;

	initial.insert(gtsam::symbol('x', poseID),
			gtsam::Pose3(convertFromCV(invR, invT)));
}

void FactorGraph::advancePoseTrivial(int poseID) {
	gtsam::Pose3 lastpose = estimate.at<gtsam::Pose3>(gtsam::symbol('x', poseID - 1));

	// insert the pose
	initial.insert(gtsam::symbol('x', poseID),lastpose);
	estimate.insert(gtsam::symbol('x', poseID),lastpose);

	// add prior
	graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', poseID), lastpose, poseNoise));
}


void FactorGraph::advancePose(int poseID, cv::Mat R, cv::Mat T) {
	gtsam::Pose3 lastpose = estimate.at<gtsam::Pose3>(gtsam::symbol('x', poseID - 1));
	gtsam::Matrix4 pm = lastpose.matrix();
	// convert R,T to gtsam matrix
	inverseRT(R, T);
	gtsam::Matrix4 change = convertFromCV(R, T);
	// increment the pose
	pm = pm * change;
	// insert the pose
	initial.insert(gtsam::symbol('x', poseID), gtsam::Pose3(pm));

	// add prior
	graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', poseID), gtsam::Pose3(pm), poseNoise));
}

void FactorGraph::getPose(int poseID, cv::Mat& R, cv::Mat& T) {
	R.create(3, 3, cv::DataType<float>::type);
	T.create(3, 1, cv::DataType<float>::type);

	gtsam::Matrix4 m = estimate.at<gtsam::Pose3>(gtsam::symbol('x', poseID)).matrix();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R.at<float>(i, j) = m(i, j);
		}
	}
	T.at<float>(0, 0) = m(0, 3);
	T.at<float>(1, 0) = m(1, 3);
	T.at<float>(2, 0) = m(2, 3);
}

void FactorGraph::addLandMark(int landmarkID, const cv::Point3f& location) {
	gtsam::Point3 p = gtsam::Point3(location.x, location.y, location.z);
	initial.insert(gtsam::symbol('l', landmarkID), p);

	if (isFirstLandmark) {
		isFirstLandmark = false;
		// Add a prior on landmark l0
		gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
		graph.push_back(gtsam::PriorFactor<gtsam::Point3>(
				gtsam::Symbol('l', landmarkID), p, pointNoise));
	}

	// add prior
	graph.push_back(gtsam::PriorFactor<gtsam::Point3>(
			gtsam::Symbol('l', landmarkID), p, pointNoise));
}

void FactorGraph::getLandMark(int landmarkID, cv::Point3f& location) {
	gtsam::Point3 p = estimate.at<gtsam::Point3>(gtsam::symbol('l', landmarkID));
	location.x = p.x();
	location.y = p.y();
	location.z = p.z();
}

void FactorGraph::addProjection(int poseID, int landmarkID, const cv::Point2f& location) {
	graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
			gtsam::Point2(location.x, location.y), measurementNoise,
			gtsam::Symbol('x', poseID), gtsam::Symbol('l', landmarkID), K));
}

void FactorGraph::addStereo(int poseID, int landmarkID,
		const cv::Point2f& loc1, const cv::Point2f& loc2) {
	graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(
			gtsam::StereoPoint2(loc1.x, loc2.x, loc1.y), stereoNoise,
			gtsam::Symbol('x', poseID), gtsam::Symbol('l', landmarkID), KS));
}

void FactorGraph::addLoopConstraint(int poseID1, int poseID2) {
	graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(poseID1,poseID2,
			gtsam::Pose3(gtsam::Matrix4::Identity(4,4)),loopNoise));
}


void FactorGraph::increUpdate() {
	using namespace gtsam;

	// incremental
	isam.update(graph, initial);
	isam.update();
	estimate = isam.calculateEstimate();
	graph.resize(0);
	initial.clear();
}

void FactorGraph::batchUpdate() {
	using namespace gtsam;

	// batch
	estimate = LevenbergMarquardtOptimizer(graph, initial).optimize();
	initial = estimate;
}

void FactorGraph::printInitials() {
	std::cout << "---------Graph Initial Estimation--------" << std::endl;
	initial.print();
	//graph.print();
}

void FactorGraph::visualizeTraj(visualization_msgs::Marker& marker) {
	gtsam::Values poses = estimate.filter<gtsam::Pose3>();
	marker.header.stamp = ros::Time::now();
	marker.points.clear();
	for (auto pose : poses) {
		gtsam::Pose3 p = static_cast<gtsam::Pose3&>(pose.value);
		geometry_msgs::Point pp;
		pp.x = p.x();
		pp.y = p.y();
		pp.z = p.z();
		marker.points.push_back(pp);
	}
}

void FactorGraph::visualizeLandmark(visualization_msgs::Marker& marker) {
	gtsam::Values landmarks = estimate.filter<gtsam::Point3>();
	marker.header.stamp = ros::Time::now();
	marker.points.clear();
	for (auto landmark : landmarks) {
		gtsam::Point3 l = static_cast<gtsam::Point3&>(landmark.value);
		geometry_msgs::Point ll;
		ll.x = l.x();
		ll.y = l.y();
		ll.z = l.z();
		marker.points.push_back(ll);
	}
}

void FactorGraph::exportGraph(std::string fileName) {
	using namespace std;
	ofstream file;
	file.open(fileName.c_str(),ios::out | ios::trunc);
	if(file.is_open()) {
		// iterate over all poses, write to file
		gtsam::Values poses = estimate.filter<gtsam::Pose3>();
		for (auto pose : poses) {
			gtsam::Pose3 p = static_cast<gtsam::Pose3&>(pose.value);
			file <<p.x()<< " " <<p.y()<< " " <<p.z()<<"\n";
		}
		file.close();
	} else {
		std::cerr<<"!!!fail to open traj export file!!!"<<std::endl;
	}
}

