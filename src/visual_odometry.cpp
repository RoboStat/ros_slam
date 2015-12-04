/*
 * visual_odometry.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: wenda
 */
#include <slam_main/visual_odometry.h>
#include <slam_main/helper.h>
#include <slam_main/feature_associator_nn.h>
#include <slam_main/feature_associator_nn2.h>
//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

VisualOdometry::VisualOdometry(const Camera& camera) :
		graph(camera) {
	featureAssoc = new FeatureAssociatorNN2(camera);
}

VisualOdometry::~VisualOdometry() {
	delete featureAssoc;
}

long VisualOdometry::run(const cv::Mat& left_frame, const cv::Mat& right_frame) {

	// print frame info
	cout << "start>> landmarks:" << landmarks.size()
			<< " trials:" << trials.size() << endl;

	// run time counting
	long t1 = cv::getTickCount();

	// state machine
	switch (state) {

	case VO_START:
		graph.addFirstPose(frameCount);
		featureAssoc->initTrials(left_frame, right_frame, frameCount, trials);
		featureAssoc->visualizePair(trials);
		state = VO_BOOT;
		break;

	case VO_BOOT:
		featureAssoc->processImage(left_frame, right_frame, frameCount, landmarks, trials, unmatched);
		if (trials.size() < trialThre+200) {
			cout << "--confirm all initial trials--" << endl;
			confirmTrials();
			trialState = false;
			state = VO_NORMAL;
		}
		graph.advancePoseTrivial(frameCount);
		break;

	case VO_NORMAL:
		// abnormal case
		if(landmarks.empty()) {
			cerr<<"!!!!!!!!!!!!ERROR::NO LANDMARKS!!!!!!!!!!!"<<endl;
			graph.advancePoseTrivial(frameCount);
			break;
		}

		// run tracker
		featureAssoc->processImage(left_frame, right_frame, frameCount, landmarks, trials, unmatched);

		// confirm all trails
		if (trialState && trials.size() < trialThre) {
			cout << "--confirm all trials--" << endl;
			confirmTrials();
			trialState = false;
		}

		// prepare points
		cv::Mat pts1(2, landmarks.size(), cv::DataType<float>::type),
				pts2(2, landmarks.size(), cv::DataType<float>::type);
		vector<cv::Point2f> imgPts;
		int colcount = 0;
		for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
			cv::KeyPoint k1, k2;
			it->second.prevPointPair(k1, k2);
			pts1.at<float>(0, colcount) = k1.pt.x;
			pts1.at<float>(1, colcount) = k1.pt.y;
			pts2.at<float>(0, colcount) = k2.pt.x;
			pts2.at<float>(1, colcount) = k2.pt.y;
			imgPts.push_back(it->second.curLeftPoint().pt);
			colcount++;
		}

		// triangulate 3D points
		cv::Mat pts3D;
		triangulate(pts1, pts2, pts3D);

		// prepare 3D points
		cv::Mat R, Rvec, Tvec, inliers;
		vector<cv::Point3f> objPts;
		for (int c = 0; c < pts3D.cols; c++) {
			objPts.push_back(cv::Point3f(pts3D.at<float>(0, c),
					pts3D.at<float>(1, c),
					pts3D.at<float>(2, c)));
		}

		// incremental 3D-2D VO
		cv::solvePnPRansac(objPts, imgPts, camera.intrinsic,
				cv::noArray(), Rvec, Tvec, false, 3000, 1.0, 0.99, inliers, cv::SOLVEPNP_P3P);
		cv::Rodrigues(Rvec, R);

		//cout << R << Tvec << endl;
		//cout << inliers << endl;
		cout << "P3P points:" << imgPts.size() << " inliers:" << inliers.rows << endl;

		//set inliers
		int j = 0, count = 0;
		if (inliers.rows > 0) {
			for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
				if (count++ == inliers.at<int>(j, 0)) {
					it->second.setInlier(true);
					if (++j >= inliers.rows)
						break;
				}
			}
		} else {
			cerr<<"!!!!!!!!!!!!WARNING::PNP FAILED!!!!!!!!!!!"<<endl;
			R = cv::Mat::eye(3, 3, cv::DataType<float>::type);
			Tvec = cv::Mat::zeros(3, 1, cv::DataType<float>::type);
		}

		// add stereo factors
		for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
			if (it->second.isInlier()) {
				cv::KeyPoint p1, p2;
				it->second.curPointPair(p1, p2);
				graph.addStereo(frameCount, it->first, p1.pt, p2.pt);
			}
		}

		// add initial pose estimation
		allR.push_back(R);
		allT.push_back(Tvec);
		graph.advancePose(frameCount, R, Tvec);

		// run bundle adjustment
		long u1 = cv::getTickCount();
		//graph.batchUpdate();
		graph.increUpdate();
		long u2 = cv::getTickCount();
		cout << "optimization:" << float(u2 - u1) / cv::getTickFrequency() << endl;
		updateLandmark();

		if (!trialState && landmarks.size() < landmarkThre) {
			cout << "--init new trails--" << endl;
			featureAssoc->refreshTrials(left_frame, right_frame, frameCount, unmatched, trials);
			featureAssoc->visualizePair(trials);
			trialState = true;

			if (trials.size() < directAddThre || landmarks.size() < 15) {
				//just add them don't wait
				cerr<<"!!!!!!!!!!!!WARNING::LOW LANDMARKS!!!!!!!!!!!"<<endl;
				confirmTrials();
				trialState = false;
			}
		}
		break;
	}

	// advance frame count number
	frameCount++;

	// count runing time
	long t2 = cv::getTickCount();

	// print info
	cout << "end>> landmarks:" << landmarks.size()
			<< " trials:" << trials.size() << endl;
	cout << "time:" << float(t2 - t1) / cv::getTickFrequency() << endl;

	// return running time
	return t2 - t1;
}

bool VisualOdometry::getRT(cv::Mat& R, cv::Mat& T) {
	if (state == VO_NORMAL) {
		graph.getPose(frameCount-1, R, T);
		return true;
	}
	return false;
}

void VisualOdometry::getDisplayFrame(cv::Mat& displayFrame) {
	featureAssoc->visualizeTrace(landmarks, trials);
	displayFrame = featureAssoc->getDisplayFrame();
}

// sychronize the landmarks position with the one in factor graph
void VisualOdometry::updateLandmark() {
	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		cv::Point3f p;
		graph.getLandMark(it->first, p);
		it->second.setLocation(p);
	}
}

void VisualOdometry::visualizeTraj(visualization_msgs::Marker& marker) {
	return graph.visualizeTraj(marker);
}

void VisualOdometry::visualizeLandmark(visualization_msgs::Marker& marker) {
	return graph.visualizeLandmark(marker);
}

// initialize a key frame
void VisualOdometry::triangulate(const cv::Mat& pts1,
		const cv::Mat& pts2,
		cv::Mat& outpts) {
	// prepare camera projection matrix
	cv::Mat camRT = identityRT();
	cv::Mat p1 = camera.intrinsic * camRT;
	camRT.at<float>(0, 3) -= camera.baseline;
	cv::Mat p2 = camera.intrinsic * camRT;

	// triangulate
	cv::triangulatePoints(p1, p2, pts1, pts2, outpts);

	// normalize
	outpts.row(0) = outpts.row(0) / outpts.row(3);
	outpts.row(1) = outpts.row(1) / outpts.row(3);
	outpts.row(2) = outpts.row(2) / outpts.row(3);
	outpts.row(3) = outpts.row(3) / outpts.row(3);
}

void VisualOdometry::confirmTrials() {
	// empty check
	if (trials.empty())
		return;
	// triangulate landmarks
	// prepare points
	cv::Mat pts1(2, trials.size(), cv::DataType<float>::type),
			pts2(2, trials.size(), cv::DataType<float>::type);
	int col = 0;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		cv::KeyPoint k1, k2;
		it->firstPointPair(k1, k2);
		pts1.at<float>(0, col) = k1.pt.x;
		pts1.at<float>(1, col) = k1.pt.y;
		pts2.at<float>(0, col) = k2.pt.x;
		pts2.at<float>(1, col) = k2.pt.y;
		col++;
	}
	cv::Mat pts3D;
	triangulate(pts1, pts2, pts3D);

	// convert to world coordinate
	int sframe = trials.back().getStartFrame();
	cv::Mat posR, posT;
	graph.getPose(sframe, posR, posT);
	cv::Mat posRT = fullRT(posR, posT);
	pts3D = posRT * pts3D;

	int colcount = 0;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		// add location to landmark data structure
		cv::Point3f p = cv::Point3f(pts3D.at<float>(0, colcount),
				pts3D.at<float>(1, colcount),
				pts3D.at<float>(2, colcount));
		it->setLocation(p);
		colcount++;

		// add initial value of land mark to factor graph data structure
		graph.addLandMark(landmarkID, p);

		// move from trail to true landmarks
		landmarks[landmarkID] = *it;

		// add factors
		int curframe = it->getStartFrame();
		for (int i = 0; i < it->getTraceSize(); i++) {
			cv::KeyPoint p1, p2;
			it->getPointPair(i, p1, p2);
			graph.addStereo(curframe, landmarkID, p1.pt, p2.pt);
			curframe++;
		}

		landmarkID++;
	}
	trials.clear();
}

