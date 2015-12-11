/*
 * feature_associator_nn2.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: wenda
 */
#include <slam_main/feature_associator_nn2.h>
#include <slam_main/helper.h>
// opencv
#include <opencv2/calib3d.hpp>

// Implement the two round match
void FeatureAssociatorNN2::processImage(
		const cv::Mat& image1,
		const cv::Mat& image2,
		int frameNum,
		map<int, Landmark>& landmarks,
		vector<Landmark>& trials,
		vector<cv::KeyPoint>& unmatched) {

	using namespace cv;

	this->displayFrame = image1.clone();
	unmatched.clear();

	// detect key points
	vector<cv::KeyPoint> kpts1, kpts2;
	evenlyDetect(image1, kpts1);
	evenlyDetect(image2, kpts2);

	//build NN Finder
	NNFinder nnFinder1(kpts1);
	NNFinder nnFinder2(kpts2);

	//mathced points
	set<int> matchPts1, matchPts2;

	// simple tracking for trials, when there is no landmark
	if (landmarks.empty()) {
		auto tit = trials.begin();
		while (tit != trials.end()) {
			if (trackLandmark(*tit, image1, image2, kpts1, kpts2,
					nnFinder1, nnFinder2, matchPts1, matchPts2)) {
				tit++;
			} else {
				tit = trials.erase(tit);
			}
		}
	// two round tracking when there is landmarks
	} else {
		//-----------triangulation--------------
		// prepare camera projection matrix
		cv::Mat camRT = identityRT();
		cv::Mat p1 = camera.intrinsic * camRT;
		camRT.at<float>(0, 3) -= camera.baseline;
		cv::Mat p2 = camera.intrinsic * camRT;

		// prepare points
		vector<cv::Point2f> tpts1, tpts2;
		for (auto &landmark : landmarks) {
			// prepare left right pair for real landmark
			cv::KeyPoint point1, point2;
			landmark.second.curPointPair(point1, point2);
			tpts1.push_back(point1.pt);
			tpts2.push_back(point2.pt);
		}
		// triangulate
		cv::Mat toutpts;
		cv::triangulatePoints(p1, p2, tpts1, tpts2, toutpts);

		// normalize
		toutpts.row(0) = toutpts.row(0) / toutpts.row(3);
		toutpts.row(1) = toutpts.row(1) / toutpts.row(3);
		toutpts.row(2) = toutpts.row(2) / toutpts.row(3);
		toutpts.row(3) = toutpts.row(3) / toutpts.row(3);

		//-----------first round tracking only in left---------
		vector<cv::Point2f> imgPts;
		vector<cv::Point3f> objPts;
		int colcount = 0;
		for (auto &landmark : landmarks) {
			// input
			cv::KeyPoint point1 = landmark.second.curLeftPoint();
			// ouput
			cv::KeyPoint newPoint1;
			cv::Mat newDescp1;
			int matchID1;
			// do track
			if (trackPoint(point1.pt, landmark.second.getLeftDescp(), nnFinder1, kpts1, image1,
					newPoint1, newDescp1, matchID1,searchRad, singleThre, doubleRatio)) {
				// prepare 2d 3d pair
				imgPts.push_back(newPoint1.pt);
				objPts.push_back(cv::Point3f(toutpts.at<float>(0, colcount),
						toutpts.at<float>(1, colcount),
						toutpts.at<float>(2, colcount)));
			}
			colcount++;
		}
		std::cout<<"NN2-first round match:"<<imgPts.size()<<std::endl;

		//--------- pose estimate via pnp------------
		cv::Mat R, Rvec, Tvec, inliers;
		cv::solvePnPRansac(objPts, imgPts, camera.intrinsic,
				cv::noArray(), Rvec, Tvec, false, 3000, 1.0, 0.99, inliers, cv::SOLVEPNP_P3P);
		cv::Rodrigues(Rvec, R);
		//std::cout<< R << Tvec <<endl;

		//--------- predict landmark in current frame ---------
		colcount = 0;
		cv::Mat cp1 = p1 * fullRT(R, Tvec);
		cv::Mat cp2 = p2 * fullRT(R, Tvec);
		for (auto &landmark : landmarks) {
			// reprojection
			cv::Mat leftProj = cp1 * toutpts.col(colcount);
			cv::Mat rightProj = cp2 * toutpts.col(colcount);
			cv::Point2f leftpt(leftProj.at<float>(0) / leftProj.at<float>(2),
					leftProj.at<float>(1) / leftProj.at<float>(2));
			cv::Point2f rightpt(rightProj.at<float>(0) / rightProj.at<float>(2),
					rightProj.at<float>(1) / rightProj.at<float>(2));

			landmark.second.setPredPair(leftpt, rightpt);
			//cv::KeyPoint k1,k2;
			//landmark.second.curPointPair(k1,k2);
			colcount++;
		}

		//-------- second round tracking for landmarks ------------
		auto cit = landmarks.begin();
		while (cit != landmarks.end()) {
			if (trackLandmarkByPred(cit->second, image1, image2, kpts1, kpts2,
					nnFinder1, nnFinder2, matchPts1, matchPts2)) {
				cit++;
			} else {
				cit->second.setEndFrame(frameNum);
				cit = landmarks.erase(cit);
			}
		}

		// ---------- second round tracking for trials -------------
		if (!trials.empty()) {
			//--------- fisrt triangulate ---------
			vector<cv::Point2f> trpts1, trpts2;
			for (auto &landmark : trials) {
				// prepare left right pair for trials
				cv::KeyPoint point1, point2;
				landmark.curPointPair(point1, point2);
				trpts1.push_back(point1.pt);
				trpts2.push_back(point2.pt);
			}
			cv::Mat troutpts;
			cv::triangulatePoints(p1, p2, trpts1, trpts2, troutpts);
			// normalize
			troutpts.row(0) = troutpts.row(0) / troutpts.row(3);
			troutpts.row(1) = troutpts.row(1) / troutpts.row(3);
			troutpts.row(2) = troutpts.row(2) / troutpts.row(3);
			troutpts.row(3) = troutpts.row(3) / troutpts.row(3);

			//--------- predict location ------------
			colcount = 0;
			for (auto &landmark : trials) {
				// reprojection
				cv::Mat leftProj = cp1 * troutpts.col(colcount);
				cv::Mat rightProj = cp2 * troutpts.col(colcount);
				cv::Point2f leftpt(leftProj.at<float>(0) / leftProj.at<float>(2),
						leftProj.at<float>(1) / leftProj.at<float>(2));
				cv::Point2f rightpt(rightProj.at<float>(0) / rightProj.at<float>(2),
						rightProj.at<float>(1) / rightProj.at<float>(2));

				landmark.setPredPair(leftpt, rightpt);
				colcount++;
			}

			//--------- do the tracking -------------
			auto tit = trials.begin();
			while (tit != trials.end()) {
				if (trackLandmarkByPred(*tit, image1, image2, kpts1, kpts2,
						nnFinder1, nnFinder2, matchPts1, matchPts2)) {
					tit++;
				} else {
					tit = trials.erase(tit);
				}
			}
		}
	}

	//compute unmatched points
	set<int> allMatchPts;
	//eliminate unMathced points which is too close to matched points
	for (auto it = matchPts1.begin(); it != matchPts1.end(); it++) {
		vector<int> nn;
		nnFinder1.findNN(kpts1[*it].pt,
				eliminateRad, eliminateRad, eliminateRad, eliminateRad, nn);
		allMatchPts.insert(nn.begin(), nn.end());
	}
	// generate the unmatched points
	for (unsigned int npt = 0; npt < kpts1.size(); npt++) {
		if (allMatchPts.find(npt) == allMatchPts.end())
			unmatched.push_back(kpts1[npt]);
	}

}

// Track land mark based on predicted location
bool FeatureAssociatorNN2::trackLandmarkByPred(Landmark& landmark,
		const cv::Mat& image1,
		const cv::Mat& image2,
		const vector<cv::KeyPoint>& kpts1,
		const vector<cv::KeyPoint>& kpts2,
		const NNFinder& nnFinder1,
		const NNFinder& nnFinder2,
		set<int>& matchPts1,
		set<int>& matchPts2) {
	// input
	cv::Point2f point1, point2;
	landmark.getPredPair(point1, point2);
	// ouput
	cv::KeyPoint newPoint1, newPoint2;
	cv::Mat newDescp1, newDescp2;
	int matchID1, matchID2;
	// do track
	// update threshold
	if (trackPoint(point1, landmark.getLeftDescp(), nnFinder1, kpts1, image1,
			newPoint1, newDescp1, matchID1,searchRad2, singleThre2, doubleRatio2) &&
			trackPoint(point2, landmark.getRightDescp(), nnFinder2, kpts2, image2,
					newPoint2, newDescp2, matchID2,searchRad2, singleThre2, doubleRatio2)) {

		landmark.appendPointPair(newPoint1, newPoint2);
		landmark.setDescpPair(newDescp1, newDescp2);
		matchPts1.insert(matchID1);
		matchPts2.insert(matchID2);

		return true;
	}
	return false;
}
