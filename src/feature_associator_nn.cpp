/*
 * feature_associator.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */
#include <slam_main/feature_associator_nn.h>
//std
#include <algorithm>
#include <iostream>
//opencv
#include <opencv2/highgui.hpp>

#define DEBUG 0

FeatureAssociatorNN::FeatureAssociatorNN() :
		searchRad(10), //10
		singleThre(50), doubleRatio(0.85),
		filter(376, 240),
		flann(new cv::flann::LshIndexParams(20, 10, 2)){

	//ptExtractor = cv::ORB::create(2000);
	ptExtractor = cv::ORB::create(200, 1.2f, 4, 0, 0, 2, cv::ORB::HARRIS_SCORE, 31);
	ptMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void FeatureAssociatorNN::processImage(
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

	//track all current landmark points
	auto cit = landmarks.begin();
	while (cit != landmarks.end()) {
		if (trackLandmark(cit->second, image1, image2, kpts1, kpts2,
				nnFinder1, nnFinder2, matchPts1, matchPts2)) {
			cit++;
		} else {
			cit->second.setEndFrame(frameNum);
			cit = landmarks.erase(cit);
		}
	}

	// track all potential landmark points
	auto tit = trials.begin();
	while (tit != trials.end()) {
		if (trackLandmark(*tit, image1, image2, kpts1, kpts2,
				nnFinder1, nnFinder2, matchPts1, matchPts2)) {
			tit++;
		} else {
			tit = trials.erase(tit);
		}
	}

	//compute unmatched points
	set<int> allMatchPts;
	//eliminate unMathced points which is too close to matched points
	for (auto it = matchPts1.begin(); it != matchPts1.end(); it++) {
		vector<int> nn;
		nnFinder1.findNN(kpts1[*it],
				searchRad / 2, searchRad / 2, searchRad / 2, searchRad / 2, nn);
		allMatchPts.insert(nn.begin(), nn.end());
	}
	// generate the unmatched points
	for (unsigned int npt = 0; npt < kpts1.size(); npt++) {
		if (allMatchPts.find(npt) == allMatchPts.end())
			unmatched.push_back(kpts1[npt]);
	}

}

void FeatureAssociatorNN::initTrials(
		const cv::Mat& image1,
		const cv::Mat& image2,
		int frameNum,
		vector<Landmark>& trials) {

	using namespace cv;
	this->displayFrame = image1.clone();

	// feature detection and description
	vector<KeyPoint> kpts1, kpts2;
	Mat descp1, descp2;
	evenlyDetect(image1, kpts1);
	evenlyDetect(image2, kpts2);
	ptExtractor->compute(image1, kpts1, descp1);
	ptExtractor->compute(image2, kpts2, descp2);

	matchAdd(kpts1,kpts2,descp1,descp2,frameNum, trials);
}

void FeatureAssociatorNN::refreshTrials(
		const cv::Mat& image1,
		const cv::Mat& image2,
		int frameNum,
		vector<cv::KeyPoint>& newPts,
		vector<Landmark>& trials) {

	using namespace cv;
	trials.clear();
	// compute descriptor and keypoints
	vector<KeyPoint> kpts2;
	Mat descp1, descp2;
	evenlyDetect(image2, kpts2);
	ptExtractor->compute(image1, newPts, descp1);
	ptExtractor->compute(image2, kpts2, descp2);

	matchAdd(newPts,kpts2,descp1,descp2,frameNum,trials);
}

bool FeatureAssociatorNN::trackLandmark(Landmark& landmark,
		const cv::Mat& image1,
		const cv::Mat& image2,
		const vector<cv::KeyPoint>& kpts1,
		const vector<cv::KeyPoint>& kpts2,
		const NNFinder& nnFinder1,
		const NNFinder& nnFinder2,
		set<int>& matchPts1,
		set<int>& matchPts2) {
	// input
	cv::KeyPoint point1, point2;
	landmark.curPointPair(point1, point2);
	// ouput
	cv::KeyPoint newPoint1, newPoint2;
	cv::Mat newDescp;
	int matchID1, matchID2;
	// do track
	if (trackPoint(point1, landmark.getDescp(), nnFinder1, kpts1, image1,
			newPoint1, newDescp, matchID1) &&
			trackPoint(point2, landmark.getDescp(), nnFinder2, kpts2, image2,
					newPoint2, newDescp, matchID2)) {

		landmark.appendPointPair(newPoint1, newPoint2);
		landmark.setDescp(newDescp);
		matchPts1.insert(matchID1);
		matchPts2.insert(matchID2);

		return true;
	}
	return false;
}

bool FeatureAssociatorNN::trackPoint(const cv::KeyPoint& point,
		const cv::Mat& descp,
		const NNFinder& nnFinder,
		const vector<cv::KeyPoint>& kpts,
		const cv::Mat& image,
		cv::KeyPoint& newPoint,
		cv::Mat& newDescp,
		int& matchID) {
	using namespace cv;
	// compute nearest neighbour
	vector<int> boundPtsInd;
	nnFinder.findNN(point, searchRad, searchRad, searchRad, searchRad, boundPtsInd);

	// compute descriptors for sub points
	vector<KeyPoint> subKpts;
	for (auto subit = boundPtsInd.begin(); subit != boundPtsInd.end(); subit++) {
		subKpts.push_back(kpts[*subit]);
#if DEBUG
		circle(displayFrame, kpts[*subit].pt, 1, BLUE);
#endif
	}
#if DEBUG
	//display the result
	circle(displayFrame, landmark.lastPoint().pt, 1, GREEN);
	imshow("SLAM", displayFrame);
	waitKey(0);
#endif

	Mat subDescps;
	ptExtractor->compute(image, subKpts, subDescps);
	// matching the points
	vector<vector<DMatch>> matches;
	if (subDescps.rows > 0)
		ptMatcher->knnMatch(descp, subDescps, matches, 2);

	// determine whether a good match
	if (!matches.empty() && matches[0][0].distance < singleThre &&
			(matches[0].size() == 1 || (matches[0].size() == 2 &&
					(matches[0][0].distance / matches[0][1].distance) < doubleRatio))) {

		newPoint = subKpts[matches[0][0].trainIdx];
		newDescp = subDescps.row(matches[0][0].trainIdx).clone();
		matchID = boundPtsInd[matches[0][0].trainIdx];

#if DEBUG
		cout << "success!" << "size:" << matches[0].size()
		<< "dist:" << matches[0][0].distance;
		if (matches[0].size() == 2) {
			cout << "ratio:" << matches[0][0].distance / matches[0][1].distance << endl;
		}
#endif
		return true;
	}
	return false;
}

void FeatureAssociatorNN::matchAdd(
		const vector<cv::KeyPoint>& kpts1,
		const vector<cv::KeyPoint>& kpts2,
		const cv::Mat& descp1,
		const cv::Mat& descp2,
		int frameNum,
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
		Landmark landmark(kpts1[it->queryIdx],kpts2[it->trainIdx], descp1.row(it->queryIdx).clone(), frameNum);
		trials.push_back(landmark);
	}

	std::cout << "pair detect:" << kpts1.size() << std::endl;
	std::cout << "flann match:" << goodmatches.size() << std::endl;
}

void FeatureAssociatorNN::evenlyDetect(
		const cv::Mat& frame,
		vector<cv::KeyPoint>& kpts) {

	using namespace cv;
	int size = 120;
	int numCol = frame.cols / size;
	int numRow = frame.rows / size;

	for (int c = 0; c < numCol; c++) {
		for (int r = 0; r < numRow; r++) {
			Mat sub = frame(Rect(c * size, r * size, size, size));
			vector<KeyPoint> subKpts;
			ptExtractor->detect(sub, subKpts);
			//cv::FAST(sub,subKpts,1);

			for (auto it = subKpts.begin(); it != subKpts.end(); it++) {
				//bias the keypoint position
				(it->pt.x) += c * size;
				(it->pt.y) += r * size;
				kpts.push_back(*it);
			}
		}
	}

#if DEBUG
// draw all key points
	for (auto d_it = kpts.begin(); d_it != kpts.end(); d_it++)
	circle(displayFrame, d_it->pt, 1, RED);
#endif
}

