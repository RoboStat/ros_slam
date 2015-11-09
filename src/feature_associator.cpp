/*
 * feature_associator.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */

#include <slam_main/feature_associator.h>
//std
#include <algorithm>
#include <iostream>

#define DEBUG 0

FeatureAssociator::FeatureAssociator() :
		searchRad(15),
				singleThre(50), doubleRatio(0.8),
				frameNum(0) {

	//ptExtractor = cv::ORB::create(500);
	ptExtractor = cv::ORB::create(300, 1.2f, 8, 0, 0, 2, cv::ORB::HARRIS_SCORE, 31);
	ptMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void FeatureAssociator::processImage(
		const cv::Mat& image,
		int frameNum,
		map<int, Landmark>& landmarks,
		vector<Landmark>& trials,
		vector<cv::KeyPoint>& unmatched) {

	using namespace cv;

	this->frame = image;
	this->frameNum = frameNum;
	this->displayFrame = frame.clone();
	this->kpts.clear();
	this->matchPts.clear();
	unmatched.clear();
	evenlyDetect(frame, kpts);
	//ptExtractor->detect(frame, kpts);

//#if DEBUG
	Scalar RED(0, 0, 255);
	Scalar BLUE(255, 0, 0);
	Scalar GREEN(0, 255, 0);
	// draw all key points
	for (auto d_it = kpts.begin(); d_it != kpts.end(); d_it++)
		circle(displayFrame, d_it->pt, 1, RED);
//#endif

// build index map
	xind.clear();
	yind.clear();
	int kptsCount = 0;
	for (auto it = kpts.begin(); it != kpts.end(); it++) {
		xind.insert(make_pair(it->pt.x, kptsCount));
		yind.insert(make_pair(it->pt.y, kptsCount));
		kptsCount++;
	}

	// track all current landmark points
	auto cit = landmarks.begin();
	while (cit != landmarks.end()) {
		if (!trackLandmark(cit->second))
			landmarks.erase(cit++);
		else
			cit++;
	}

	// track all potential landmark points
	auto tit = trials.begin();
	while (tit != trials.end()) {
		if (!trackLandmark(*tit))
			tit = trials.erase(tit);
		else
			tit++;
	}

	//compute unmatched points
	set<int> allMatchPts;
	for (auto it = matchPts.begin(); it != matchPts.end(); it++) {
		vector<int> nn;
		vector<int>::iterator nnend = findNN(kpts[*it], searchRad/3, nn);
		allMatchPts.insert(nn.begin(),nnend);
	}

	for (unsigned int npt = 0; npt < kpts.size(); npt++) {
		if (allMatchPts.find(npt) == allMatchPts.end())
			unmatched.push_back(kpts[npt]);
	}

}

bool FeatureAssociator::trackLandmark(Landmark& landmark) {
	using namespace cv;
	// compute nearest neighbour
	vector<int> boundPtsInd;
	vector<int>::iterator ptsEnd=findNN(landmark.lastPoint(), searchRad,boundPtsInd);

	// compute descriptors for sub points
	vector<KeyPoint> subKpts;
	for (auto subit = boundPtsInd.begin(); subit != ptsEnd; subit++) {
		subKpts.push_back(kpts[*subit]);
#if DEBUG
		circle(displayFrame, kpts[*subit].pt, 1, BLUE);
#endif
	}
#if DEBUG
	//display the result
	circle(displayFrame, pt, 1, GREEN);
	imshow("SLAM", displayFrame);
	waitKey(0);
#endif

	Mat subDescps;
	Mat descp = landmark.getDescp();
	ptExtractor->compute(frame, subKpts, subDescps);
	// matching the points
	vector<vector<DMatch>> matches;
	if (subDescps.rows > 0)
		ptMatcher->knnMatch(descp, subDescps, matches, 2);

	// determine whether a good match
	if (!matches.empty() &&
			((matches[0].size() == 1 && matches[0][0].distance < singleThre)
					|| (matches[0].size() == 2 && (matches[0][0].distance / matches[0][1].distance) < doubleRatio))) {

		//record matched point
		matchPts.insert(boundPtsInd[matches[0][0].trainIdx]);
		// successful to track update landmark
		landmark.appendPoint(subKpts[matches[0][0].trainIdx]);
		landmark.setDescp(subDescps.row(matches[0][0].trainIdx).clone());

#if DEBUG
		cout << "success!" << "size:" << matches[0].size()
		<< "dist:" << matches[0][0].distance;
		if (matches[0].size() == 2) {
			cout << "ratio:" << matches[0][0].distance / matches[0][1].distance << endl;
		}
#endif
		return true;
	}
	landmark.setEndFrame(frameNum);
	return false;
}

vector<int>::iterator FeatureAssociator::findNN(const cv::KeyPoint& point, float rad, vector<int>& nn) {
	using namespace cv;
	Point2f pt = point.pt;
	map<float, int>::iterator itl, ith;
	set<int> xset, yset;
	// filter in x range
	itl = xind.lower_bound(pt.x - rad);
	ith = xind.upper_bound(pt.x + rad);
	for (; itl != ith; itl++) {
		xset.insert(itl->second);
	}
	// filter in y range
	itl = yind.lower_bound(pt.y - rad);
	ith = yind.upper_bound(pt.y + rad);
	for (; itl != ith; itl++) {
		yset.insert(itl->second);
	}
	// intersect both set
	nn.resize(xset.size());
	return set_intersection(xset.begin(), xset.end(), yset.begin(), yset.end(), nn.begin());
}

void FeatureAssociator::evenlyDetect(
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

			for (auto it = subKpts.begin(); it != subKpts.end(); it++) {
				//bias the keypoint position
				(it->pt.x) += c * size;
				(it->pt.y) += r * size;
				kpts.push_back(*it);
			}
		}
	}
}

void FeatureAssociator::initTrials(
		const cv::Mat& image,
		int frameNum,
		vector<Landmark>& trials) {

	using namespace cv;
	this->frame = image;
	this->displayFrame = image.clone();
	this->frameNum = frameNum;
	vector<KeyPoint> initKpts;
	Mat initDescp;
	evenlyDetect(frame, initKpts);
	KeyPointsFilter::retainBest(initKpts, 600);
	ptExtractor->compute(frame, initKpts, initDescp);
	//ptExtractor->detectAndCompute(image, noArray(), initKpts, initDescp);

	int rowCount = 0;
	for (auto it = initKpts.begin(); it != initKpts.end(); it++) {
		trials.push_back(Landmark(*it, initDescp.row(rowCount).clone(), frameNum));
		rowCount++;
	}
}

void FeatureAssociator::refreshTrials(
		vector<cv::KeyPoint>& newPts,
		vector<Landmark>& trials) {

	using namespace cv;
	trials.clear();
	Mat descps;
	KeyPointsFilter::retainBest(newPts, 400);
	ptExtractor->compute(frame, newPts, descps);
	int rowCount = 0;
	for (auto it = newPts.begin(); it != newPts.end(); it++) {
		trials.push_back(Landmark(*it, descps.row(rowCount++).clone(), frameNum));
	}
}

void FeatureAssociator::visualizeTrace(
		const map<int, Landmark>& landmarks,
		const vector<Landmark>& trials) {

	using namespace cv;

	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		it->second.visualizeTrace(displayFrame, Scalar(255, 0, 0));
	}

	for (auto it = trials.begin(); it != trials.end(); it++) {
		it->visualizeTrace(displayFrame, Scalar(0, 255, 0));
	}
}

cv::Mat FeatureAssociator::getDisplayFrame() {
	return displayFrame;
}
