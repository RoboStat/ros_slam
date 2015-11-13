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
//opencv
#include <opencv2/highgui.hpp>

#define DEBUG 0

const cv::Scalar RED(0,0,255);
const cv::Scalar BLUE(255,0,0);
const cv::Scalar GREEN(0,255,0);

FeatureAssociator::FeatureAssociator() :
				searchRad(15),
				disparityRad(30),
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

	// detect key points
	evenlyDetect(frame, kpts);
	//ptExtractor->detect(frame, kpts);

    //build NN Finder
	NNFinder nnFinder(kpts);

    //track all current landmark points
	auto cit = landmarks.begin();
	while (cit != landmarks.end()) {
		if (!trackLandmark(cit->second, nnFinder))
			cit = landmarks.erase(cit);
		else
			cit++;
	}

	// track all potential landmark points
	auto tit = trials.begin();
	while (tit != trials.end()) {
		if (!trackLandmark(*tit, nnFinder))
			tit = trials.erase(tit);
		else
			tit++;
	}

	//compute unmatched points
	set<int> allMatchPts;
	//eliminate unMathced points which is too close to matched points
	for (auto it = matchPts.begin(); it != matchPts.end(); it++) {
		vector<int> nn;
		nnFinder.findNN(kpts[*it],
				searchRad/3, searchRad/3, searchRad/3, searchRad/3, nn)
		allMatchPts.insert(nn.begin(), nn.end());
	}
	// generate the unmatched points
	for (unsigned int npt = 0; npt < kpts.size(); npt++) {
		if (allMatchPts.find(npt) == allMatchPts.end())
			unmatched.push_back(kpts[npt]);
	}

}

void FeatureAssociator::initTrials(
		const cv::Mat& image1,
		const cv::Mat& image2,
		int frameNum,
		vector<Landmark>& trials) {

	using namespace cv;
	this->frame = image2;
	this->displayFrame = image1.clone();
	this->frameNum = frameNum;
	this->matchPts.clear();
	this->kpts.clear();

	// feature detection and description
	vector<KeyPoint> kpts1, kpts2;
	Mat descp1;
	evenlyDetect(image1, kpts1);
	evenlyDetect(image2, kpts2);
	KeyPointsFilter::retainBest(kpts1, 600);
	ptExtractor->compute(image1, kpts1, descp1);
	//ptExtractor->detectAndCompute(image, noArray(), initKpts, initDescp);

	//try to match to the right frame
	this->kpts = kpts2;
	NNFinder nnFinder(kpts);
	int rowCount=0;
	for(auto it=kpts1.begin(); it!=kpts1.end(); it++) {
		Landmark landmark(*it, descp1.row(rowCount++).clone(),frameNum);
		if(pairLandmark(landmark, nnFinder))
			trials.push_back(landmark);
	}
}

void FeatureAssociator::refreshTrials(
		const cv::Mat& image2,
		vector<cv::KeyPoint>& newPts,
		vector<Landmark>& trials) {

	using namespace cv;
	trials.clear();
	// compute descriptor on the left frame
	Mat descps;
	KeyPointsFilter::retainBest(newPts, 600);
	ptExtractor->compute(frame,newPts,descps);

	// now focus on right frame
	this->frame = image2;
	this->matchPts.clear();
	this->kpts.clear();
	evenlyDetect(image2, kpts);
	NNFinder nnFinder(kpts);

	// try to match to right frame
	int rowCount = 0;
	for (auto it = newPts.begin(); it != newPts.end(); it++) {
		Landmark landmark(*it, descps.row(rowCount++).clone(), frameNum);
		if(pairLandmark(landmark))
			trials.push_back(landmark);
	}
}

bool FeatureAssociator::trackLandmark(Landmark& landmark, const NNFinder& nnFinder) {
	using namespace cv;
	// compute nearest neighbour
	vector<int> boundPtsInd;
	nnFinder.findNN(landmark.lastPoint(),
			searchRad, searchRad, searchRad, searchRad, boundPtsInd);

	// compute descriptors for sub points
	vector<KeyPoint> subKpts;
	for (auto subit = boundPtsInd.begin(); subit != boundPts.end(); subit++) {
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

bool FeatureAssociator::pairLandmark(Landmark& landmark, const NNFinder& nnFinder) {
	using namespace cv;
	// compute nearest neighbour
	vector<int> boundPtsInd;
	nnFinder.findNN(landmark.firstPoint(),
			disparityRad, -5, 3, 3, boundPtsInd);

	// compute descriptors for sub points
	vector<KeyPoint> subKpts;
	for (auto subit = boundPtsInd.begin(); subit != boundPts.end(); subit++) {
		subKpts.push_back(kpts[*subit]);
#if DEBUG
		circle(displayFrame, kpts[*subit].pt, 1, BLUE);
#endif
	}
#if DEBUG
	//display the result
	circle(displayFrame, landmark.firstPoint().pt, 1, GREEN);
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
			((matches[0].size() == 1 && matches[0][0].distance < singleThre-10)
					|| (matches[0].size() == 2 && (matches[0][0].distance / matches[0][1].distance) < doubleRatio-0.1))) {

		landmark.setPair(subKpts[matches[0][0].trainIdx]);
		return true;
	}
	return false;
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

#if DEBUG
	// draw all key points
	for (auto d_it = kpts.begin(); d_it != kpts.end(); d_it++)
	circle(displayFrame, d_it->pt, 1, RED);
#endif
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

void FeatureAssociator::visualizePair(const vector<Landmark>& trials){
	using namespace cv;
	for(auto it=trials.begin(); it!=trials.end(); it++) {
		line(displayFrame,it->firstPoint().pt,it->getPair().pt,Scalar(0,0,255));
	}
}

cv::Mat FeatureAssociator::getDisplayFrame() {
	return displayFrame;
}
