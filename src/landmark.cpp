/*
 * landmark.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: wenda
 */
#include <slam_main/landmark.h>
//std
#include <iostream>
///opencv
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

Landmark::Landmark() {
	startFrame = -1;
	endFrame = -1;
	isinlier = false;
}

Landmark::Landmark(const cv::KeyPoint& point1, const cv::KeyPoint& point2
		, const cv::Mat& descp1, const cv::Mat& descp2, int sFrame) {
	traceLeft.push_back(point1);
	traceRight.push_back(point2);
	descpLeft = descp1;
	descpRight = descp2;
	startFrame = sFrame;
	endFrame = -1;
	isinlier = false;
}

void Landmark::appendPointPair(const cv::KeyPoint& point1, const cv::KeyPoint& point2){
	traceLeft.push_back(point1);
	traceRight.push_back(point2);
}

void Landmark::firstPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const {
	point1 = *(traceLeft.begin());
	point2 = *(traceRight.begin());
}

void Landmark::prevPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const {
	point1 = *(traceLeft.end()-2);
	point2 = *(traceRight.end()-2);
}

void Landmark::curPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const {
	point1 = traceLeft.back();
	point2 = traceRight.back();
}

void Landmark::getPointPair(int index, cv::KeyPoint& point1, cv::KeyPoint& point2) const {
	point1 = traceLeft[index];
	point2 = traceRight[index];
}

cv::KeyPoint Landmark::curLeftPoint() const {
	return traceLeft.back();
}

int Landmark::getTraceSize() const {
	return traceLeft.size();
}


void Landmark::setDescpPair(const cv::Mat& descp1, const cv::Mat& descp2) {
	descpLeft = descp1;
	descpRight = descp2;
}

cv::Mat Landmark::getLeftDescp() const{
	return descpLeft;
}

cv::Mat Landmark::getRightDescp() const{
	return descpRight;
}

cv::Point3f Landmark::getLocation() const{
	return location;
}

void Landmark::setLocation(const cv::Point3f loc) {
	location = loc;
}

void Landmark::setEndFrame(int frame) {
	endFrame = frame;
}

int Landmark::getStartFrame() const{
	return startFrame;
}

void Landmark::setInlier(bool val) {
	isinlier = val;
}

bool Landmark::isInlier() const{
	return isinlier;
}

void Landmark::visualizeTrace(cv::Mat& display, cv::Scalar color) const{
	using namespace cv;
	//visulize last n points in trace
	int count=0;
	Point2f lastPt =curLeftPoint().pt;
	circle(display,lastPt,1,Scalar(0,255,0));
	for (auto it = traceLeft.end()-1;it!=traceLeft.begin()-1; it--) {
		line(display, lastPt, it->pt, color);
		lastPt = it->pt;
		if(count++ > 6)
			break;
	}
}
