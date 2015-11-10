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
}

Landmark::Landmark(const cv::KeyPoint& point, const cv::Mat& descp,
		int sFrame) {
	trace.push_back(point);
	descriptor = descp;
	startFrame = sFrame;
	endFrame = -1;
}

void Landmark::appendPoint(const cv::KeyPoint& point) {
	trace.push_back(point);
}

cv::KeyPoint Landmark::firstPoint() const {
	return *(trace.begin());
}

cv::KeyPoint Landmark::lastPoint() const {
	return trace.back();
}

cv::KeyPoint Landmark::lastlastPoint() const {
	return *(trace.end()-1);
}

vector<cv::KeyPoint>::iterator Landmark::pointBegin() {
	return trace.begin();
}

vector<cv::KeyPoint>::iterator Landmark::pointEnd() {
	return trace.end();
}


void Landmark::setDescp(const cv::Mat& descp) {
	descriptor = descp;
}

cv::Mat Landmark::getDescp() const{
	return descriptor;
}

cv::Point3f Landmark::getLocation() const{
	return location;
}

void Landmark::setLocation(const cv::Point3f loc) {
	location = loc;
}

cv::KeyPoint Landmark::getPair() const {
	return pair;
}

void Landmark::setPair(const cv::KeyPoint& point) {
	pair=point;
}

void Landmark::setEndFrame(int frame) {
	endFrame = frame;
}

int Landmark::getStartFrame() const{
	return startFrame;
}

void Landmark::visualizeTrace(cv::Mat& display, cv::Scalar color) const{
	using namespace cv;
	KeyPoint lastPt = lastPoint();
	for (auto it = trace.begin();it!=trace.end(); it++) {
		line(display, lastPt.pt, it->pt, color);
		lastPt = *it;
	}
}
