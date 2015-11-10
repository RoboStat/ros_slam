#ifndef INCLUDE_SLAM_MAIN_LAND_MARK_H
#define INCLUDE_SLAM_MAIN_LAND_MARK_H

//std
#include <vector>
#include <utility>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

class Landmark {
public:
	Landmark();
	Landmark(const cv::KeyPoint& point, const cv::Mat& descp, int sFrame);

	void appendPoint(const cv::KeyPoint& point);
	cv::KeyPoint firstPoint() const;
	cv::KeyPoint lastPoint() const;
	cv::KeyPoint lastlastPoint() const;
	vector<cv::KeyPoint>::iterator pointBegin();
	vector<cv::KeyPoint>::iterator pointEnd();

	void setDescp(const cv::Mat& descp);
	cv::Mat getDescp() const;

	cv::Point3f getLocation() const;
	void setLocation(const cv::Point3f loc);

	cv::KeyPoint getPair() const;
	void setPair(const cv::KeyPoint& point);

	void setEndFrame(int frame);
	int getStartFrame() const;

	void visualizeTrace(cv::Mat& display, cv::Scalar color) const;

private:
	vector<cv::KeyPoint> trace;
	cv::Mat descriptor;
	cv::Point3f location;

	cv::KeyPoint pair;

	int startFrame;
	int endFrame;
};

#endif
