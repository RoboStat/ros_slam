#ifndef INCLUDE_SLAM_MAIN_LAND_MARK_H
#define INCLUDE_SLAM_MAIN_LAND_MARK_H

//std
#include <vector>
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

	cv::Point3d getLocation() const;
	void setLocation(const cv::Point3d loc);

	void setEndFrame(int frame);
	int getStartFrame() const;

	void visualizeTrace(cv::Mat& display, cv::Scalar color) const;

private:
	vector<cv::KeyPoint> trace;
	cv::Mat descriptor;
	cv::Point3d location;

	int startFrame;
	int endFrame;
};

#endif
