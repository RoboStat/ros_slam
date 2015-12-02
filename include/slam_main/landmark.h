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
	Landmark(const cv::KeyPoint& point1, const cv::KeyPoint& point2,
			const cv::Mat& descp1, const cv::Mat& descp2, int sFrame);

	void appendPointPair(const cv::KeyPoint& point1, const cv::KeyPoint& point2);
	void firstPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const;
	void prevPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const;
	void curPointPair(cv::KeyPoint& point1, cv::KeyPoint& point2) const;
	void getPointPair(int index, cv::KeyPoint& point1, cv::KeyPoint& point2) const;
	cv::KeyPoint curLeftPoint() const;
	int getTraceSize() const;

	void setDescpPair(const cv::Mat& descp1, const cv::Mat& descp2);
	cv::Mat getLeftDescp() const;
	cv::Mat getRightDescp() const;

	cv::Point3f getLocation() const;
	void setLocation(const cv::Point3f loc);

	void setEndFrame(int frame);
	int getStartFrame() const;

	void setInlier(bool val);
	bool isInlier() const;

	void visualizeTrace(cv::Mat& display, cv::Scalar color) const;

private:
	// the sequence of tracked location in previous images
	vector<cv::KeyPoint> traceLeft;
	vector<cv::KeyPoint> traceRight;
	// latest descriptor calculated from image
	cv::Mat descpLeft;
	cv::Mat descpRight;
	// its 3D location in world coordinate
	cv::Point3f location;
	// is classified as inlier during
	bool isinlier;

	int startFrame;
	int endFrame;
};

#endif
