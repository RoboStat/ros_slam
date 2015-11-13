#ifndef INCLUDE_SLAM_MAIN_KEYPOINT_FILTER_H_
#define INCLUDE_SLAM_MAIN_KEYPOINT_FILTER_H_
//Implementation of SUPPRESSION VIA DISK COVERING
//Following paper EFFICIENTLY SELECTING SPATIALLY DISTRIBUTED KEYPOINTS FOR VISUAL TRACKING

//std
#include <vector>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

class KeyPointFilter {
public:
	KeyPointFilter(int width, int height);
	~KeyPointFilter();

	// Non max suppresion based on fixed radius size
	// The number of return points may varies
	// Not excat solution
	void filterByRadius(int radius, 
						const std::vector<cv::KeyPoint>& input,
						std::vector<cv::KeyPoint>& filtered);

private:
	// reset the used bitmap
	void reset();
	// check whether the location is already covered
	bool isOccupied(const cv::Point2f& point);
	// cover a fixed window around the keypoint
	void coverAround(const cv::Point2f& point, int rad);

	void getIndex(const cv::Point2f& point, int& row, int& col);
	bool isSet(int row, int col);
	void set(int row, int col, int rad);

	void setMap(int sInd, int eInd);
	void setByte(char* byte, int s, int e);

	char* bitmap;
	int cellWidth;
	int numOfRows;
	int numOfCols;
};

#endif INCLUDE_SLAM_MAIN_KEYPOINT_FILTER_H_