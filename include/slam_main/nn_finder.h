#ifndef INCLUDE_SLAM_NN_FINDER_H_
#define INCLUDE_SLAM_NN_FINDER_H_

//std
#include <vector>
#include <map>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

class NNFinder {
public:
	NNFinder();
	void build(const std::vector<cv::KeyPoint>& points);
	void findNN(const cv::KeyPoint& point,
				float xl, float xh, float yl, float yh,
				std::vector<int>& output) const;

private:
	std::multimap<float, int> xind;
	std::multimap<float, int> yind;
};


#endif INCLUDE_SLAM_NN_FINDER_H_