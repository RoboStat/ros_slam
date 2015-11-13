#include <slam_main/nn_finder.h>
//std
#include <set>

NNFinder::NNFinder(const std::vector<cv::KeyPoint>& points) {
	build(points);
}

void NNFinder::build(const std::vector<cv::KeyPoint>& points) {
	
	using namespace std;
	xind.clear();
	yind.clear();
	
	int kptsCount = 0;
	for (auto it = points.begin(); it != points.end(); it++) {
		xind.insert(make_pair(it->pt.x, kptsCount));
		yind.insert(make_pair(it->pt.y, kptsCount));
		kptsCount++;
	}
}

void NNFinder::findNN(const cv::KeyPoint& point,
					  float xl, float xh, float yl, float yh,
					  std::vector<int>& output) const{

	using namespace cv;
	using namespace std;

	Point2f pt = point.pt;
	multimap<float, int>::const_iterator itl, ith;

	// filter in x range
	set<int> xset;
	itl = xind.lower_bound(pt.x - xl);
	ith = xind.upper_bound(pt.x + xh);
	for (; itl != ith; itl++) {
		xset.insert(itl->second);
	}

	// filter in y range
	itl = yind.lower_bound(pt.y - yl);
	ith = yind.upper_bound(pt.y + yh);
	for (; itl != ith; itl++) {
		if(xset.find(itl->second)!=xset.end()) {
			output.push_back(itl->second);
		}
	}
}
