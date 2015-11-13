#include <slam_main/keypoint_filter.h>
//std
#include <math.h>
#include <algorithm>

KeyPointFilter::KeyPointFilter(int width, int height) {
	cellWidth = 2;
	numOfRows = ceil(((float)height)/cellWidth);
	numOfCols = ceil(((float)width)/cellWidth);
	reset();
}

KeyPointFilter::~KeyPointFilter() {
	delete[] bitmap;
}

void KeyPointFilter::filterByRadius(int radius,
						const std::vector<cv::KeyPoint>& input,
						std::vector<cv::KeyPoint>& filtered) {
	// clear the old bitmap
	reset();
	filtered.clear();

	// sort points by response

	// eliminating by covering
	for(auto it=input.begin(); it!=input.end(); it++)  {
		if(!isOccupied(it->pt)) {
			coverAround(it->pt,radius);
			filtered.push_back(*it);
		}
	}
}

void KeyPointFilter::reset() {
	delete[] bitmap;
	int mapSize = ceil(numOfRows * numOfCols / 8.0);
	bitmap = new char[mapSize]();
}

bool KeyPointFilter::isOccupied(const cv::Point2f& point) {
	int row, col;
	getIndex(point, row, col);
	return isSet(row, col);
}

void KeyPointFilter::coverAround(const cv::Point2f& point, int rad) {
	int row, col;
	getIndex(point, row, col);
	set(row, col, rad);
}

void KeyPointFilter::getIndex(const cv::Point2f& point, int& row, int& col) {
	row = point.y / cellWidth;
	col = point.x / cellWidth;
}

bool KeyPointFilter::isSet(int row, int col) {
	int ind = row*numOfCols + col;
	char by = bitmap[ind/8];
	char t = pow(2, 7-ind%8);
	return ((unsigned int)(by & t) > 0);
}

void KeyPointFilter::set(int row, int col, int rad) {
	for(int curR=row-rad; curR<=row+rad; curR++) {
		int offSet = curR*numOfCols + col;
		setMap(offSet - rad, offSet + rad + 1);
	}
}

void KeyPointFilter::setMap(int sInd, int eInd) {
	int sb = sInd/8;
	int so = sInd%8;
	int eb = eInd/8;
	int eo = eInd%8;

	if(sb == eb)
		setByte(bitmap+sb, so, eo);
	else {
		setByte(bitmap+sb, so, 8);
		sb++;
		while(sb<eb) {
			setByte(bitmap+sb, 0, 8);
			sb++;
		}
		setByte(bitmap+eb, 0, eo);
	}
}

void KeyPointFilter::setByte(char* byte, int s, int e) {
	char right = pow(2,8-s)-1;
	char left = pow(2,8-e)-1;

	*byte = *byte | (right & (~left));
}
