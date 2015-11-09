#include <slam_main/feature_associator.h>
#include <slam_main/factor_graph.h>
#include <slam_main/camera.h>
//std
#include <iostream>
//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

#define DEBUG 0

string fixedNum(int value, int digits = 3) {
	std::string result;
	while (digits-- > 0) {
		result += ('0' + value % 10);
		value /= 10;
	}
	std::reverse(result.begin(), result.end());
	return result;
}

// camera info
Camera camera;

// all the tracked landmarks
int landmarkID = 0;
map<int, Landmark> landmarks;
vector<Landmark> trials;
vector<cv::KeyPoint> unmatched;

// features
unsigned int landmarkThre = 100;
FeatureAssociator featureAssoc;

// factor graph
FactorGraph graph(camera);

// sychronize the landmarks position with the one in factor graph
void updateLandmark(){
	for(auto it=landmarks.begin(); it!=landmarks.end(); it++) {
		cv::Point3d p;
		graph.getLandMark(it->first,p);
		it->second.setLocation(p);
	}
}

// initialize a key frame
void initKeyFrame(int frameNum) {
	// triangulate trial points
	int sframe = trials.back().getStartFrame();
	int eframe = frameNum;
	cv::Mat sr,st,er,et;
	graph.getPose(sframe,sr,st);
	graph.getPose(eframe,er,et);
	cv::Mat sp,ep,sh,eh;
	cv::hconcat(sr,st,sh);
	cv::hconcat(er,et,eh);

	sp = camera.intrinsic * sh;
	ep = camera.intrinsic * eh;

	vector<cv::Point2f> spts, epts;
	for(auto it=trials.begin(); it!=trials.end(); it++) {
		spts.push_back(it->firstPoint().pt);
		epts.push_back(it->lastPoint().pt);
	}

	cv::Mat outpts;
	std::cout<<sp<<endl;
	std::cout<<ep<<endl;
	std::cout<<spts<<endl;
	std::cout<<epts<<endl;
	cv::triangulatePoints(sp,ep,spts,epts,outpts);

	int colcount=0;
	for(auto it=trials.begin(); it!=trials.end(); it++){
		cv::Point3d p = cv::Point3d(outpts.at<double>(0,colcount),
									outpts.at<double>(1,colcount),
									outpts.at<double>(2,colcount));
		// set land mark 3d location
		it->setLocation(p);
		// add initial value of land mark to factor graph
		graph.addLandMark(landmarkID,p);
		// move from trail to true landmarks
		landmarks[landmarkID]=*it;
		// add factors
		int curframe=sframe;
		for(auto itt=it->pointBegin(); itt!=it->pointEnd(); itt++){
			graph.addProjection(curframe,landmarkID,itt->pt);
			curframe++;
		}
		colcount++;
		landmarkID++;
	}

	// run bundle adjustment
	graph.update();

	// update landmark position
	updateLandmark();

	// start new trials
	featureAssoc.refreshTrials(unmatched, trials);
}

//////////////////////main///////////////////////////
int main() {
	cv::namedWindow("SLAM", cv::WINDOW_NORMAL);

	int startFrame = 100;
	for (int i = startFrame; i < 634; i++) {
		//read image
		string path = "/home/wenda/Developer/SLAM/";
		cv::Mat left_frame = cv::imread(path + "resource/left" + fixedNum(i) + ".jpg");
		cv::Mat right_frame = cv::imread(path + "resource/right" + fixedNum(i) + ".jpg");

		//track points
		int t1 = cv::getTickCount();
		// fist frame
		if (i == startFrame) {
			featureAssoc.initTrials(left_frame,i, trials);
			graph.addFirstPose(i);
		} else {
			// boot strap
			if (landmarks.empty()) {
				featureAssoc.processImage(left_frame, i, landmarks, trials, unmatched);

				// 2D-2D VO
				vector<cv::Point2f> pta, ptb;
				for(auto it=trials.begin(); it!=trials.end(); it++) {
					pta.push_back(it->lastlastPoint().pt);
					ptb.push_back(it->lastPoint().pt);
				}

				cv::Mat E = cv::findEssentialMat(ptb,pta,
										camera.focalLength,camera.principalPoint,
										cv::RANSAC, 0.99, 1.0);
				cv::Mat R,T;
				cv::recoverPose(E,ptb,pta,R,T,camera.focalLength,camera.principalPoint);

				// insert to graph
				graph.advancePose(i,R,T);

				if (trials.size() < 2 * landmarkThre) {
					initKeyFrame(i);
				}
			}
			// normal state
			else {
				featureAssoc.processImage(left_frame, i, landmarks, trials, unmatched);

				// 3D-2D VO

				// add pose init, and factors

				if (landmarks.size() < landmarkThre) {
					cout << "new key frame" << endl;
					initKeyFrame(i);
				}
			}
		}
		int t2 = cv::getTickCount();

		//frame info
		cout << i << ":" << "landmarks:" << landmarks.size();
		cout << " trials:" << trials.size();
		cout << " time:" << float(t2 - t1) / cv::getTickFrequency() << endl;
		graph.printInitials();

		//visualize frame
		featureAssoc.visualizeTrace(landmarks,trials);
		cv::imshow("SLAM", featureAssoc.getDisplayFrame());
		cv::waitKey(0);

		//send path to rviz
	}

	return 0;
}
