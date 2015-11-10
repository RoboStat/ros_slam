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
unsigned int trialThre = 200;
FeatureAssociator featureAssoc;

// factor graph
FactorGraph graph(camera);

// sychronize the landmarks position with the one in factor graph
void updateLandmark() {
	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		cv::Point3f p;
		graph.getLandMark(it->first, p);
		it->second.setLocation(p);
	}
}

// initialize a key frame
void initTrials() {
	// triangulate trial points
	int sframe = trials.back().getStartFrame();
	cv::Mat camR, camT;
	graph.getPose(sframe, camR, camT);
	camR = camR.t();
	camT = -camR * camT;
	cv::Mat camRT;
	cv::hconcat(camR, camT, camRT);
	cv::Mat p1 = camera.intrinsic * camRT;
	camRT.at<float>(0, 3) -= camera.baseline;
	cv::Mat p2 = camera.intrinsic * camRT;

	vector<cv::Point2f> pts1, pts2;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		pts1.push_back(it->firstPoint().pt);
		pts2.push_back(it->getPair().pt);
	}
	//cout << "RT" << camRT <<endl;
	//cout << "p1" << p1 << endl;
	//cout << "p2" << p2 << endl;
	//cout << "pts1" << pts1 << endl;
	//cout << "pts2" << pts2 << endl;

	cv::Mat outpts;
	cv::triangulatePoints(p1, p2, pts1, pts2, outpts);
	outpts.row(0) = outpts.row(0) / outpts.row(3);
	outpts.row(1) = outpts.row(1) / outpts.row(3);
	outpts.row(2) = outpts.row(2) / outpts.row(3);
	//cout << "triangulated points:" << outpts << endl;

	int colcount = 0;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		cv::Point3f p = cv::Point3f(outpts.at<float>(0, colcount),
				outpts.at<float>(1, colcount),
				outpts.at<float>(2, colcount));
		it->setLocation(p);
		colcount++;
	}

}

void confirmTrials() {
	for (auto it = trials.begin(); it != trials.end(); it++) {
		// add initial value of land mark to factor graph
		graph.addLandMark(landmarkID, it->getLocation());
		// move from trail to true landmarks
		landmarks[landmarkID] = *it;
		// add factors
		int curframe = it->getStartFrame();
		for (auto itt = it->pointBegin(); itt != it->pointEnd() - 1; itt++) {
			graph.addProjection(curframe, landmarkID, itt->pt);
			curframe++;
		}
		landmarkID++;
	}
	trials.clear();
}

//////////////////////main///////////////////////////
int main() {
	cv::namedWindow("SLAM", cv::WINDOW_NORMAL);

	int startFrame = 100;
	bool trialState = true;
	for (int i = startFrame; i < 634; i++) {
		//read image
		string path = "/home/wenda/Developer/SLAM/";
		cv::Mat left_frame = cv::imread(path + "resource/left" + fixedNum(i) + ".jpg");
		cv::Mat right_frame = cv::imread(path + "resource/right" + fixedNum(i) + ".jpg");

		// print frame info
		cout << "-----------frame " << i << " ------------" << endl;
		cout << "start>> landmarks:" << landmarks.size()
				<< " trials:" << trials.size() << endl;

		//track points
		long t1 = cv::getTickCount();

		// first frame
		if (i == startFrame) {
			graph.addFirstPose(i);
			featureAssoc.initTrials(left_frame, right_frame, i, trials);
			initTrials();
			featureAssoc.visualizePair(trials);
		} else {
			featureAssoc.processImage(left_frame, i, landmarks, trials, unmatched);

			if (trialState && trials.size() < trialThre) {
				cout << "--confirm all trials--" << endl;
				confirmTrials();
				trialState = false;
			}

			if (!landmarks.empty()) {
				// 3D-2D VO
				vector<cv::Point3f> objPts;
				vector<cv::Point2f> imgPts;
				for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
					objPts.push_back(it->second.getLocation());
					imgPts.push_back(it->second.lastPoint().pt);
				}
				cv::Mat R, Rvec, Tvec, inliers;
				cv::solvePnPRansac(objPts, imgPts, camera.intrinsic,
						cv::noArray(), Rvec, Tvec, false, 300, 1.0, 0.99, inliers, cv::SOLVEPNP_P3P);
				cv::Rodrigues(Rvec, R);
				//cout<<"R"<<R<<endl;
				//cout<<"T"<<Tvec<<endl;
				//cout<<"inlier"<<inliers<<endl;

				// add pose init, and factors
				for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
					graph.addProjection(i, it->first, it->second.lastPoint().pt);
				}
				graph.addPose(i, R, Tvec);

				// run bundle adjustment
				//graph.printInitials();
				long u1 = cv::getTickCount();
				graph.update();
				long u2 = cv::getTickCount();
				cout << "optimization:" << float(u2 - u1) / cv::getTickFrequency() << endl;
				updateLandmark();

			}

			if (!trialState && landmarks.size() < landmarkThre) {
				cout << "--init new trails--" << endl;
				featureAssoc.refreshTrials(right_frame, unmatched, trials);
				initTrials();
				featureAssoc.visualizePair(trials);
				trialState = true;
			}
		}
		long t2 = cv::getTickCount();

		//frame info
		cv::Mat curR,curT;
		graph.getPose(i,curR,curT);
		cout << "current pose:" << curR <<endl;
		cout << curT << endl;
		cout << "end>> landmarks:" << landmarks.size()
				<< " trials:" << trials.size() << endl;
		cout << "time:" << float(t2 - t1) / cv::getTickFrequency() << endl;

		//visualize frame
		featureAssoc.visualizeTrace(landmarks, trials);
		cv::imshow("SLAM", featureAssoc.getDisplayFrame());
		cv::waitKey(1);

		//send path to rviz
	}

	return 0;
}
