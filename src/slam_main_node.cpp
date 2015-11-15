#include <slam_main/feature_associator.h>
#include <slam_main/factor_graph.h>
#include <slam_main/camera.h>
#include <slam_main/helper.h>
//std
#include <iostream>
//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

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

	// prepare camera projection matrix
	cv::Mat camRT=identityRT();
	cv::Mat p1 = camera.intrinsic * camRT;
	camRT.at<float>(0, 3) -= camera.baseline;
	cv::Mat p2 = camera.intrinsic * camRT;
	// prepare 2D image points
	vector<cv::Point2f> pts1, pts2;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		pts1.push_back(it->firstPoint().pt);
		pts2.push_back(it->getPair().pt);
	}
	// triangulate
	cv::Mat outpts;
	cv::triangulatePoints(p1, p2, pts1, pts2, outpts);

	// convert to work coordinate
	int sframe = trials.back().getStartFrame();
	cv::Mat posR, posT;
	graph.getPose(sframe, posR, posT);
	cv::Mat posRT= fullRT(posR, posT);
	outpts = posRT*outpts;

	// normalize
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
int main( int argc, char** argv ) {
	cv::namedWindow("SLAM", cv::WINDOW_NORMAL);

	int startFrame = 130;
	bool trialState = true;

	//rviz marker setup
	ros::init(argc, argv, "slam_traj");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	visualization_msgs::Marker marker = createMarker();

	for (int i = startFrame; i < 634; i++) {
		//read image
		string path = "/home/wenda/Developer/Autonomy/cmu_16662_p2/sensor_data/";
		cv::Mat left_frame = cv::imread(path + "left" + fixedNum(i) + ".jpg");
		cv::Mat right_frame = cv::imread(path + "right" + fixedNum(i) + ".jpg");

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
						cv::noArray(), Rvec, Tvec, false, 3000, 1.0, 0.99, inliers, cv::SOLVEPNP_EPNP);
				cv::Rodrigues(Rvec, R);
				//cout<<"R"<<R<<endl;
				//cout<<"T"<<Tvec<<endl;
				//cout<<"inlier"<<inliers<<endl;
				cout<<"P3P points:" << objPts.size() << " inliers:" << inliers.size() << endl;

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
		if (!landmarks.empty()) {
			graph.getPose(i,curR,curT);

			//send path to rviz
			geometry_msgs::Point new_p;
			new_p.x = curT.at<float>(0, 0);
			new_p.y = curT.at<float>(1, 0);
			new_p.z = curT.at<float>(2, 0);

			marker.points.push_back(new_p);
			marker_pub.publish(marker);
		}
		cout << "current pose:" << curR <<endl;
		cout << curT << endl;
		cout << "end>> landmarks:" << landmarks.size()
				<< " trials:" << trials.size() << endl;
		cout << "time:" << float(t2 - t1) / cv::getTickFrequency() << endl;

		//visualize frame
		featureAssoc.visualizeTrace(landmarks, trials);
		cv::imshow("SLAM", featureAssoc.getDisplayFrame());
		cv::waitKey(1);

	}

	return 0;
}
