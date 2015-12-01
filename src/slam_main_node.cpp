#include <slam_main/feature_associator.h>
#include <slam_main/feature_associator_nn.h>
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

// features associator
unsigned int landmarkThre = 100;
unsigned int trialThre = 300;
FeatureAssociator* featureAssoc = new FeatureAssociatorNN();

// factor graph
FactorGraph graph(camera);

// Pose Chain
vector<cv::Mat> allR;
vector<cv::Mat> allT;

// sychronize the landmarks position with the one in factor graph
void updateLandmark() {
	for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
		cv::Point3f p;
		graph.getLandMark(it->first, p);
		it->second.setLocation(p);
	}
}

// initialize a key frame
void triangulate(const cv::Mat& pts1,
				 const cv::Mat& pts2,
				 cv::Mat& outpts) {
	// prepare camera projection matrix
	cv::Mat camRT=identityRT();
	cv::Mat p1 = camera.intrinsic * camRT;
	camRT.at<float>(0, 3) -= camera.baseline;
	cv::Mat p2 = camera.intrinsic * camRT;

	// triangulate
	cv::triangulatePoints(p1, p2, pts1, pts2, outpts);

	// normalize
	outpts.row(0) = outpts.row(0) / outpts.row(3);
	outpts.row(1) = outpts.row(1) / outpts.row(3);
	outpts.row(2) = outpts.row(2) / outpts.row(3);
	outpts.row(3) = outpts.row(3) / outpts.row(3);
}

void confirmTrials() {
	// triangulate landmarks
	// prepare points
	cv::Mat pts1(2,trials.size(),cv::DataType<float>::type),
			pts2(2,trials.size(),cv::DataType<float>::type);
	int col = 0;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		cv::KeyPoint k1, k2;
		it->firstPointPair(k1, k2);
		pts1.at<float>(0,col) = k1.pt.x;
		pts1.at<float>(1,col) = k1.pt.y;
		pts2.at<float>(0,col) = k2.pt.x;
		pts2.at<float>(1,col) = k2.pt.y;
		col++;
	}
	cv::Mat pts3D;
	triangulate(pts1,pts2,pts3D);

	// convert to world coordinate
	int sframe = trials.back().getStartFrame();
	cv::Mat posR, posT;
	graph.getPose(sframe, posR, posT);
	cv::Mat posRT= fullRT(posR, posT);
	pts3D = posRT*pts3D;

	int colcount = 0;
	for (auto it = trials.begin(); it != trials.end(); it++) {
		// add location to landmark data structure
		cv::Point3f p = cv::Point3f(pts3D.at<float>(0, colcount),
									pts3D.at<float>(1, colcount),
									pts3D.at<float>(2, colcount));
		it->setLocation(p);
		colcount++;

		// add initial value of land mark to factor graph data structure
		graph.addLandMark(landmarkID, p);

		// move from trail to true landmarks
		landmarks[landmarkID] = *it;

		// add factors
		int curframe = it->getStartFrame();
		for (int i=0; i<it->getTraceSize(); i++) {
			cv::KeyPoint p1, p2;
			it->getPointPair(i,p1,p2);
			graph.addStereo(curframe, landmarkID, p1.pt, p2.pt);
			curframe++;
		}

		landmarkID++;
	}
	trials.clear();
}

//////////////////////main///////////////////////////
int main( int argc, char** argv ) {
	cv::namedWindow("SLAM", cv::WINDOW_NORMAL);
	cv::resizeWindow("SLAM",760,500);

	int startFrame = 1;
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
			featureAssoc->initTrials(left_frame, right_frame, i, trials);
			featureAssoc->visualizePair(trials);
		} else {
			featureAssoc->processImage(left_frame, right_frame, i, landmarks, trials, unmatched);

			if (trialState && (trials.size() < trialThre || landmarks.empty())) {
				cout << "--confirm all trials--" << endl;
				confirmTrials();
				trialState = false;
			}

			if (!landmarks.empty()) {
				// prepare points
				cv::Mat pts1(2,landmarks.size(),cv::DataType<float>::type),
						pts2(2,landmarks.size(),cv::DataType<float>::type);
				vector<cv::Point2f> imgPts;
				int colcount = 0;
				for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
					cv::KeyPoint k1, k2;
					it->second.prevPointPair(k1, k2);
					pts1.at<float>(0,colcount) = k1.pt.x;
					pts1.at<float>(1,colcount) = k1.pt.y;
					pts2.at<float>(0,colcount) = k2.pt.x;
					pts2.at<float>(1,colcount) = k2.pt.y;
					imgPts.push_back(it->second.curLeftPoint().pt);
					colcount++;
				}

				// triangulate 3D points
				cv::Mat pts3D;
				triangulate(pts1,pts2,pts3D);

				// prepare 3D points
				cv::Mat R, Rvec, Tvec, inliers;
				vector<cv::Point3f> objPts;
				for (int c=0; c<pts3D.cols; c++) {
					objPts.push_back(cv::Point3f(pts3D.at<float>(0,c),
												 pts3D.at<float>(1,c),
												 pts3D.at<float>(2,c)));
				}

				// incremental 3D-2D VO
				cv::solvePnPRansac(objPts, imgPts, camera.intrinsic,
						cv::noArray(), Rvec, Tvec, false, 3000, 1.0, 0.99, inliers, cv::SOLVEPNP_P3P);
				cv::Rodrigues(Rvec, R);

				//cout << R << Tvec << endl;
				//cout << inliers << endl;
				cout<<"P3P points:" << imgPts.size() << " inliers:" << inliers.rows << endl;

				//set inliers
				int j=0, count=0;
				for(auto it = landmarks.begin(); it != landmarks.end(); it++) {
					if(count++ ==inliers.at<int>(j,0)) {
						it->second.setInlier(true);
						if(++j>=inliers.rows)
							break;
					}
				}

				// add stereo factors
				for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
					if (it->second.isInlier()) {
						cv::KeyPoint p1, p2;
						it->second.curPointPair(p1, p2);
						graph.addStereo(i, it->first, p1.pt, p2.pt);
					}
				}

				// add initial pose estimation
				allR.push_back(R);
				allT.push_back(Tvec);
				graph.advancePose(i, R, Tvec);

				// run bundle adjustment
				long u1 = cv::getTickCount();
				graph.update();
				long u2 = cv::getTickCount();
				cout << "optimization:" << float(u2 - u1) / cv::getTickFrequency() << endl;
				updateLandmark();
			}

			if (!trialState && landmarks.size() < landmarkThre) {
				cout << "--init new trails--" << endl;
				featureAssoc->refreshTrials(left_frame, right_frame,i, unmatched, trials);
				featureAssoc->visualizePair(trials);
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
		featureAssoc->visualizeTrace(landmarks, trials);
		cv::imshow("SLAM", featureAssoc->getDisplayFrame());
		cv::waitKey(1);

	}

	return 0;
}
