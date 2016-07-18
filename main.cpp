#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "blob.h"
#include "BlobResult.h"
#include "highgui.h" //include it to use GUI functions.

#define WIDTH 640
#define HEIGHT 360
const int NUMCORES = 2;
using namespace std;
using namespace cv;


int cMod(int color){
	return color % 256;
}

void csv(ofstream* stream, int lx, int ly, bool lh, int rx, int ry, int rh, bool confirmed){
	*stream << lx << "," << ly << "," << lh << "," << rx << "," << ry << "," << rh << "," << confirmed << endl;
}
void initialize_kalman(KalmanFilter* filter, Point start);


int main(){

	Scalar robotColor = CV_RGB(255, 0, 0);
	Scalar rightColor = CV_RGB(0, 255, 0);
	Scalar leftColor = CV_RGB(0, 0, 255);
	Scalar robotColor_2 = CV_RGB(0, 255, 255);
	Scalar rightColor_2 = CV_RGB(255, 0, 255);
	Scalar leftColor_2 = CV_RGB(255, 255, 0);

	int lowH = 0;
	int highH = 14;
	int top_cut = 120;
	int bot_cut = 70;
	int lowV = 200;
	int type = 0;
	int ticks = 0;
	int nb_errors = 0;
	int len = 150;
	int trace = 25;
	int sensitivity = 100;
	int area = 3000;
	int flip = 0; //set to 0 if no flips are needed, 1 for y axis, 2 for x axis and 3 for both

	namedWindow("My Window", 1);
	createTrackbar("lowH", "My Window", &lowH, 180);
	createTrackbar("highH", "My Window", &highH, 180);
	createTrackbar("top_cut", "My Window", &top_cut, 255);
	createTrackbar("bot_cut", "My Window", &bot_cut, 255);
	createTrackbar("lowV", "My Window", &lowV, 255);
	createTrackbar("LEN", "My Window", &len, 300);
	createTrackbar("TRACE", "My Window", &trace, 100);
	createTrackbar("SENSITIVITY", "My Window", &sensitivity, 200);
	createTrackbar("AREA", "My Window", &area, 7000);
	createTrackbar("FLIP", "My Window", &flip, 3);
	moveWindow("My Window", 0, 0);

	namedWindow("kalman", 1);
	moveWindow("kalman", 500, 0);
	namedWindow("Blobs Image", 1);
	moveWindow("Blobs Image", 500, 300);
	namedWindow("frame", 1);
	moveWindow("frame", 0, 500);
	namedWindow("test", WINDOW_AUTOSIZE);
	moveWindow("test", 0, 500);
	namedWindow("white", WINDOW_AUTOSIZE);
	moveWindow("white", 0, 500);

	//file of video input
	string filename = "testVideo_5.webm";
	ofstream logs;
	ofstream stats;
	stats.open("stats.txt");
	logs.open("logs.csv");
	logs << "Left_x,Left_y,Left_holds,Right_x,Right_y,Right_holds,confirmed" << endl;

	Point center_window = Point(WIDTH/2, (HEIGHT - top_cut - bot_cut)/2);
	Point center_left = Point(WIDTH/4, .5*max(10, HEIGHT - top_cut - bot_cut));
	Point center_right = Point(3*WIDTH/4, .5*max(10, HEIGHT - top_cut - bot_cut));


	// initialize the kalman filters
	KalmanFilter KF_left(4, 2, 0);
	KalmanFilter KF_right(4, 2, 0);

	Mat_<float> measurement_left(2,1); measurement_left.setTo(Scalar(0));
	Mat_<float> measurement_right(2,1); measurement_right.setTo(Scalar(0));

	initialize_kalman(&KF_left, center_left);
	initialize_kalman(&KF_right, center_right);

	VideoCapture cap(0);

  // VideoCapture cap(filename);

	Mat kf_img(HEIGHT - top_cut - bot_cut, WIDTH, CV_8UC3);
	vector<Point> mousev_left,kalmanv_left;
	mousev_left.clear();
	kalmanv_left.clear();
	vector<Point> mousev_right,kalmanv_right;
	mousev_right.clear();
	kalmanv_right.clear();

	int counter = 0;
	int nb_confirmed = 0;
	int nb_total = 0;
	double average_left = 0;
	double average_right = 0;
	double error_left = 0;
	double error_right = 0;
	double prev_dist = norm(center_left - center_right);
	double new_dist = prev_dist;
	bool left_valid = false;
	bool right_valid = true;
	Mat temp = Mat::zeros(100,400, CV_8UC3);
	putText(temp, "Press any key to start", Point(50,50), FONT_HERSHEY_SIMPLEX, .5, Scalar(255,255,255));
	putText(temp, "and ESC to end", Point(50, 75), FONT_HERSHEY_SIMPLEX, .5, Scalar(255,255,255));
	imshow("Blobs Image", temp);


	waitKey(-1);
	int key;
	bool eof = false;

	for(;;){

		Mat frame;
		Mat prediction_left = KF_left.predict();
		Point new_left(prediction_left.at<float>(0), prediction_left.at<float>(1));
		measurement_left(0) = center_left.x;
		measurement_left(1) = center_left.y;

		Mat estimated_left = KF_left.correct(measurement_left);

		Point statePt_left(estimated_left.at<float>(0),estimated_left.at<float>(1));
		Point measPt_left(measurement_left(0),measurement_left(1));

		Mat prediction_right = KF_right.predict();
		Point new_right(prediction_right.at<float>(0), prediction_right.at<float>(1));
		measurement_right(0) = center_right.x;
		measurement_right(1) = center_right.y;

		Mat estimated_right = KF_right.correct(measurement_right);

		Point statePt_right(estimated_right.at<float>(0),estimated_right.at<float>(1));
		Point measPt_right(measurement_right(0),measurement_right(1));

		ticks ++;
		error_left = norm(statePt_left - measPt_left);
		average_left = ((average_left * (ticks - 1)) + error_left) / ticks;
		error_right = norm(statePt_right - measPt_right);
		average_right = ((average_right * (ticks - 1)) + error_right) / ticks;

		imshow("kalman", kf_img);
		// waitKey(-1);
		kf_img = Scalar::all(0);
		mousev_left.push_back(measPt_left);
		kalmanv_left.push_back(statePt_left);

		circle(kf_img, statePt_left, 1,  Scalar(255,255,255), -1);
		circle(kf_img, measPt_left, 1, Scalar(0,0,255), -1);
		int nb_mousev_left = mousev_left.size() - 1;
		int nb_kalmanv_left = mousev_left.size() - 1;
		int nb_mousev_right = mousev_left.size() - 1;
		int nb_kalmanv_right = mousev_left.size() - 1;

		for(int i = max(0, nb_mousev_left - trace); i< nb_mousev_left; i++){
			line(kf_img, mousev_left[i], mousev_left[i+1], Scalar(255,255,0), 1);
		}
		for(int i = max(0, nb_kalmanv_left - trace); i< nb_kalmanv_left; i++){
			line(kf_img, kalmanv_left[i], kalmanv_left[i+1], Scalar(0,0,255), 1);
		}

		mousev_right.push_back(measPt_right);
		kalmanv_right.push_back(statePt_right);

		circle(kf_img, statePt_right, 1,  Scalar(255,255,255), -1);
		circle(kf_img, measPt_right, 1, Scalar(0,0,255), -1);

		for(int i = max(0, nb_mousev_right - trace); i< nb_mousev_right; i++){
			line(kf_img, mousev_right[i], mousev_right[i+1], Scalar(0,255,0), 1);
		}
		for(int i = max(0, nb_kalmanv_right - trace); i< nb_kalmanv_right; i++){
			line(kf_img, kalmanv_right[i], kalmanv_right[i+1], Scalar(255,0,255), 1);
		}


		Rect border(0, top_cut, WIDTH, max(10, HEIGHT - top_cut - bot_cut));
		cap >> frame;

		if(!frame.empty()){

			Mat image;
			int flip_type = 1;
			switch (flip) {
				case 0: break;
				case 1:	break;
				case 2: flip_type = 0;
				break;
				case 3: flip_type = -1;
				break;
			}
			if(flip) cv::flip(frame, frame, flip_type);

			resize(frame, frame, Size(WIDTH, HEIGHT));
			image = frame(border);
			imshow("frame", image);

			//performs the skin detection
			Mat converted_skin;
			cvtColor(image, converted_skin, CV_BGR2HSV);

			Mat skin_masked;
			inRange(converted_skin, Scalar(min(lowH, highH), 48, 80),Scalar(max(lowH, highH), 255, 255), skin_masked);
			imshow("test", skin_masked);

			//performs the robot detection
			Mat converted_white, white_masked, lights_masked;
			cvtColor(image, converted_white, CV_BGR2GRAY);
			inRange(converted_skin, Scalar(0, 0, 245), Scalar(180, 255, 255), lights_masked);
			threshold(converted_white, white_masked, lowV, 255, type);
			bitwise_or(white_masked, lights_masked, white_masked);
			imshow("white", white_masked);


			Mat copy(converted_skin.size(), converted_skin.type());// = converted.clone();

			//detects hands as blobs
			CBlobResult blobs;
			IplImage temp = (IplImage)skin_masked;
			blobs = CBlobResult(&temp,NULL,1);
			blobs = CBlobResult(skin_masked,Mat(),NUMCORES);
			int numBlobs = blobs.GetNumBlobs();
			if(0 == numBlobs){
				cout << "can't find blobs!" << endl;
				continue;
			}

			// detects robot as a blob
			CBlobResult robot_blobs;
			IplImage robot_temp = (IplImage) white_masked;
			robot_blobs = CBlobResult(&robot_temp, NULL, 1);
			robot_blobs = CBlobResult(white_masked, Mat(), NUMCORES);
			if(0 == robot_blobs.GetNumBlobs()){
				cout << "can't find robot_blobs!" << endl;
				continue;
			}

			CBlob *curblob;
			CBlob* blob_1;
			CBlob* blob_2;
			CBlob* leftBlob;
			CBlob* rightBlob;
			CBlob* robotBlob;


			copy.setTo(Vec3b(0,0,0));

			// chooses the two largest blobs for the hands
			Point center_1, center_2;
			int max_1 = 0;
			int max_2 = 0;
			int maxArea_1 = 0;
			int maxArea_2 = 0;
			for(int i=0;i<numBlobs;i++){
				int area = blobs.GetBlob(i)->Area();
				if(area > maxArea_1){
					maxArea_2 = maxArea_1;
					maxArea_1 = area;
					max_2 = max_1;
					max_1 = i;
				} else if(area > maxArea_2){
					maxArea_2 = area;
					max_2 = i;
				}
			}
			int i_1 = max_1;
			int i_2 = max_2;
			double area_left, area_right;
			Rect rect_1;
			Rect rect_2;

			//determines which hand is left/right
			blob_1 = blobs.GetBlob(i_1);
			blob_2 = blobs.GetBlob(i_2);
			center_1 = blob_1->getCenter();
			center_2 = blob_2->getCenter();
			bool left_is_1 = (center_1.x < center_2.x)? true : false;
			leftBlob = (left_is_1)? blob_1 : blob_2;
			rightBlob = (left_is_1)? blob_2 : blob_1;
			center_left = leftBlob->getCenter();
			center_right = rightBlob->getCenter();

			//determine the number of valid hands
			//validity is decided by whether or not the hand followed a logical movement,
			//and if the area of the blob is large enough to be accepted
			int valids = 0;
			rect_1 = leftBlob->GetBoundingBox();
			rectangle(copy, rect_1.tl(), rect_1.br(), leftColor_2, 5);
			error_left = norm(statePt_left - center_left);
			area_left = leftBlob->Area();
			left_valid = error_left < sensitivity && area_left > area;
			if(left_valid){
				leftBlob->FillBlob(copy,leftColor, true);
				valids ++;
			}
			circle(copy, center_left, 5, leftColor_2, -1);


			rect_2 = rightBlob->GetBoundingBox();
			rectangle(copy, rect_2.tl(), rect_2.br(), rightColor_2, 5);
			error_right = norm(statePt_right - center_right);
			area_right = rightBlob->Area();
			right_valid = error_right < sensitivity && area_right > area;
			if(right_valid){
				rightBlob->FillBlob(copy,rightColor, true);
				valids ++;
			}
			circle(copy, center_right, 5, rightColor_2, -1);


			//finds the blob representing the robot
			//we could add a restriction to only choose a blob between the two hands
			//in terms of x-coordinate
			//a Kalman check can easily be done for the robot
			Point robot_center;
			maxArea_1 = 0;
			max_1 = 0;
			numBlobs = robot_blobs.GetNumBlobs();
			if(0 < numBlobs){
				for(int i=0;i<numBlobs;i++){
					curblob = robot_blobs.GetBlob(i);
					robot_center = curblob->getCenter();
					double dist_1 = norm(center_1 - robot_center);
					double dist_2 = norm(center_2 - robot_center);
					if(dist_1 < len || dist_2 < len){
						double area = robot_blobs.GetBlob(i)->Area();
						if(area > maxArea_1){
							max_1 = i;
							maxArea_1 = area;
						}
					}
				}
				int i_3 = max_1;
				curblob = robot_blobs.GetBlob(i_3);
				curblob->FillBlob(copy,robotColor, true);
				robot_center = curblob->getCenter();
				circle(copy, robot_center, 5, robotColor_2, -1);
				Rect rect_3 = curblob->GetBoundingBox();
				rectangle(copy, rect_3.tl(), rect_3.br(), robotColor_2, 5);

				// determines which hand is controlling the robot
				// by cheching the position of the 3 blobs
				// an additional check could be done by verifying if
				//the center of the robot is moving in the same direction
				//as the center of the hand moving it
				bool is_left = false;
				bool is_right = false;
				bool confirmed = false;

				double dist_left = norm(center_left - robot_center);
				double dist_right = norm(center_right - robot_center);
				double dist_both = norm(center_left - center_right);

				Point robot_tl = rect_3.tl();
				Point robot_br = rect_3.br();

				int left_count = 0;
				int right_count = 0;

				if(rect_1.contains(robot_tl)) left_count++;
				if(rect_1.contains(robot_br)) left_count++;
				if(rect_1.contains(robot_center)) left_count++;
				if(rect_2.contains(robot_tl)) right_count++;
				if(rect_2.contains(robot_br)) right_count++;
				if(rect_2.contains(robot_center)) right_count++;


				switch(valids){
					case 0: break;
					case 1:{
						int area_sum = area_left + area_right;
						if(dist_left > 2* dist_right || dist_right > 2*dist_left){
							if(area_sum > 2 * area && (area_left > 2*area_right || area_right > 2*area_left) &&
							((left_valid && left_count > 0)||(right_valid && right_count > 0))){
								is_left = true;
								is_right = true;
								if(left_count > 2 || right_count > 2) confirmed = true;
							}
						}
						if(left_valid && left_count > 1) {
							is_left = true;
							if(left_count > 2) confirmed = true;
						}
						if(right_valid && right_count > 1) {
							is_right = true;
							if(right_count > 2) confirmed = true;
						}

						//if just one hand is on screen
						if(area_right < area/2){
							if(center_left.x > robot_center.x){
								is_left = true;
							} else{
								is_right = true;
							}
						} else if (area_left < area/2){
							if(center_right.x < robot_center.x){
								is_right = true;
							} else{
								is_left = true;
							}
						}
						break;}
						case 2:{
							int moreLeft = left_count - right_count;
							int moreRight = right_count - left_count;
							int countSum = left_count + right_count;

							switch (countSum) {
								case 3:{

									switch (left_count) {
										case 3: is_left = true;
										confirmed = true;
										break;
										case 2:
										case 1: is_left = true;
										is_right = true;
										confirmed= true;
										break;
										case 0: is_right = true;
										confirmed = true;
										break;
									}
								}
								case 2:{

									switch (left_count) {
										case 2: is_left = true;
										confirmed = true;
										break;
										case 1: is_left = true;
										is_right = true;
										break;
										case 0: is_right = true;
										confirmed = true;
										break;
									}
								}
								case 1:{

									switch (left_count) {
										case 1: is_left = true;
										break;
										case 0: is_right = true;
										break;
									}
								}
								case 0:{
									break;
								}
							}


							break;}
						}

						bool found = false;
						for(size_t i = robot_tl.x; i<= robot_br.x && !found; i++){
							for(size_t j = robot_tl.y; j<= robot_br.y && !found; j++){
								int color1 = 0; int color2 = 255;
								Vec3b colour = copy.at<Vec3b>(Point(i, j));
								if(colour[1] == color1 && colour[0] == color2){
									found = true;
									is_left = true;
								}
								if(colour[1] == color2 && colour[0] == color1){
									found = true;
									is_right = true;
								}
							}
						}
						if (found) confirmed = true;

						if(!is_left && !is_right){
							cout << "-- none!";
							if(left_count == 0 && right_count == 0) confirmed = true;
						} else if(is_left && is_right){
							cout << "-- both!";
						} else {

							if (is_left){
								cout << " -- left!";
							} else {
								cout << " -- right!";
							}
						}



imshow("kalman", kf_img);
// up till here

						if(confirmed){
							nb_confirmed ++;
							cout << " -- confirmed" << endl;
						} else {
							cout << endl;
						}
						csv(&logs, center_left.x, center_left.y, is_left, center_right.x, center_right.y, is_right, confirmed);
					}
					nb_total ++;



					//
					// //displayOverlay("Blobs Image","Multi Thread");
					new_dist = norm(center_left - center_right);
					// don't throw errors in the first 10 frames
					if(ticks > 10){
						if(error_left > 20 && error_right > 20 /*&& new_dist < prev_dist*/){
							circle(copy, Point(WIDTH/2, HEIGHT/2), 100, Scalar(0, 0, 255), 30);
							nb_errors ++;
						}
					}

					prev_dist = new_dist;

					imshow("Blobs Image",copy);


					key = waitKey(10);
		} else{
			eof = true;
		}

		if(27 == key || 1048603 == key || eof){
			double kalman_error_percentage = (nb_errors*100.0)/ticks;
			double confirm_percentage = (nb_confirmed*100.0/nb_total);
			stats << "kalman error frequency: " << kalman_error_percentage << "\%" << endl;
			stats << "confirmed: " << confirm_percentage << "\%" << endl;

			logs.close();
			stats.close();
			return 0;
		}

	}
}

void initialize_kalman(KalmanFilter* filter, Point start){
	filter->transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);
	filter->statePre.at<float>(0) = start.x;
	filter->statePre.at<float>(1) = start.y;
	filter->statePre.at<float>(2) = 0;
	filter->statePre.at<float>(3) = 0;
	setIdentity(filter->measurementMatrix);
	setIdentity(filter->processNoiseCov, Scalar::all(1e-4));
	setIdentity(filter->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(filter->errorCovPost, Scalar::all(.1));

}
