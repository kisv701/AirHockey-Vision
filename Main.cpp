#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/video.hpp>

//Serial
#include <conio.h>
#include <tserial.h>
#include "bot_control.h"

using namespace cv;
using namespace std;

//Options

const bool CALIBRATE = false;
const bool SHOW_DISTORTED = false;
const bool SHOW_CALC_FIGURE = true;

//Variabler

//Positionsvariabler

Point lastPosPuck;
Point posPuck;
Point posMallet;
Point posToSend;
Mat calcFigure;

//Camera undisort parameters
Mat dist = (Mat_<double>(5, 1) << -0.6208589298836318, 0.5161176834408764, -0.0001082253293384215, 0.007012593959103706, -0.2293897091733605);
Mat intrinsic = (Mat_<double>(3, 3) << 568.6134663339634, 0, 336.2520667351025, 0, 568.281098319841, 247.0246373180379, 0, 0, 1);

//Color range variables Puck
int iLowH_puck = 51;
int iHighH_puck = 85;

int iLowS_puck = 100;
int iHighS_puck = 253;

int iLowV_puck = 43;
int iHighV_puck = 237;

//Color range variables Mallet
int iLowH_mallet = 100;
int iHighH_mallet = 112;

int iLowS_mallet = 133;
int iHighS_mallet = 224;

int iLowV_mallet = 21;
int iHighV_mallet = 231;

//Field variables
int UPPER_END_OF_FIELD = 34;
int LOWER_END_OF_FIELD = 413;
int LEFT_END_OF_FIELD = 0;
int RIGHT_END_OF_FIELD = 638;

int GOAL_UPPER_EDGE = 175;
int GOAL_LOWER_EDGE = 275;

int DEF_X = LEFT_END_OF_FIELD + 2; //Default X-position
int DEF_Y = ((LOWER_END_OF_FIELD - UPPER_END_OF_FIELD) / 2) + UPPER_END_OF_FIELD; //Default Y-position

const double pixelsPerMM = 1070.0 / (double)(LOWER_END_OF_FIELD - UPPER_END_OF_FIELD);

const int PUCK_RADIUS = 63 / 2 / pixelsPerMM;

//Seriella
serial comm;
Point lastSentPos;
int sercommThreshold = 25;

//Funktioner
Mat getColorMask(Mat src, Scalar lowerBound, Scalar upperBound);
void calibrate(VideoCapture cap);
void printCalibration();
Point getPositionFromMask(Mat mask);
Point unDisortPos(Point pos, Size frameSize);
bool isFirstFrame();
bool isMovingTowardsUs(int dx);
string getFourCharString(string s);
void sendPosition(bool force);
int calcY(int xm);
void generateCalcFigure();


int main(int argc, char** argv) {


	comm.startDevice("COM3", 115200);		//Open serial communication

	VideoCapture cap(1);
	if (!cap.isOpened()) {
		std::cout << "Cannot open video!\n";
		return -1;
	}

	if (CALIBRATE) {
		calibrate(cap);
		printCalibration();
	}

	Mat frame;

	lastPosPuck.x = -1;
	lastPosPuck.y = -1;


	Mat maskPuck;
	Mat maskMallet;

	Point posPuckDisorted;
	Point posMalletDisorted;

	int ySum = 0;
	int it = 1;

	bool attackMode = false;
	int nrMoves = 0;

	while (cap.read(frame)) {

		maskMallet = getColorMask(frame, Scalar(iLowH_mallet, iLowS_mallet, iLowV_mallet), Scalar(iHighH_mallet, iHighS_mallet, iHighV_mallet));
		posPuckDisorted = getPositionFromMask(maskPuck);

		maskPuck = getColorMask(frame, Scalar(iLowH_puck, iLowS_puck, iLowV_puck), Scalar(iHighH_puck, iHighS_puck, iHighV_puck));
		posMalletDisorted = getPositionFromMask(maskMallet);

		if ((posPuckDisorted.x != -1 || posPuckDisorted.y != -1) && (posMalletDisorted.x != -1 || posMalletDisorted.y != -1)) {
			//Puck and mallet was found.
			posPuck = unDisortPos(posPuckDisorted, frame.size());
			posMallet = unDisortPos(posMalletDisorted, frame.size());

			if (!isFirstFrame()) {
				//Puck is moving towards us
				if (abs(posPuck.x - lastPosPuck.x) < 3 && posPuck.x < 650 / pixelsPerMM) {
					posToSend.y = posPuck.y;
					posToSend.x = posPuck.x;
					//cout << "still" << endl;
				}
				else if (isMovingTowardsUs(5)) {

					if (isMovingTowardsUs(25) && !attackMode) {
						attackMode = 0;
					}
					else {
						attackMode = 1;
					}

					if (attackMode) {

						cout << "Attack" << endl;

						if (nrMoves == 0 && posPuck.x < (RIGHT_END_OF_FIELD - LEFT_END_OF_FIELD) / 2 + 50) {
							if (ySum == 0) {
								posToSend.y = ySum / it;
							}
							else {
								posToSend.y = calcY(DEF_X);
							}
							cout << "Rör sig i sidled" << endl;
							sendPosition(0);
							nrMoves++;
							ySum = 0;
							it = 1;
						}
						else if (nrMoves == 0) {

							int yCalc = calcY(DEF_X);

							if (ySum == 0) {
								ySum = yCalc;
							}

							else {

								ySum += yCalc * 3;
								it++; it++; it++;

							}

						}
						if (nrMoves == 1 && abs(lastSentPos.y - posMallet.y) < 25) {

							cout << "Attackerar" << endl;
							if (ySum != 0) {
								posToSend.y = ySum / it;
							}
							else {
								posToSend.y = calcY(600 / pixelsPerMM);
							}
							posToSend.x = 600 / pixelsPerMM;
							nrMoves++;

						}
						else if (nrMoves == 1) {

							int yCalc = calcY(600 / pixelsPerMM);

							if (ySum == 0) {
								ySum = yCalc;
							}

							else {

								ySum += yCalc * 3;
								it++; it++; it++;

							}

						}

					}
					else {
						cout << "Försvara" << endl;
						int yCalc = calcY(posToSend.x);
						if (ySum == 0) {
							ySum = yCalc;
						}
						else { //if (abs(yCalc - yToSend) < 50) {
							ySum += yCalc * 3;
							it++; it++; it++;
						}

						posToSend.y = ySum / it;

						if (posToSend.y > GOAL_LOWER_EDGE) {
							posToSend.y = GOAL_LOWER_EDGE - PUCK_RADIUS * 2;
						}
						if (posToSend.y < GOAL_UPPER_EDGE) {
							posToSend.y = GOAL_UPPER_EDGE + PUCK_RADIUS * 2;
						}
					}
				}
				//här är att pucken inte rör sig mot oss
				else
				{
					nrMoves = 0;
					posToSend.x = DEF_X;
					posToSend.y = DEF_Y;
					if (abs(DEF_X - posMallet.x) > 25 || abs(DEF_Y - posMallet.y) > 25) {
						//sendPosition(1);
					}
					ySum = 0;
					it = 1;
					attackMode = 0;
				}

			}

			sendPosition(0);

			lastPosPuck.x = posPuck.x;
			lastPosPuck.y = posPuck.y;

		}

		if (SHOW_DISTORTED) {
			imshow("Distorted ", frame);
		}

		if (SHOW_CALC_FIGURE) {
			calcFigure = frame;
			generateCalcFigure();
			imshow("Clac figure", calcFigure);
		}




		if (waitKey(30) == 27) {  //Wait 30ms for user to press ESC
			break;
		}


	}



	return 0;
}

void generateCalcFigure() {
	calcFigure = calcFigure.zeros(calcFigure.size(), calcFigure.type());
	circle(calcFigure, posPuck, 4, Scalar(0, 0, 255), 1);
	circle(calcFigure, posToSend, 4, Scalar(0, 0, 255), 1);
	rectangle(calcFigure, Rect(Point(LEFT_END_OF_FIELD, UPPER_END_OF_FIELD), Point(RIGHT_END_OF_FIELD, LOWER_END_OF_FIELD)), Scalar(0, 255, 0), 2);
}

//Calibrate mask for puck and mallet
void calibrate(VideoCapture cap) {
	//create a window called "Puck Control" for puck mask
	namedWindow("Puck Control", CV_WINDOW_AUTOSIZE);


	//Create trackbars in "Positive Control" window
	createTrackbar("LowH", "Puck Control", &iLowH_puck, 180); //Hue (0 - 180)
	createTrackbar("HighH", "Puck Control", &iHighH_puck, 180);

	createTrackbar("LowS", "Puck Control", &iLowS_puck, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Puck Control", &iHighS_puck, 255);

	createTrackbar("LowV", "Puck Control", &iLowV_puck, 255);//Value (0 - 255)
	createTrackbar("HighV", "Puck Control", &iHighV_puck, 255);


	//create a window called "Mallet Control" for mallet mask
	namedWindow("Mallet Control", CV_WINDOW_AUTOSIZE);


	//Create trackbars in "Positive Control" window
	createTrackbar("LowH", "Mallet Control", &iLowH_mallet, 180); //Hue (0 - 180)
	createTrackbar("HighH", "Mallet Control", &iHighH_mallet, 180);

	createTrackbar("LowS", "Mallet Control", &iLowS_mallet, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Mallet Control", &iHighS_mallet, 255);

	createTrackbar("LowV", "Mallet Control", &iLowV_mallet, 255);//Value (0 - 255)
	createTrackbar("HighV", "Mallet Control", &iHighV_mallet, 255);

	//Create a window called "Field Control" to calibrate the field
	namedWindow("Field Control", CV_WINDOW_AUTOSIZE);

	createTrackbar("Left End Of Field", "Field Control", &LEFT_END_OF_FIELD, 640);
	createTrackbar("Right End Of Field", "Field Control", &RIGHT_END_OF_FIELD, 640);
	createTrackbar("Upper End Of Field", "Field Control", &UPPER_END_OF_FIELD, 200);
	createTrackbar("Lower End Of Field", "Field Control", &LOWER_END_OF_FIELD, 480);

	Mat frame;
	Mat maskPuck;
	Mat maskMallet;
	Mat unDisorted;

	while (cap.read(frame)) {

		maskPuck = getColorMask(frame, Scalar(iLowH_puck, iLowS_puck, iLowV_puck), Scalar(iHighH_puck, iHighS_puck, iHighV_puck));
		maskMallet = getColorMask(frame, Scalar(iLowH_mallet, iLowS_mallet, iLowV_mallet), Scalar(iHighH_mallet, iHighS_mallet, iHighV_mallet));


		undistort(frame, unDisorted, intrinsic, dist);
		rectangle(unDisorted, Rect(Point(LEFT_END_OF_FIELD, UPPER_END_OF_FIELD), Point(RIGHT_END_OF_FIELD, LOWER_END_OF_FIELD)), Scalar(0, 255, 0), 2);

		imshow("Undisorted Image", unDisorted);
		imshow("Puck Mask", maskPuck);
		imshow("Mallet Mask", maskMallet);
		imshow("Real Image", frame);

		if (waitKey(30) == 27) {  //Wait 30ms for user to press ESC
			destroyAllWindows();
			break;
		}
	}
}

void printCalibration() {
	printf("Puck Hue Interval: %d - %d \n", iLowH_puck, iHighH_puck);
	printf("Puck Saturation Interval: %d - %d \n", iLowS_puck, iHighS_puck);
	printf("Puck Saturation Interval: %d - %d \n", iLowV_puck, iHighV_puck);

	printf("---------------------------------\n");
	printf("Mallet Hue Interval: %d - %d \n", iLowH_mallet, iHighH_mallet);
	printf("Mallet Saturation Interval: %d - %d \n", iLowS_mallet, iHighS_mallet);
	printf("Mallet Saturation Interval: %d - %d \n", iLowV_mallet, iHighV_mallet);
}

//Generates a colorMask given HSV-parameters
Mat getColorMask(Mat src, Scalar lowerBound, Scalar upperBound) {

	Mat imgHSV;

	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	Mat imgThresholded;

	//Threshold the image
	inRange(imgHSV, lowerBound, upperBound, imgThresholded);

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	return imgThresholded;
}

//Returns a point representing the position of the object filtered out by the mask.
Point getPositionFromMask(Mat mask) {
	Point position;
	position.x = -1;
	position.y = -1;

	//Calculate the moments of the thresholded image
	Moments oMoments = moments(mask);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= 20000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 20000) {

		//calculate the position of the ball
		position.x = dM10 / dArea;
		position.y = dM01 / dArea;
	}

	return position;
}

Point unDisortPos(Point pos, Size frameSize) {
	double a = 0.0;
	double b = -0.0000026;
	double c = 0.0;
	double d = 1.0 - a - b - c;

	int xCam = (frameSize.width / 2);
	int yCam = (frameSize.height / 2);

	int x1 = xCam - pos.x;
	int y1 = yCam - pos.y;

	double r = sqrt((x1*x1) + (y1*y1));

	double rReal = a*r*r*r*r + b*r*r*r + c*r*r + d*r;

	int xReal = xCam - (x1 * (r / rReal));
	int yReal = yCam - (y1 * (r / rReal));

	return Point(xReal, yReal);
}

bool isFirstFrame() {
	if (lastPosPuck.x == -1 && lastPosPuck.y == -1) {
		return true;
	}
	return false;
}

bool isMovingTowardsUs(int dx) {
	if ((posPuck.x - lastPosPuck.x) < -dx) {
		return true;
	}
	return false;
}


void sendPosition(bool force) {

	if (abs(posToSend.x - lastSentPos.x) > sercommThreshold || abs(posToSend.y - lastSentPos.y) > sercommThreshold || force) {

		lastSentPos.x = posToSend.x;
		lastSentPos.y = posToSend.y;

		int xPix = posToSend.x - LEFT_END_OF_FIELD;
		int yPix = posToSend.y - UPPER_END_OF_FIELD;

		int yMM = yPix * pixelsPerMM;
		int xMM = xPix * pixelsPerMM;

		char xArray[4] = { 0,0,0,0 };
		char yArray[4] = { 0,0,0,0 };

		string stringY = std::to_string(yMM);
		string stringX = std::to_string(xMM);

		stringY = getFourCharString(stringY);
		stringX = getFourCharString(stringX);

		//std::cout << "xMM: " << stringX << " , yMM: " << stringY << endl;

		stringX.append(",");
		string outPutString = stringX.append(stringY);

		string begin = "B";
		string terminate = "T";

		outPutString = outPutString.append(terminate);
		outPutString = begin.append(outPutString);

		std::cout << outPutString << endl;

		char stringArray[9];
		for (int i = 0; i < outPutString.length(); i++)
			stringArray[i] = outPutString[i];

		comm.send_array(stringArray, strlen(stringArray));
	}
	else
		return;

}
//Converts a string shorter than 4 chars to a 4 char string with zeroes before the string.
string getFourCharString(string s) {
	string zeroZeroZero = "000";
	string zeroZero = "00";
	string zero = "0";


	if (s.size() > 4) {
		s.resize(4);
	}
	else if (s.size() == 3) {
		s = zero.append(s);
		zero = "0";
	}
	else if (s.size() == 2) {
		s = zeroZero.append(s);
		zeroZero = "00";
	}
	else if (s.size() == 1) {
		s = zeroZeroZero.append(s);
		zeroZeroZero = "000";
	}

	return s;
}
int calcY(int xm) {

	int calcY = (xm - posPuck.x) / (posPuck.x - lastPosPuck.x) * (posPuck.y - lastPosPuck.y) + posPuck.y;

	while (calcY > LOWER_END_OF_FIELD || calcY < UPPER_END_OF_FIELD) {
		if (calcY < UPPER_END_OF_FIELD)
			calcY = 2 * UPPER_END_OF_FIELD - calcY;

		if (calcY > LOWER_END_OF_FIELD)
			calcY = 2 * LOWER_END_OF_FIELD - calcY;

		//std::cout << calcY << endl;
	}

	return calcY;

}