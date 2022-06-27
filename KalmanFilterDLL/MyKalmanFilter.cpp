//#include "StdAfx.h"
#include "MyKalmanFilter.h"

using namespace std;


MyKalmanFilterDLL::MyKalmanFilter::MyKalmanFilter(void)
{
	//KalmanFilter::rng = cvRNG(-1);

	kf = new cv::KalmanFilter(4, 4, 0, CV_32F);
	state = new cv::Mat(4, 1, CV_32FC1);
	processNoise = new cv::Mat(4, 1, CV_32FC1);
	measurement = new cv::Mat(4, 1, CV_32FC1);
	//char code = (char)-1;

	////cvZero(measurement);

	randn(*state, cv::Scalar::all(0), cv::Scalar::all(0.1));
	kf->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1.43, 1, 0, 1, 0, 1.43, 0, 0, 1, 0, 0, 0, 0, 1);	// TODO: 1, 0, 1.43, 0 ??????
	cv::setIdentity(kf->measurementMatrix);
	cv::setIdentity(kf->processNoiseCov, cv::Scalar::all(1e-5));
	cv::setIdentity(kf->measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(kf->errorCovPost, cv::Scalar::all(1));

	randn(kf->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
}


MyKalmanFilterDLL::MyKalmanFilter::MyKalmanFilter(double x, double y, double vHoriz, double vVert)
{
	kf = new cv::KalmanFilter(4, 4, 0);
	state = new cv::Mat(4, 1, CV_32FC1); /* (x, y, vcosL, vsinL) */
	processNoise = new cv::Mat(4, 1, CV_32FC1);
	measurement = new cv::Mat(4, 1, CV_32FC1);

	////cvZero(measurement);

	////cvRandArr(&rng, state, CV_RAND_NORMAL, cvRealScalar(0), cvRealScalar(0.1));

	state->at<float>(0) = (float) x;
	state->at<float>(1) = (float) y;
	state->at<float>(2) = (float) vHoriz;
	state->at<float>(3) = (float) vVert;

	kf->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 10.0, 1, 0, 1, 0, 10.0, 0, 0, 1, 0, 0, 0, 0, 1);

	cv::setIdentity(kf->measurementMatrix);
	cv::setIdentity(kf->processNoiseCov, cv::Scalar::all(1e-5));
	cv::setIdentity(kf->measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(kf->errorCovPost, cv::Scalar::all(1));

	kf->statePost.at<float>(0) = (float) x;
	kf->statePost.at<float>(1) = (float) y;
	kf->statePost.at<float>(2) = (float) vHoriz;
	kf->statePost.at<float>(3) = (float) vVert;

	////cvRandArr(&rng, kalman->state_post, CV_RAND_NORMAL, cvRealScalar(0), cvRealScalar(0.1));
	////randn(kf->statePost, cv::Scalar(0), cv::Scalar::all(0.1));
}


void MyKalmanFilterDLL::MyKalmanFilter::filter(double atPositionX, double atPositionY, double speedHoriz, double speedVert)
{
	cv::Mat prediction = kf->predict();
	float x_predicted = prediction.at<float>(0);
	float y_predicted = prediction.at<float>(1);
	float v_Horiz_predicted = prediction.at<float>(2);
	float v_Vert_predicted = prediction.at<float>(3);

	measurement->at<float>(0) = (float) atPositionX;
	measurement->at<float>(1) = (float) atPositionY;
	measurement->at<float>(2) = (float) speedHoriz;
	measurement->at<float>(3) = (float) speedVert;

	kf->correct(*measurement);

	x = round(kf->statePost.at<float>(0));
	y = round(kf->statePost.at<float>(1));

	//this->limit();

	randn(*processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(kf->processNoiseCov.at<float>(0, 0))));
	*state = (kf->transitionMatrix*(*state)) + (*processNoise);
}


double MyKalmanFilterDLL::MyKalmanFilter::getX()
{
	return this->x;
}


double MyKalmanFilterDLL::MyKalmanFilter::getY()
{
	return this->y;
}


void MyKalmanFilterDLL::MyKalmanFilter::limit()
{
	if (this->x < 0) this->x = 0;
	if (this->y < 0) this->y = 0;
}