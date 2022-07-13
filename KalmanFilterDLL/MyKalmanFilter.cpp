#include "MyKalmanFilter.h"

using namespace std;


MyKalmanFilterDLL::MyKalmanFilter::MyKalmanFilter(void)
{
	this->kf = new cv::KalmanFilter(4, 4, 0, CV_32F);
	this->state = new cv::Mat(4, 1, CV_32FC1);
	this->processNoise = new cv::Mat(4, 1, CV_32FC1);
	this->measurement = new cv::Mat(4, 1, CV_32FC1);

	randn(*this->state,
		   cv::Scalar::all(0),
		   cv::Scalar::all(0.1)
		   );
	kf->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1.43, 1,	// TODO: 1, 0, 1.43, 0 ??????
													 0, 1, 0, 1.43,
													 0, 0, 1, 0,
													 0, 0, 0, 1);
	cv::setIdentity(this->kf->measurementMatrix);
	cv::setIdentity(this->kf->processNoiseCov, cv::Scalar::all(1e-5));
	cv::setIdentity(this->kf->measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(this->kf->errorCovPost, cv::Scalar::all(1));

	randn(this->kf->statePost,
		  cv::Scalar::all(0),
		  cv::Scalar::all(0.1)
		  );
}

void MyKalmanFilterDLL::MyKalmanFilter::filter(double atPositionX, double atPositionY, double speedHoriz, double speedVert)
{
	cv::Mat prediction = kf->predict();

	this->measurement->at<float>(0) = (float)atPositionX;
	this->measurement->at<float>(1) = (float)atPositionY;
	this->measurement->at<float>(2) = (float)speedHoriz;
	this->measurement->at<float>(3) = (float)speedVert;

	this->kf->correct(*this->measurement);

	this->x = round(this->kf->statePost.at<float>(0));
	this->y = round(this->kf->statePost.at<float>(1));

	randn(*this->processNoise,
		   cv::Scalar(0),
		   cv::Scalar::all(sqrt(this->kf->processNoiseCov.at<float>(0, 0)))
		  );
	*this->state = (this->kf->transitionMatrix*(*this->state)) + (*this->processNoise);
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