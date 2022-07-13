#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <io.h>


namespace MyKalmanFilterDLL {

	// [x, y, v*sin(alfa), v*cos(alfa)]
	//const float A[] =  {1, 0, 1.43f, 0,	// x = x + dt * (v*sin(alfa)), dt = ((1000 ms / FPS) * nEarlierSetts.getT1()(=3)) / 100 = 10 * nEarlierSetts.getT1() / FPS
	//					  0, 1, 0, 1.43f,	// y = y + dt * (v*cos(alfa))
	//					  0, 0, 1, 0,		// v*sin(alfa) = v*sin(alfa)
	//					  0, 0, 0, 1};		// v*cos(alfa) = v*cos(alfa)	

	public ref class MyKalmanFilter
	{
	private:
		cv::KalmanFilter* kf;
		cv::Mat* state;
		cv::Mat* processNoise;
		cv::Mat* measurement;
		double x;
		double y;

	public:
		MyKalmanFilter(void);
		void filter(double, double, double, double);
		double getX();
		double getY();

	private:
		void limit();
	};
}