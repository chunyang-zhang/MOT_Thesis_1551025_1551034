#pragma once

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Types.h"
using namespace std;
#define StateType cv::Rect_<float>

class KalmanTracker
{
private:
	void initKF(StateType stateMat);

	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<StateType> history;

	int timeSinceUpdate;
	int hits;
	int hitStreak;
	int age;
	int id;
public:
	~KalmanTracker();
	KalmanTracker(StateType initRect);
	KalmanTracker();
	StateType predict();
	void update(StateType stateMat);

	StateType getState();
	StateType getRectXysr(float cx, float cy, float s, float r);

	int getTimeSinceUpdate();
	int getHits();
	int getHitStreak();
	int getage();
	int getId();
};