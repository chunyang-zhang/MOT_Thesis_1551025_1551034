#include "KalmanTracker.h"
using namespace cv;
KalmanTracker::KalmanTracker()
{
	initKF(StateType());
	timeSinceUpdate = 0;
	hits = 0;
	hitStreak = 0;
	age = 0;
	id = 0;
}
KalmanTracker::KalmanTracker(StateType initRect)
{
	initKF(initRect);
	timeSinceUpdate = 0;
	hits = 0;
	hitStreak = 0;
	age = 0;
	id = 0;
	//kfCount++;
}

KalmanTracker::~KalmanTracker()
{
	history.clear();
}

StateType KalmanTracker::predict()
{
	// predict
	Mat p = kf.predict();
	age += 1;

	if (timeSinceUpdate > 0)
		hitStreak = 0;
	
	timeSinceUpdate += 1;

	StateType predictBox = getRectXysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	history.push_back(predictBox);
	return history.back();
}

void KalmanTracker::update(StateType stateMat)
{
	timeSinceUpdate = 0;
	history.clear();
	hits += 1;
	hitStreak += 1;

	// measurement
	measurement.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	measurement.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	measurement.at<float>(2, 0) = stateMat.area();
	measurement.at<float>(3, 0) = stateMat.width / stateMat.height;

	// update
	kf.correct(measurement);
}

StateType KalmanTracker::getState()
{

	Mat s = kf.statePost;
	return getRectXysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
}

StateType KalmanTracker::getRectXysr(float cx, float cy, float s, float r)
{
	float w = sqrt(s * r);
	float h = s / w;
	float x = (cx - w / 2);
	float y = (cy - h / 2);

	if (x < 0 && cx > 0)
		x = 0;
	if (y < 0 && cy > 0)
		y = 0;

	return StateType(x, y, w, h);
}

int KalmanTracker::getTimeSinceUpdate()
{
	return timeSinceUpdate;
}

int KalmanTracker::getHits()
{
	return hits;
}

int KalmanTracker::getHitStreak()
{
	return hitStreak;
}

int KalmanTracker::getage()
{
	return age;
}

int KalmanTracker::getId()
{
	return id;
}

void KalmanTracker::initKF(StateType stateMat)
{
	int stateNum = 7;
	int measureNum = 4;
	kf = KalmanFilter(stateNum, measureNum, 0);

	measurement = Mat::zeros(measureNum, 1, CV_32F);

	kf.transitionMatrix = (Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1);

	setIdentity(kf.measurementMatrix);
	setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf.errorCovPost, Scalar::all(1));

	// initialize state vector with bounding box in [cx,cy,s,r] style
	kf.statePost.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	kf.statePost.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	kf.statePost.at<float>(2, 0) = stateMat.area();
	kf.statePost.at<float>(3, 0) = stateMat.width / stateMat.height;
}
