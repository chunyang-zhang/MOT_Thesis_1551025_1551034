#pragma once
#include "TrackingStrategy.h"
#include "Hungarian.h"
class KCFTracker :
	public TrackingStrategy
{
private:
	cv::Ptr<cv::Tracker> kcfTracker;
	int hitStreak;
	int timeSinceUpdate;
	bool predict(cv::Mat& image, cv::Rect2d& box);
	void updateInternal(cv::Mat&image, cv::Rect2d box);
public:
	KCFTracker();
	~KCFTracker();

	KCFTracker(cv::Mat& frame, cv::Rect& bbox,int firstDetectedId);

	// Inherited via TrackingStrategy
	virtual bool update(cv::Mat& image, cv::Rect& bbox) override;

};

