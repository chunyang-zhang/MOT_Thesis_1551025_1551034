#pragma once
#include "TrackingStrategy.h"
#include "Hungarian.h"
class IoUTracking :
	public TrackingStrategy
{
private:
	int hitStreak;
	int timeSinceUpdate;
	void update();
	bool predict();
public:
	// Inherited via TrackingStrategy
	bool update( cv::Mat& image, cv::Rect& bbox) override;
	IoUTracking(int firstDetectedId);
	~IoUTracking();
};

