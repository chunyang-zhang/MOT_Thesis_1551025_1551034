#pragma once
#include "TrackingStrategy.h"
class IoUTracking :
	public TrackingStrategy
{
private:
	int countLost;
	float ratioThreshold;
public:
	// Inherited via TrackingStrategy
	bool update(const cv::Mat& image, cv::Rect& bbox) override;
	IoUTracking(int firstDetectedId);
	~IoUTracking();
};

