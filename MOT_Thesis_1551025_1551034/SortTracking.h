#pragma once
#include "TrackingStrategy.h"
#include "KalmanTracker.h"
#include "Hungarian.h"
class SortTracking :
	public TrackingStrategy
{
private:
	vector<KalmanTracker> trackers;
public:
	SortTracking();
	SortTracking(cv::Rect2f box, int firstDetectedId );
	SortTracking(cv::Rect2f box);
	~SortTracking();
	// Inherited via TrackingStrategy
	bool update(cv::Mat& image, cv::Rect& bbox) override;
};

