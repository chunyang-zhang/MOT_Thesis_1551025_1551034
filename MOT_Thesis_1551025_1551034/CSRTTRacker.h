#pragma once
#include "TrackingStrategy.h"
#include "Types.h"
class CSRTTRacker :
	public TrackingStrategy
{
private:
	cv::Ptr<cv::Tracker > csrtTracker;
	int hitStreak;
	int timeSinceUpdate;
	bool predict(cv::Mat& image, cv::Rect2d& box);
	void update();
public:
	CSRTTRacker();
	~CSRTTRacker();
	CSRTTRacker(cv::Mat& frame,cv::Rect& bbox );
	// Inherited via TrackingStrategy
	bool update(cv::Mat& image, cv::Rect& bbox) override;
	
};

