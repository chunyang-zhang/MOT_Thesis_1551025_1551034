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
	void updateInternal(cv::Mat& image, cv::Rect2d box);
public:
	CSRTTRacker();
	~CSRTTRacker();
	CSRTTRacker(cv::Mat& frame,cv::Rect& bbox,int firstDetectedId );
	// Inherited via TrackingStrategy
	bool update(cv::Mat& image, cv::Rect& bbox) override;
	
};

