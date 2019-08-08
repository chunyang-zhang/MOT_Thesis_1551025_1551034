#pragma once
#include"ImageMatchingTracking.h"
#include"IoUImageMatchingTracking.h"
#include"IoUTracking.h"
#include"SortTracking.h"
#include"CSRTTRacker.h"
#include"KCFTracker.h"
#include"TrackingStrategy.h"

class TrackingController
{
private:
	TrackingStrategy* trackingStrategy;
public:
	TrackingStrategy* selectTrackingStrategy(string method, cv::Mat& preBBoxFrame,cv::Rect trackingBox,int firstDetectedId);
};

