#pragma once
#include"ImageMatchingTracking.h"
#include"IoUImageMatchingTracking.h"
#include"IoUTracking.h"
#include"SortTracking.h"
#include"TrackingStrategy.h"
class TrackingController
{
private:
	TrackingStrategy* trackingStrategy;
public:
	TrackingStrategy* selectTrackingStrategy(string method,const cv::Mat& preBBoxFrame,cv::Rect trackingBox,int firstDetectedId);
};

