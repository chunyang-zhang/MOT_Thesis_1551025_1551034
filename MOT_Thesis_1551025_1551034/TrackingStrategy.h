#pragma once
#include"Types.h"
#include"YOLOObjectDetection.h"
#include "BoundingBoxHelper.h"
#include "BoundingBox.h"
#include "Hungarian.h"
class TrackingStrategy
{
protected:
	BoundingBoxHelper boxHelper;
	YOLOObjectDetection* objectDetection;
	int firstDetectedId;
	float ratio;
	int maxAge;
	float trackingTime;
	int trackingCount;
	float iouThreshold;
public:
	~TrackingStrategy();
	TrackingStrategy();
	TrackingStrategy(float iouThres);
	TrackingStrategy(int Id);
	float getTrackingTime();
	virtual bool update(cv::Mat &image, cv::Rect& bbox) = 0;
};

