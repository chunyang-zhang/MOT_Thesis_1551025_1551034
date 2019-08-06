#pragma once
#include"Types.h"
#include"YOLOObjectDetection.h"
#include "BoundingBoxHelper.h"
#include "BoundingBox.h"
class TrackingStrategy
{
protected:
	BoundingBoxHelper boxHelper;
	YOLOObjectDetection* objectDetection;
	int firstDetectedId;
	float ratio;
public:
	TrackingStrategy(int Id);
	virtual bool update(const cv::Mat &image, cv::Rect& bbox) = 0;
};

