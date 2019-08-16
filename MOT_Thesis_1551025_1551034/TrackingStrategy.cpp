#include "TrackingStrategy.h"

TrackingStrategy::~TrackingStrategy()
{
	if (objectDetection!=NULL)
	{
		delete objectDetection;
	}
}

TrackingStrategy::TrackingStrategy():
ratio(2.0f),maxAge(3),trackingTime(0.0f),trackingCount(0),iouThreshold(0.25)
{
	objectDetection = new YOLOObjectDetection();
}

TrackingStrategy::TrackingStrategy(float iouThres):
iouThreshold(iouThres), ratio(2.0f), maxAge(3), trackingTime(0.0f), trackingCount(0)
{
}

TrackingStrategy::TrackingStrategy(int Id):
firstDetectedId(Id),ratio(2.0f),maxAge(3), trackingTime(0.0f), trackingCount(0),iouThreshold(0.25)
{
	objectDetection = new YOLOObjectDetection();
}

float TrackingStrategy::getTrackingTime()
{
	if (trackingCount > 0)
	{
		trackingTime /= (double)CLOCKS_PER_SEC;
		return trackingTime / trackingCount;
	}
	return 0.0f;
}
