#include "TrackingStrategy.h"

TrackingStrategy::~TrackingStrategy()
{
	if (objectDetection!=NULL)
	{
		delete objectDetection;
	}
}

TrackingStrategy::TrackingStrategy():
ratio(2.0f),maxAge(3),trackingTime(0.0f),trackingCount(0)
{
	objectDetection = new YOLOObjectDetection();
}

TrackingStrategy::TrackingStrategy(int Id):
firstDetectedId(Id),ratio(2.0f),maxAge(3), trackingTime(0.0f), trackingCount(0)
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
