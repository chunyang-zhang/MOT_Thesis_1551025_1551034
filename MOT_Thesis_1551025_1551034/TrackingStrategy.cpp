#include "TrackingStrategy.h"

TrackingStrategy::~TrackingStrategy()
{
	if (objectDetection!=NULL)
	{
		delete objectDetection;
	}
}

TrackingStrategy::TrackingStrategy():
ratio(2.0f)
{
	objectDetection = new YOLOObjectDetection();
}

TrackingStrategy::TrackingStrategy(int Id):
firstDetectedId(Id),ratio(2.0f)
{
	objectDetection = new YOLOObjectDetection();
}
