#include "TrackingStrategy.h"

TrackingStrategy::TrackingStrategy(int Id):
firstDetectedId(Id),ratio(2.0f)
{
	objectDetection = new YOLOObjectDetection();
}
