#include "TrackingController.h"
using namespace cv;
TrackingStrategy* TrackingController::selectTrackingStrategy(string method, const Mat& preBBoxFrame, int firstDetectedId)
{
	if (method.compare("IoU") == 0)
	{
		trackingStrategy = new IoUTracking(firstDetectedId);
	}
	else if (method.compare("IoUMatching") == 0 )
	{
		trackingStrategy = new IoUImageMatchingTracking(preBBoxFrame,firstDetectedId);
	}
	else if (method.compare("ImageMatching") == 0)
	{
		trackingStrategy = new ImageMatchingTracking(preBBoxFrame,firstDetectedId);
	}
	else
	{
		cout << "Not available tracking technique!" << endl;
	}
	return trackingStrategy;
}
