#include "TrackingController.h"
using namespace cv;
TrackingStrategy* TrackingController::selectTrackingStrategy(string method, Mat& frame,Rect trackingBox, int firstDetectedId)
{
	if (method.compare("IoU") == 0)
	{
		trackingStrategy = new IoUTracking(firstDetectedId);
	}
	else if (method.compare("IoUMatching") == 0 )
	{
		Mat preBBoxFrame = frame(trackingBox);
		trackingStrategy = new IoUImageMatchingTracking(preBBoxFrame,firstDetectedId);
	}
	else if (method.compare("ImageMatching") == 0)
	{
		Mat preBBoxFrame = frame(trackingBox);
		trackingStrategy = new ImageMatchingTracking(preBBoxFrame,firstDetectedId);
	}
	else if (method.compare("Sort") == 0)
	{
		trackingStrategy = new SortTracking(trackingBox, firstDetectedId);
	}
	else if (method.compare("CSRT") == 0)
	{
		trackingStrategy = new CSRTTRacker(frame, trackingBox,firstDetectedId);
	}
	else if (method.compare("KCF") == 0)
	{
		trackingStrategy = new KCFTracker(frame, trackingBox,firstDetectedId);
	}
	else
	{
		cout << "Not available tracking technique!" << endl;
	}
	return trackingStrategy;
}
