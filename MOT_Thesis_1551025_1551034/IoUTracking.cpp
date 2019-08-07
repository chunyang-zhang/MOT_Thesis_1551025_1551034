#include "IoUTracking.h"
using namespace cv;
IoUTracking::IoUTracking(int firstDetectedId):TrackingStrategy(firstDetectedId),
	ratioThreshold(0.55f),countLost(0)
{

}

IoUTracking::~IoUTracking()
{

}

bool IoUTracking::update( cv::Mat& image, cv::Rect& bbox)
{

	Rect processBounding;
	Mat detectFrame;
	BoundingBox croppedBoxResult;
	//get small surrounding frame

	processBounding = boxHelper.getNewBoundingBox(bbox, ratio, image.rows, image.cols);

	//Surrounding Object For Detection

	detectFrame = image(processBounding);
	//Tracking
	//bool ok = tracker->update(image, bbox);
	//Detect


	bool checkDetect = objectDetection->objectDetect(detectFrame);
	int btmText = image.rows - 10;
	//Init Tracking Method
	//Tracking(Frame);
	//IoU
	//
	if (checkDetect)
	{
		cout << "Successfully detected" << endl;
		//get box with the same class Id with the original and also best IoU with original boundingbox
		if (countLost < 2)
		{
			cout << "Perform IoU" << endl;
			//get relating bounding box based on first detected Id
			checkDetect = objectDetection->getRelatedBoundingBox(firstDetectedId, bbox, processBounding, croppedBoxResult);
			if (checkDetect)
			{

				//0.5 if 0 or 0.27 if more than 1
				objectDetection->setIoUThreshold(1 / pow(ratioThreshold, countLost));
				//normalize to output
				bbox = boxHelper.normalizeCroppedBox(boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y), image.cols, image.rows);
				countLost = 0;
			}
			else
			{
				cout << "Fail to detect a object" << endl;
				//Reduce ratio threshold for next tracking
				if (countLost < 1)
				{
					objectDetection->setIoUThreshold(ratioThreshold);
				}
				countLost++;
			}

		}
		else
		{
			checkDetect = false;
		}
	}
	if (countLost >= 2)
	{
		cout << "Lost the object" << endl;
	}
	detectFrame.release();
	return checkDetect;
}


