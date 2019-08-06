#include "IoUImageMatchingTracking.h"
using namespace cv;
IoUImageMatchingTracking::IoUImageMatchingTracking(const cv::Mat& preBBoxFrame, int firstDetectedId):TrackingStrategy(firstDetectedId),
preBBoxFrame(preBBoxFrame),countLost(0),ratioThreshold(0.55)
{
	imageMatching = new ImageMatching();

}
IoUImageMatchingTracking::~IoUImageMatchingTracking()
{
	if (imageMatching != NULL)
	{
		delete imageMatching;
	}
	if(objectDetection!=NULL)
	{ 
		delete objectDetection;

	}
	preBBoxFrame.release();
}

bool IoUImageMatchingTracking::update(const cv::Mat& image, cv::Rect& bbox)
{
	//Find bounding to track
	//Full Frame Image matching
	Rect processBounding;
	Mat detectFrame;
	BoundingBox croppedBoxResult;
	vector<BoundingBox> bboxList;
	if (countLost >= 2)
	{
		cout << "Perform Image Matching for full frame!" << endl;
		processBounding = Rect(0, 0, image.cols, image.rows);
	}
	else
	{
		processBounding = boxHelper.getNewBoundingBox(bbox, ratio, image.rows, image.cols);
	}
	//Surrounding Object For Detection

	detectFrame = image(processBounding);
	//Tracking
	//bool ok = tracker->update(image, bbox);
	//Detect


	bool checkDetect = objectDetection->objectDetect(detectFrame);
	//Tracking(Frame);
	//IoU
	if (checkDetect)
	{
		cout << "Successfully detected" << endl;
		
		if (countLost < 2)
		{
			cout << "Perform IoU" << endl;
			//get box with the same class Id with the original and also best IoU with original boundingbox
			//Perform IoU only when countLost<2
			checkDetect = objectDetection->getRelatedBoundingBox(firstDetectedId, bbox, processBounding, croppedBoxResult);
			if (checkDetect)
			{
				cout << "Successfully Detect a Object" << endl;
				objectDetection->setIoUThreshold(1 / pow(ratioThreshold, countLost));
				//Convert to original size
				//width height the same
				bbox = boxHelper.normalizeCroppedBox(boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y), image.cols, image.rows);
				
				preBBoxFrame.release();
				preBBoxFrame = image(bbox).clone();
				countLost = 0;

			}

		}
		//If can detect object but count lost, or cannot detect
		if ((checkDetect && countLost >= 2) || !checkDetect)
		{

			objectDetection->getAllBoundingBox(bboxList);
			cout << "Perform matching since IoU not working" << endl;
			int boxIndex = imageMatching->getBoundingBoxImageMatching(preBBoxFrame, detectFrame, firstDetectedId, bboxList);
			if (boxIndex == -1)
			{
				checkDetect = false;
			}
			else
			{
				croppedBoxResult = bboxList[boxIndex];
				bbox = boxHelper.normalizeCroppedBox(boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y), image.cols, image.rows);
				preBBoxFrame.release();
				preBBoxFrame = image(bbox).clone();
				checkDetect = true;

				countLost = 0;
			}
		}
		//onlyy reduce the IoU threshold after ImageMatching phase.
		if(!checkDetect)
		{
			cout << "Fail to detect a object" << endl;
			if (countLost < 1)
			{
				objectDetection->setIoUThreshold(ratioThreshold);
			}
			countLost++;
		}
	}
	if (countLost >= 2)
	{
		cout << "Lost the object" << endl;
	}
	return checkDetect;
}


