#include "ImageMatchingTracking.h"
using namespace cv;
ImageMatchingTracking::~ImageMatchingTracking()
{
	if (imageMatching != NULL)
	{
		delete imageMatching;
	}
	preBBoxFrame.release();
}

ImageMatchingTracking::ImageMatchingTracking(const cv::Mat& preBBoxFrame,int firstDetectedId):
	TrackingStrategy(firstDetectedId),preBBoxFrame(preBBoxFrame),countLost(0)
{
	imageMatching = new ImageMatching();
}

bool ImageMatchingTracking::update( cv::Mat& image, cv::Rect& bbox)
{
	Rect processBounding;
	Mat detectFrame;
	vector<BoundingBox> bboxList;
	BoundingBox croppedBoxResult;
	//Allow one time to be Lost 
	if (countLost >= 2 )
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


	bool checkDetect = objectDetection->objectDetect(detectFrame);
	cout << "Perform Image Matching" << endl;
	objectDetection->getAllBoundingBox(bboxList);
	int boxIndex = imageMatching->getBoundingBoxImageMatching(preBBoxFrame, detectFrame, firstDetectedId, bboxList);
	if (boxIndex == -1)
	{
		checkDetect = false;
		countLost++;
	}
	else
	{
		cout << "Successfully detected" << endl;
		croppedBoxResult = bboxList[boxIndex];
		//UPdate to the original bounding box
		bbox = boxHelper.normalizeCroppedBox(boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y), image.cols, image.rows);
		checkDetect = true;
		preBBoxFrame.release();
		preBBoxFrame = image(bbox).clone();
		countLost = 0;
	}
	if (countLost >= 2)
	{
		cout << "Lost the object" << endl;
	}
	detectFrame.release();
	return checkDetect;

}
