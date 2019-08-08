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
	TrackingStrategy(firstDetectedId),preBBoxFrame(preBBoxFrame),timeSinceUpdate(0)
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
	if (timeSinceUpdate > maxAge )
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
	if (!checkDetect)
	{
		return false;
	}
	cout << "Perform Image Matching" << endl;
	//get Bounding boxes with its original size
	objectDetection->getRelatedBoundingBoxes(bboxList,processBounding,firstDetectedId);
	if (bboxList.size() == 0)
	{
		return false;
	}
	clock_t start = clock();
	trackingCount++;

	int boxIndex = imageMatching->getBoundingBoxImageMatching(preBBoxFrame, image, bboxList);
	if (boxIndex == -1)
	{
		checkDetect = false;
		timeSinceUpdate++;
	}
	else
	{
		cout << "Succesfully matched~" << endl;
		croppedBoxResult = bboxList[boxIndex];
		//UPdate to the original bounding box
		bbox = boxHelper.normalizeCroppedBox(croppedBoxResult.getRegion(), image.cols, image.rows);
		checkDetect = true;
		preBBoxFrame.release();
		preBBoxFrame = image(bbox).clone();
		timeSinceUpdate = 0;
	}
	if (timeSinceUpdate > maxAge)
	{
		cout << "Lost the object" << endl;
	}
	detectFrame.release();
	trackingTime += clock() - start;
	return checkDetect;

}
