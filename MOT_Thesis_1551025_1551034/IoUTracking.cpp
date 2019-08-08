#include "IoUTracking.h"
using namespace cv;
IoUTracking::IoUTracking(int firstDetectedId):TrackingStrategy(firstDetectedId),
	timeSinceUpdate(0),hitStreak(0)
{

}

IoUTracking::~IoUTracking()
{

}

void IoUTracking::update()
{
	timeSinceUpdate = 0;
	hitStreak++;
}

bool IoUTracking::predict()
{
	timeSinceUpdate += 1;
	if (timeSinceUpdate > 0)
	{
		hitStreak = 0;
	}
	return true;
}

bool IoUTracking::update( cv::Mat& image, cv::Rect& bbox)
{

	Rect processBounding;
	Mat detectFrame;
	BoundingBox croppedBoxResult;
	vector<BoundingBox> bboxList;
	vector<int> assignment;
	//get small surrounding frame
	vector<Rect> predictedBoxes;
	//iouMatrix
	vector<vector<double>> iouMatrix;
	vector<cv::Point> matchedPairs;
	unsigned int trkNum = 0;//l
	unsigned int detNum = 0;//l
	HungarianAlgorithm HungAlgo;
	int minHits = 0;
	float iouThreshold = 0.25f;

	processBounding = boxHelper.getNewBoundingBox(bbox, ratio, image.rows, image.cols);

	//Surrounding Object For Detection

	detectFrame = image(processBounding);
	//Tracking
	//bool ok = tracker->update(image, bbox);
	//Detect


	bool checkDetect = objectDetection->objectDetect(detectFrame);
	//Init Tracking Method
	//Tracking(Frame);
	//IoU
	//

	if (!checkDetect)
	{
		return false;
	}
	cout << "Successfully detected" << endl;
	objectDetection->getRelatedBoundingBoxes(bboxList, processBounding, firstDetectedId);
	if (bboxList.size() == 0)
	{
		return false;
	}
	clock_t start = clock();
	trackingCount++;
	//predicted box is the current box 
	predictedBoxes.push_back(bbox);
	predict();

	//2. associate detections to tracked object (from yolo to kalman)
	trkNum = predictedBoxes.size();
	detNum = bboxList.size();
	//iou Matrix of each tracking with full detection
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));
	//compute iou matrix
	for (int i = 0; i < trkNum; i++)
	{
		for (int j = 0; j < detNum;j++)
		{
			iouMatrix[i][j] = 1 - objectDetection->calculateIoU(predictedBoxes[i], bboxList[j].getRegion());
		}
	}
	//solve the assignment problem using hugarian algorithm
	//the result is [track:detection]
	HungAlgo.Solve(iouMatrix, assignment);

	//filter out matched with low IoU
	//only have 1 object tracked if cant track?
	for (int i = 0;i < trkNum;i++)
	{
		if (assignment[i] == -1)
		{
			cout << "No Matched pair" << endl;
			continue;
		}
		//iou small
		if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
		{
			cout << "No Matched pair" << endl;
		}
		else {
			matchedPairs.push_back(Point(i, assignment[i]));
		}
	}
	//3.3 update trackers?
	//should be only 1 match pair
	//If there is no matched pair -> time since update increase -> = 2 then lost track
	int detIdx = 0, trkIdx = 0;
	for (int i = 0;i < matchedPairs.size();i++)
	{
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		update();
		checkDetect = true;
		//if first time update and continously tracked hit >2 or hit streak just start
		//or it hit for the first two frame.
	}
	//fail when 
	if (timeSinceUpdate< 1 && hitStreak > minHits)//minHits || (*it).getHits() <= minHits))
	{
		bbox = boxHelper.normalizeCroppedBox(bboxList[detIdx].getRegion(), image.cols, image.rows); 
	}
	//allow maxAge frames lost or delete it.
	else if (timeSinceUpdate > maxAge)
	{
		//bbox = Rect(box);
		cout << "Time since update: " << timeSinceUpdate << endl;
		checkDetect = false;
	}
		//cout << "Successfully detected" << endl;
		////get box with the same class Id with the original and also best IoU with original boundingbox
		//if (countLost < 2)
		//{
		//	cout << "Perform IoU" << endl;
		//	//get relating bounding box based on first detected Id
		//	checkDetect = objectDetection->getRelatedBoundingBox(firstDetectedId, bbox, processBounding, croppedBoxResult);
		//	if (checkDetect)
		//	{

		//		//0.5 if 0 or 0.27 if more than 1
		//		objectDetection->setIoUThreshold(1 / pow(ratioThreshold, countLost));
		//		//normalize to output
		//		bbox = boxHelper.normalizeCroppedBox(boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y), image.cols, image.rows);
		//		countLost = 0;
		//	}
		//	else
		//	{
		//		cout << "Fail to detect a object" << endl;
		//		//Reduce ratio threshold for next tracking
		//		if (countLost < 1)
		//		{
		//			objectDetection->setIoUThreshold(ratioThreshold);
		//		}
		//		countLost++;
		//	}

		//}
		//else
		//{
		//	checkDetect = false;
		//}
	detectFrame.release();
	trackingTime += clock() - start;
	return checkDetect;
}


