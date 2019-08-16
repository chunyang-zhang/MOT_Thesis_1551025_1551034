#include "IoUImageMatchingTracking.h"
using namespace cv;
void IoUImageMatchingTracking::update()
{
	timeSinceUpdate = 0;
	hitStreak++;
}
bool IoUImageMatchingTracking::predict()
{
	timeSinceUpdate += 1;
	if (timeSinceUpdate > 0)
	{
		hitStreak = 0;
	}
	return true;
}
IoUImageMatchingTracking::IoUImageMatchingTracking(const cv::Mat& preBBoxFrame, int firstDetectedId):TrackingStrategy(firstDetectedId),
preBBoxFrame(preBBoxFrame),hitStreak(0),timeSinceUpdate(0)
{
	imageMatching = new ImageMatching();

}
IoUImageMatchingTracking::~IoUImageMatchingTracking()
{
	if (imageMatching != NULL)
	{
		delete imageMatching;
	}
	preBBoxFrame.release();
}

bool IoUImageMatchingTracking::update(cv::Mat& image, cv::Rect& bbox)
{
	//Find bounding to track
	//Full Frame Image matching
	Rect processBounding;
	Mat detectFrame;
	BoundingBox croppedBoxResult;
	vector<BoundingBox> bboxList;
	vector<int> assignment;
	//get small surrounding frame
	vector<Rect> predictedBoxes;
	int minHits = 0;
	//iouMatrix
	vector<vector<double>> iouMatrix;
	vector<cv::Point> matchedPairs;
	unsigned int trkNum = 0;//l
	unsigned int detNum = 0;//l
	HungarianAlgorithm HungAlgo;

	if (timeSinceUpdate > maxAge)
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
	if (!checkDetect)
	{
		return false;
	}
	cout << "Successfully detected" << endl;
	//get bouding boxes with its original size
	objectDetection->getRelatedBoundingBoxes(bboxList, processBounding, firstDetectedId);
	if (bboxList.size() == 0)
	{
		cout << "Cant detect any object" << endl;
		return false;
	}
	clock_t start = clock();
	trackingCount++;

	predictedBoxes.push_back(bbox);
	predict();

	cout << "Perform IoU" << endl;

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
		//if first time update and continously tracked hit >2 or hit streak just start
		//or it hit for the first two frame.
	}
	//fail when 
	if (timeSinceUpdate< 1 && hitStreak > minHits)//minHits || (*it).getHits() <= minHits))
	{
		bbox = boxHelper.normalizeCroppedBox(bboxList[detIdx].getRegion(),image.cols,image.rows);
		checkDetect = true;
		preBBoxFrame.release();
		preBBoxFrame = image(bbox).clone();
	}
	else if (timeSinceUpdate > maxAge)
	{
		checkDetect = false;
	}

	//If can detect object but tim since last update large, or cannot detect
	if (!checkDetect)
	{
		cout << "Perform matching since IoU not working" << endl;
		//get box by image matching.
		int boxIndex = imageMatching->getBoundingBoxImageMatching(preBBoxFrame, image, bboxList);
		if (boxIndex == -1)
		{
			cout << "Time since update: " << timeSinceUpdate << endl;
			checkDetect = false;
		}
		else
		{
			cout << "Succesfully matched~" << endl;
			croppedBoxResult = bboxList[boxIndex];
			bbox = boxHelper.normalizeCroppedBox(croppedBoxResult.getRegion(), image.cols, image.rows);
			checkDetect = true;
			preBBoxFrame.release();
			preBBoxFrame = image(bbox).clone();
			update();
		}
	}

	detectFrame.release();
	trackingTime += clock() - start;
	return checkDetect;
}


