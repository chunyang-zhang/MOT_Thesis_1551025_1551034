#include "CSRTTRacker.h"
using namespace cv;
bool CSRTTRacker::predict(Mat& image, Rect2d& box)
{
	bool checkTrack = csrtTracker->update(image, box);
	timeSinceUpdate += 1;
	if (timeSinceUpdate > 0)
	{
		hitStreak = 0;
	}
	return checkTrack;
}
void CSRTTRacker::update()
{
	timeSinceUpdate = 0;
	hitStreak++;
}
CSRTTRacker::CSRTTRacker()
{
	csrtTracker = TrackerCSRT::create();
	timeSinceUpdate = 0;
	hitStreak = 0;
}

CSRTTRacker::~CSRTTRacker()
{
}

CSRTTRacker::CSRTTRacker(cv::Mat& frame, cv::Rect& bbox)
{
	csrtTracker = TrackerCSRT::create();
	csrtTracker->init(frame, bbox);
	timeSinceUpdate = 0;
	hitStreak = 0;
}

bool CSRTTRacker::update(cv::Mat& image, cv::Rect& bbox)
{
	Rect processBounding;
	BoundingBox croppedBoxResult;
	//get small surrounding frame
	vector<BoundingBox> bboxList;
	int maxAge = 4;
	int minHits = 0;

	float iouTheshold = 0.25f;
	//use 
	vector<Rect> predictedBoxes;
	vector<vector<double>> iouMatrix;

	//Hungarian result
	vector<int> assignment;
	//matches of detection and track
	vector<cv::Point> matchedPairs;
	unsigned int trkNum = 0;//l
	unsigned int detNum = 0;//l
	HungarianAlgorithm HungAlgo;

	//Convert to Box2D
	Rect2d box(bbox);

	//Surrounding Object For Detection

	//bool ok = tracker->update(image, bbox);
	//Detect

	//box from yolo
	bool checkDetect = objectDetection->objectDetect(image);
	////get box from yolo detections

	if (!checkDetect)
	{
		return checkDetect;
	}
	objectDetection->getAllBoundingBox(bboxList);
	bool checkTrack = predict(image, box);
	//if (checkTrack)
	//{
	//	bbox = Rect(box);
	//	return true;
	//}
	//return false;
	//get predicted location from existing trackers
	//only 1
	//csrt update tracking
	if (!checkTrack)
	{
		return false;
	}
	predictedBoxes.push_back(box);
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

	//there are unmatched detections (cant track, can detect)
	//With unmatched detections init trackers for it ?


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
		if (1 - iouMatrix[i][assignment[i]] < iouTheshold)
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
		bbox = bboxList[detIdx].getRegion();
	}
	//allow 2 frames lost or delete it.
	else if (timeSinceUpdate > maxAge)
	{
		//bbox = Rect(box);
		cout << "Time since update: " << timeSinceUpdate << endl;
		return false;

	}
	return true;
}
