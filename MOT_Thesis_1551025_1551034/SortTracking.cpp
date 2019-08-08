#include "SortTracking.h"
using namespace cv;

SortTracking::SortTracking()
{
}

SortTracking::SortTracking(cv::Rect2f box,int firstDetectedId):TrackingStrategy(firstDetectedId)
{
	//init kalman tracker for first frame
	KalmanTracker trk = KalmanTracker(box);
	trackers.push_back(trk);
}
SortTracking::SortTracking(cv::Rect2f box)
{
	//init kalman tracker for first frame
	KalmanTracker trk = KalmanTracker(box);
	trackers.push_back(trk);
}
SortTracking::~SortTracking()
{

}
bool SortTracking::update( cv::Mat& image, cv::Rect& bbox)
{
	Rect processBounding = Rect(0,0,image.cols,image.rows);
	BoundingBox croppedBoxResult;
	//get small surrounding frame
	vector<BoundingBox> bboxList;
	int minHits =0;
	float iouTheshold = 0.25f;
	//use 
	vector<Rect_<float>> predictedBoxes;
	vector<vector<double>> iouMatrix;

	//Hungarian result
	vector<int> assignment;

	vector<cv::Point> matchedPairs;
	unsigned int trkNum = 0;//l
	unsigned int detNum = 0;//l
	HungarianAlgorithm HungAlgo;



	//Surrounding Object For Detection

	//Tracking
	//bool ok = tracker->update(image, bbox);
	//Detect

	//box from yolo
	bool checkDetect = objectDetection->objectDetect(image);
	//get box from yolo detections

	if (!checkDetect)
	{
		return checkDetect;
	}
	objectDetection->getRelatedBoundingBoxes(bboxList, processBounding, firstDetectedId);
	if (bboxList.size() == 0)
	{
		return false;
	}
	clock_t start = clock();
	trackingCount++;
	//get predicted location from existing trackers
	//only 1
	for (auto it = trackers.begin();it != trackers.end();) 
	{
		//get predicted box from kalman
		Rect_<float> pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			predictedBoxes.push_back(pBox);
			it++;
		}
		else//not available
		{
			it = trackers.erase(it);
		}
	}
	if (trackers.size() == 0)
	{
		trackingTime += clock() - start;
		return false;
	}
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
	int detIdx, trkIdx;
	for (int i = 0;i < matchedPairs.size();i++)
	{
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		trackers[trkIdx].update(bboxList[detIdx].getRegion());
	}
	int count = 0;
	//get tracker's output
	for (auto it = trackers.begin();it != trackers.end();)
	{
		//if first time update and continously tracked hit >2 or hit streak just start
		//or it hit for the first two frame.
		//fail when 
		if (((*it).getTimeSinceUpdate() < 1) &&
			((*it).getHitStreak() > minHits))//minHits || (*it).getHits() <= minHits))
		{
			bbox = boxHelper.normalizeCroppedBox((*it).getState(), image.cols, image.rows);
			it++;
			checkDetect = true;
		}
		//allow 2 frames lost or delete it.
		else if (it != trackers.end() && (*it).getTimeSinceUpdate() > maxAge)
		{
			it = trackers.erase(it);
			cout << "Time since update: " << (*it).getTimeSinceUpdate() << endl;
			checkDetect = false;
		}
		else
		{

			it++; //keep it maxAge more frame.
		}
	}

	trackingTime += clock() - start;
	return checkDetect;
}
