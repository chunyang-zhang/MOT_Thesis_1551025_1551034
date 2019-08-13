#include "MotionCompensation.h"
using namespace cv;
bool MotionCompensation::validSearchKeyPoint(int width, int height, Point searchLocation)
{
	int indent = blockSize + blockRadius;
	if (searchLocation.x - indent<0)
	{
		return false;
	}
	if (searchLocation.y - indent < 0)
	{
		return false;
	}
	if (searchLocation.x + indent >= width)
	{
		return false;
	}
	if (searchLocation.y + indent >= height)
	{
		return false;
	}
	return true;
}

float MotionCompensation::sumAroundPoint(const cv::Mat& frame, cv::Point& p)
{
	Mat block = frame(Rect(p.x - blockRadius, p.y - blockRadius, blockSize, blockSize));
	return sum(block)[0];
}

float MotionCompensation::euclideanDistance(cv::Point& p1, cv::Point& p2)
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

MotionCompensation::MotionCompensation():
blockSize(16),acceptDistance(16),blockRadius(blockSize/2),stepSize(blockSize/4)
{

}

MotionCompensation::MotionCompensation(int blockS)
{
	blockSize = blockS;
	if (blockSize < 4 ||blockSize % 4 != 0)
	{
		blockSize = 4;
	}
	acceptDistance = blockSize;
	blockRadius = blockSize / 2;
	stepSize = blockSize / 4;
}
MotionCompensation::MotionCompensation(int blockS,int stepS)
{
	blockSize = blockS;
	if (blockSize < 4 || blockSize % 4 != 0)
	{
		blockSize = 4;
	}
	acceptDistance = blockSize;
	blockRadius = blockSize / 2;
	stepSize = stepS;
}

void MotionCompensation::setBlockSize(int blockS)
{
	blockSize = blockS;
	if (blockSize < 4 || blockSize % 4 != 0)
	{
		blockSize = 4;
	}
	acceptDistance = blockSize;
	blockRadius = blockSize / 2;
	stepSize = blockSize / 4;
}

void MotionCompensation::setBlockStepSize(int blockS, int stepS)
{
	blockSize = blockS;
	if (blockSize < 4 || blockSize % 4 != 0)
	{
		blockSize = 4;
	}
	acceptDistance = blockSize;
	blockRadius = blockSize / 2;
	stepSize = stepS;
}

void MotionCompensation::performBlockMatching(cv::Mat& preFrame, cv::Mat& mainFrame, const vector<cv::Point2f>& preKeyPoints, vector<cv::Point2f>& candidateKeyPoint, vector<bool>& status)
{
	candidateKeyPoint.clear();
	status.clear();
	candidateDistances.clear();
	candidatePoints.clear();
	Point searchLocation;
	int width = preFrame.cols;
	int height = preFrame.rows;
	int searchRange = blockSize;
	float sumOfSearchBlock = 0;
	float sumOfCandiateBlock = 0;
	float blockDiff;
	Point candidatePoint;
	Point bestLocation;
	float bestCost;
	float candidateDistance = 0;
	//find the big block distance range for comparison.
	Point point1 = cv::Point(0, 0);
	Point point2 = cv::Point((searchRange * 2) + 1, (searchRange * 2) + 1);
	float maxDistance = euclideanDistance(point1, point2);
	float closestMatch;
	int count = 0;
	for (size_t i = 0; i < preKeyPoints.size(); i++)
	{
		bestCost = 9999999;
		searchLocation = preKeyPoints[i];
		bestLocation = searchLocation;
		closestMatch = maxDistance;

		bool check = validSearchKeyPoint(width, height, searchLocation);
		if (!check)
		{
			status.push_back(false);
			candidateKeyPoint.push_back(bestLocation);
			candidateDistances.push_back(closestMatch);
			continue;
		}
		sumOfSearchBlock = sumAroundPoint(preFrame, searchLocation);
		for (int i = -searchRange; i <= searchRange; i+=stepSize) {
			for (int j = -searchRange; j <= searchRange; j+=stepSize)
			{
				count++;
				candidatePoint = Point(searchLocation.x + i, searchLocation.y + j);
				sumOfCandiateBlock = sumAroundPoint(mainFrame, candidatePoint);
				blockDiff = abs(sumOfCandiateBlock - sumOfSearchBlock);
				candidateDistance = euclideanDistance(candidatePoint, searchLocation);
				//if the cost is smaller or sam cost but smaller distance from pre Keypoint.
				if (blockDiff < bestCost || (blockDiff == bestCost && candidateDistance < closestMatch))
				{
					bestCost = blockDiff;
					bestLocation = candidatePoint;
					closestMatch = candidateDistance;
				}
			}
		}
		//check to choose suitable keypoint
		if (closestMatch > acceptDistance)
		{
			candidateKeyPoint.push_back(bestLocation);
			candidateDistances.push_back(closestMatch);
			status.push_back(false);
		}
		else {
			candidateKeyPoint.push_back(bestLocation);
			candidateDistances.push_back(closestMatch);
			status.push_back(true);
		}
	}
	cout << "Total process" << count << endl;
	//save for internal used
	candidatePoints = candidateKeyPoint;
}


bool MotionCompensation::findMotionPoints(vector<cv::Point2f>& currKeyPoints,vector<uchar> status, vector<int>& assignment)
{
	assignment.clear();

	//because the currkeyPoint Size is the same with preKeyPoints size 
	for (size_t i = 0;i < currKeyPoints.size();i++)
	{
		if (!status[i])
		{
			continue;
		}
		Point p1 = candidatePoints[i];
		Point p2 = currKeyPoints[i];
		float eD = euclideanDistance(p1,p2);
		//The distance to the moving point is small and its moving too far away compare to other points
		if (eD < 2 && candidateDistances[i] >= acceptDistance)
		{
			assignment.push_back(i);
		}
	}
	return false;
}
