#pragma once
#include"Types.h"
class MotionCompensation
{
private:
	int blockSize;
	int acceptDistance;
	int blockRadius;
	int stepSize;
	vector<cv::Point2f> candidatePoints;
	vector<float>candidateDistances;
	bool validSearchKeyPoint(int width, int height,cv::Point searchLocation );
	float sumAroundPoint(const cv::Mat& frame, cv::Point& p);
	float euclideanDistance(cv::Point& p1, cv::Point& p2); 
public:
	MotionCompensation();
	MotionCompensation(int blockS);
	MotionCompensation(int blockS, int stepS);
	void setBlockSize(int blockS);
	void setBlockStepSize(int blockS, int stepS);
	//Perform block matching to find candiate motion point for preKeypoints.
	void performBlockMatching(cv::Mat& preFrame, cv::Mat& mainFrame, const vector<cv::Point2f>& preKeyPoints, vector<cv::Point2f>& candidateKeyPoint, vector<bool>& status);
	//if currKeyPoints = motion keypoints
	//compare with optical flow point?
	bool findMotionPoints(vector<cv::Point2f>& currKeyPoints, vector<uchar> status, vector<int>& assignment);
};

