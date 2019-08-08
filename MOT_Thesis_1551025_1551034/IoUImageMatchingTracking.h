#pragma once
#include "TrackingStrategy.h"
#include"ImageMatching.h"
class IoUImageMatchingTracking :
	public TrackingStrategy
{
private:
	int timeSinceUpdate;
	int hitStreak;
	ImageMatching*imageMatching;
	cv::Mat preBBoxFrame;
	void update();
	bool predict();
public:
	IoUImageMatchingTracking(const cv::Mat& preBBoxFrame, int firstDetectedId);
	~IoUImageMatchingTracking();
	// Inherited via TrackingStrategy
	bool update(cv::Mat& image, cv::Rect& bbox) override;
};

