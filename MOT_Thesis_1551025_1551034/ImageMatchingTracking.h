#pragma once
#include "TrackingStrategy.h"
#include "ImageMatching.h"
class ImageMatchingTracking :
	public TrackingStrategy
{
private:
	//Tracking lost to change the state.
	int countLost;
	ImageMatching* imageMatching;
	cv::Mat preBBoxFrame;
public:
	ImageMatchingTracking(const cv::Mat& preBBoxFrame,int firstDetectedId);
	~ImageMatchingTracking();
	// Inherited via TrackingStrategy
	virtual bool update(const cv::Mat& image, cv::Rect& bbox ) override;

};

