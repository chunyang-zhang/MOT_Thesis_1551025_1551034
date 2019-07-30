#pragma once
#include"Config.h"
#include"Types.h"
#include"BoundingBox.h"
using cv::xfeatures2d::BriefDescriptorExtractor;
using namespace cv::xfeatures2d;
class ImageMatching
{
private:
	cv::Ptr<cv::Feature2D> detector;
	cv::Ptr<BriefDescriptorExtractor> briefDescriptor;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	//Ptr<DescriptorMatcher> matcher;
	cv::Rect normalizeCroppedBox(cv::Rect oriBox, float width, float height);
public:
	int getBoundingBoxImageMatching(const cv::Mat& preBBoxFrame, const cv::Mat& detectFrame, int firstDetectedId,vector<BoundingBox>& bboxList);
	ImageMatching();
};

