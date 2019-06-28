#pragma once
#include "Utils.h"
class BoundingBox
{
public:
	BoundingBox();
	void setClassId(int Id);
	void setConfidence(float confid);
	void setRegion(cv::Rect bbox);
	int getClassId();
	int getConfidence();
	cv::Rect getRegion();
private:
	int classId;
	float confidence;
	cv::Rect box;
};

