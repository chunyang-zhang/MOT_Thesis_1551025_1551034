#include "BoundingBox.h"
using namespace cv;
BoundingBox::BoundingBox()
{
}
void BoundingBox::setClassId(int Id)
{
	classId = Id;
}

void BoundingBox::setConfidence(float confid)
{
	confidence = confid;
}

void BoundingBox::setRegion(Rect bbox)
{
	box = bbox;
}

int BoundingBox::getClassId()
{
	return classId;
}

int BoundingBox::getConfidence()
{
	return confidence;
}

cv::Rect BoundingBox::getRegion()
{
	return box;
}
