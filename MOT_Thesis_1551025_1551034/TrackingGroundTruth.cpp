#include "TrackingGroundTruth.h"

TrackingGroundTruth::TrackingGroundTruth(int frameId, int index, string name, cv::Rect bbox, Point3D pos):
	frameId(frameId),id(index),name(name),bbox(bbox),pos(pos)
{
}

TrackingGroundTruth::TrackingGroundTruth()
{
}

TrackingGroundTruth::TrackingGroundTruth(string name, cv::Rect bbox, Point3D pos):
	name(name),bbox(bbox),pos(pos)
{
}

int TrackingGroundTruth::getId()
{
	return id;
}

string TrackingGroundTruth::getName()
{
	return name;
}

int TrackingGroundTruth::getFrameId()
{
	return frameId;
}

cv::Rect TrackingGroundTruth::getBoundingBox()
{
	return bbox;
}

Point3D TrackingGroundTruth::getObjectPos()
{
	return pos;
}

void TrackingGroundTruth::setName(string n)
{
	name = n;
}

void TrackingGroundTruth::setBoundingBox(cv::Rect box)
{
	bbox = box;
}

void TrackingGroundTruth::setObjectPos(Point3D objPose)
{
	pos = objPose;
}
