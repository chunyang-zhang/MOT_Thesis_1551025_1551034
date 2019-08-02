#pragma once
#include<string>
#include"Types.h"
using namespace std;
class TrackingGroundTruth
{
private:
	int frameId;
	int id;
	string name;
	cv::Rect bbox;
	Point3D pos;
public:
	TrackingGroundTruth(int frameId, int index, string name, cv::Rect bbox, Point3D pos);
	int getId();
	string getName();
	int getFrameId();
	cv::Rect getBoundingBox();
	Point3D getObjectPos();
};

