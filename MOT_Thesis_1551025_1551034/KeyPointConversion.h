#pragma once
#include "Types.h"
class KeyPointConversion
{
public:
	vector<cv::KeyPoint> Point2f2KeyPoint(vector<cv::Point2f>& points);
	vector<cv::Point2f> KeyPoint2Point2f(vector<cv::KeyPoint>& keyPoint);

	Mat3x3 inverseMat3x3(const Mat3x3 A);
	//convert rpy to rotation matrix
	Mat3x3 RPY2Rotation(float roll, float pitch, float yaw);

};

