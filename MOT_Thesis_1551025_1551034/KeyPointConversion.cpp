#include "KeyPointConversion.h"
using namespace cv;
vector<KeyPoint> KeyPointConversion::Point2f2KeyPoint(vector<Point2f>& points)
{
	//convert Point2f to KeyPoint
	vector<KeyPoint> keyPoints;
	Point2d tmp;
	for (size_t i = 0;i < points.size();i++)
	{
		tmp = points[i];
		keyPoints.push_back(KeyPoint(tmp, 1.f));
	}
	return keyPoints;
}

vector<Point2f> KeyPointConversion::KeyPoint2Point2f(vector<KeyPoint>& keyPoint)
{
	vector <Point2f> points;
	int size = keyPoint.size();
	for (int i = 0;i < size;i++)
	{
		points.push_back(keyPoint[i].pt);
	}
	return points;
}

Mat3x3 KeyPointConversion::inverseMat3x3(const Mat3x3 A)
{
	float c = A(0, 0) * A(2, 2) - A(2, 0) * A(0, 2) - A(1, 0) * A(0, 1) +
		A(1, 1) * A(2, 2) + A(0, 0) * A(1, 1) - A(1, 2) * A(2, 1) - 4;

	float lam1 = 2;
	float lam2 = 1 + sqrt(fabs(1 - c));
	float lam3 = 4 - lam1 - lam2;
	//inverse of determinant
	float inv_det = 1 / (lam1 * lam2 * lam3);

	Mat3x3 Ainv;
	Ainv(0, 0) = (-A(2, 1) * A(1, 2) + A(1, 1) * A(2, 2)) * inv_det;
	Ainv(0, 1) = (-A(0, 1) * A(2, 2) + A(0, 2) * A(2, 1)) * inv_det;
	Ainv(0, 2) = (A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1)) * inv_det;
	Ainv(1, 0) = (A(2, 0) * A(1, 2) - A(1, 0) * A(2, 2)) * inv_det;
	Ainv(1, 1) = (-A(2, 0) * A(0, 2) + A(0, 0) * A(2, 2)) * inv_det;
	Ainv(1, 2) = (A(1, 0) * A(0, 2) - A(0, 0) * A(1, 2)) * inv_det;
	Ainv(2, 0) = (-A(2, 0) * A(1, 1) + A(1, 0) * A(2, 1)) * inv_det;
	Ainv(2, 1) = (A(2, 0) * A(0, 1) - A(0, 0) * A(2, 1)) * inv_det;
	Ainv(2, 2) = (-A(1, 0) * A(0, 1) + A(0, 0) * A(1, 1)) * inv_det;

	return Ainv;
}

Mat3x3 KeyPointConversion::RPY2Rotation(float roll, float pitch, float yaw)
{
	// degree to radian
	float rollRad = (roll * M_PI) / 180.0;
	float pitchRad = (pitch * M_PI) / 180.0;
	float yawRad = (yaw * M_PI) / 180.0;

	Mat3x3 R;
	Mat3x3 Roll = Mat3x3::Identity();
	Mat3x3 Pitch = Mat3x3::Identity();
	Mat3x3 Yaw = Mat3x3::Identity();
	Roll(1, 1) = cos(rollRad);
	Roll(1, 2) = -sin(rollRad);
	Roll(2, 1) = sin(rollRad);
	Roll(2, 2) = cos(rollRad);

	Pitch(0, 0) = cos(pitchRad);
	Pitch(0, 2) = sin(pitchRad);
	Pitch(2, 0) = -sin(pitchRad);
	Pitch(2, 2) = cos(pitchRad);

	Yaw(0, 0) = cos(yawRad);
	Yaw(0, 1) = -sin(yawRad);
	Yaw(1, 0) = sin(yawRad);
	Yaw(1, 1) = cos(yawRad);

	R = Yaw * Pitch * Roll;

	return R.transpose();
}
