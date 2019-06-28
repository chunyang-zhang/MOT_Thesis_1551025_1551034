#include "StereoCalibration.h"
using namespace cv;
StereoCalibration::StereoCalibration()
{
}

void StereoCalibration::readCalibrateInfo(CameraParameters& camParams)
{
	FileStorage fload("mystereocalib - Copy.yml", FileStorage::READ);
	if (!fload.isOpened())
	{
		cout << "File is not valid";
		waitKey(0);
	}
	fload["CM1"] >> camParams.cm1;
	fload["CM2"] >> camParams.cm2 ;
	fload["D1"] >> camParams.d1;
	fload["D2"] >> camParams.d2;
	fload["R"] >> camParams.r;
	fload["T"] >> camParams.t;
	fload["E"] >> camParams.e;
	fload["F"] >> camParams.f;
	fload["R1"] >> camParams.r1;
	fload["R2"] >> camParams.r2;
	fload["P1"] >> camParams.p1;
	fload["P2"] >> camParams.p2;
	fload["Q"] >> camParams.q;
	fload.release();

	Eigen::Matrix3d F_; //eigen vector 3x3
	cv2eigen(camParams.f, F_); //
	camParams.F_f = F_.cast<float>();

	cout << "Calibrated!!!";
}
