#pragma once
#include "Types.h"


class CameraParameters {
public: 
	cv::Mat cm1, cm2; //matrix frame of camera 1 and 2
	cv::Mat invMat1;  //inverse of frame 1 for caculating feature extraction 
	cv::Mat d1, d2; //Distortion coefficient
	cv::Mat r, t; //rotation and translation from cm1 -> cm2
	cv::Mat e, f; //essential matrix, fundamental matrix?
	cv::Mat r1, r2; //rectification transform (rotation matrix for C1, C2)?
	cv::Mat p1, p2; //projection matrix in the new (rectified) coordinate system for c1,c2
	cv::Mat q;		//Disparity-to-depth mapping matrix?
	cv::Mat map1x, map1y, map2x, map2y; //undistortion and rectification transformation map
	Mat3x3 F_f; //Fundamental matrix
	~CameraParameters();
//public:
//
//	void setCM1(const Mat& cm1);
//	Mat getCM1();
//
//	void setCM2(const Mat& cm1);
//	Mat getCM2();
//
//	void setInvMat1(const Mat& invMat1);
//	Mat getInvMat1();
//
//	void setD1(const Mat& cm1);
//	Mat setD1();
//
};