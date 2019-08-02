#pragma once
#include"Utils.h"
/*
Frame camera, and its roll pitch yaw from IMU
*/

class CamerasIMUFrame {
public:
	int id; // Frame ID number
	double timestamp; // Time stamp of current frame
	cv::Mat preMainFrame; // Previous main frame
	cv::Mat mainFrame; // mainFrame
	bool isStereo;  // If have two image from Stereo
	cv::Mat subFrame; // Img from Secondary camer
	// From IMU roll, pitch, yaw;
	float roll;
	float pitch;
	float yaw;
	//smart pointer of boost
	typedef boost::shared_ptr<CamerasIMUFrame> Ptr;
	void releaseFrame();
	~CamerasIMUFrame();
};
