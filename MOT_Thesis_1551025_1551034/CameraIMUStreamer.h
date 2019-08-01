#pragma once
#include"InputStreamer.h"
#include"Types.h"
#include"IMUPackage.h"
#include"Config.h"

class CameraIMUStreamer:public InputStreamer
{
private:
	//videostream from camera 1
	cv::VideoCapture cap1;
	//video stream from camera2
	cv::VideoCapture cap2;
	
	//dỉrectory to image folder
	string pathL;
	string pathR;

	vector<cv::String> fn1, fn2;

	int numFrames;

	//initial rpy state
	float rollT0;
	float pitchT0;
	float yawT0;

	//Video stream thread img 
	cv::Mat mainImg, preMainImg, subImg;
	double time;
	int imgStreamId;
	bool turnIMUStream;
	bool canTrack;
	bool stopTrack;
public: 

	// Inherited via InputStreamer
	bool connect() ;

	void disconnect() ;

	bool read(CamerasIMUFrame::Ptr &frame) ;
	CameraIMUStreamer(string outputCamPos,string outputObjectPos);
	void threadReadImg();
	ofstream outGPSandPose;
	ofstream outObjectPose;

	void setStopTrack(bool stopTrack);
	void setCanTrack(bool canTrack);
	int getNumFrames();
};

