#pragma once
#include "Utils.h"
#include "StereoCalibration.h"
#include "CamerasIMUFrame.h"

class InputStreamer
{
public:
	InputStreamer();
	//Connect stream to the source of RGB-D frames
	virtual bool connect() = 0;
	//Disconnect stream from source;
	virtual void disconnect() = 0;
	//Read a new RGB-D frame from source 
	virtual bool read(CamerasIMUFrame::Ptr &frame) = 0;

	//Intrinsic Parameter
	CameraParameters getCamParams();
protected:
	//Intrinsic Parameter
	CameraParameters camParams;
	bool isConnected;
	int frameCounter;
};

