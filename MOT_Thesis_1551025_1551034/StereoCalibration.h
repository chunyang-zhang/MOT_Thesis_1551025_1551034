#pragma once
#include "utils.h"
#include "CameraParamters.h"
class StereoCalibration
{
public: 
	StereoCalibration();
	void readCalibrateInfo(CameraParameters& camParams);
};

