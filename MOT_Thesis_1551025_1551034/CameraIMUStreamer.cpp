#include "CameraIMUStreamer.h"
using namespace cv;
CameraIMUStreamer::CameraIMUStreamer(string outputCamPos) {
	StereoCalibration stereoCabInfo;
	//read the stereo info for calibrating
	stereoCabInfo.readCalibrateInfo(camParams);
	//cout << camParams.cm1 << endl;
	initUndistortRectifyMap(camParams.cm1, camParams.d1, camParams.r1, camParams.p1, Size(CAMERA_1_HEIGHT, CAMERA_1_WIDTH), CV_32FC1, camParams.map1x, camParams.map1y);
	initUndistortRectifyMap(camParams.cm2, camParams.d2, camParams.r2, camParams.p2, Size(CAMERA_2_HEIGHT, CAMERA_2_WIDTH), CV_32FC1, camParams.map2x, camParams.map2y);
	frameCounter = 0;
	imgStreamId = 0;
	//init rpy at t0
	rollT0 = 0;
	pitchT0 = 0;
	yawT0 = 0;
	//enable IMU stream
	turnIMUStream = 1;
	//output gps and pose
	outGPSandPose.open(outputCamPos);
}
bool CameraIMUStreamer::connect()
{
	//path to main L and sub R camera
	pathL = "./DATA/image_00/data/*.png";
	pathR = "./DATA/image_01/data/*.png";
	//String pathTime = "./";
	//String pathTimeStamps = "";
	//load whole images folder
	canTrack = true;
	stopTrack = false;
	glob(pathL, fn1, false);
	glob(pathR, fn2, false);
	if (fn1.size() <= 0 || fn2.size() <= 0)
	{
		cout << "Data is not available. Plz try again";
		return false;
	}

	else {
		if (fn1.size() < fn2.size()) 
		{
			numFrames = fn1.size();
		}
		else
		{
			numFrames = fn2.size();
		}
	}

	return true;
}

void CameraIMUStreamer::disconnect()
{
	/*cap1.release();
	cap2.release();
	turnIMUStream = 0;*/
}

bool CameraIMUStreamer::read(CamerasIMUFrame::Ptr &frame)
{
	if (frameCounter >= numFrames)
	{
		return false;
	}
	while (imgStreamId == frameCounter)
	{
		cout << "wait Thread!" << endl;
	}
	//update public frame with
	frame->id = frameCounter;
	frame->timestamp = time;
	//Save curr main + sub Img
	//Release old Frame
	//frame->releaseFrame();


	mainImg.copyTo(frame->mainFrame);
	preMainImg.copyTo(frame->preMainFrame);
	subImg.copyTo(frame->subFrame);
	//Divide based on frame numbers
	//FPS = 25 but = 40?
	if (frameCounter % Config::priFPS == 0)
	{
		frame->isStereo = true;
	}
	else
	{
		frame->isStereo = false;
	}
	frameCounter++;
	return true;
}

void CameraIMUStreamer::threadReadImg()
{
	while (1) 
	{
		if ( imgStreamId== fn1.size())
		{
			cout << "Finish!" << endl;
			//close position file
			outGPSandPose.close();
			outObjectPose.close();
			break;
		}
		if (!canTrack)
		{
			cout << "Finish!" << endl;
			//close position file
			outGPSandPose.close();
			outObjectPose.close();

			break;
		}
		if (stopTrack)
		{
				cout << "Finish!" << endl;
				//close position file
				outGPSandPose.close();
				outObjectPose.close();

				break;
		}
		if (imgStreamId == frameCounter) 
		{
			//pre main Image = current Image
			if (!mainImg.empty())
			{
				swap(preMainImg, mainImg);
			}
			if (frameCounter == 2)
			{

			}
			//read images from 2 camera 
			Mat tmp1 = imread(fn1[frameCounter], 1);
			Mat tmp2 = imread(fn2[frameCounter], 1);
			//get time stamp: 2011-09-26 13:04:32.351950336
			//will it work?
			stringstream ss(fn1[frameCounter]);
			String token;
			getline(ss, token, '_'); //token 
			getline(ss, token, '_'); //token 
			getline(ss, token, '_'); //token 
			ss.str(token);
			//cout << token << endl;
			ss >> time;
			cout << time << endl;
			//convert RGB to Gray
			cvtColor(tmp1, mainImg, COLOR_BGR2GRAY);
			cvtColor(tmp2, subImg, COLOR_BGR2GRAY);
			//imshow("Temp1", mainImg);
			//imshow("Temp2", subImg);
			imgStreamId++;
			tmp1.release();
			tmp2.release();
		}
	}
}

void CameraIMUStreamer::setStopTrack(bool stopTrack)
{
	this->stopTrack = stopTrack;
}

void CameraIMUStreamer::setCanTrack(bool canTrack)
{
	this->canTrack = canTrack;
}

int CameraIMUStreamer::getNumFrames()
{
	return numFrames;
}

CameraIMUStreamer::~CameraIMUStreamer()
{
	mainImg.release();
	preMainImg.release();
	subImg.release();
}
