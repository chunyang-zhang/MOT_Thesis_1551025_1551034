#pragma once
#include"Utils.h"
#define CAMERA_1_HEIGHT  480     // height of main camera's frame
#define CAMERA_1_WIDTH   640     // width of main camera's frame
#define CAMERA_2_HEIGHT  480     // height of sub camera's frame
#define CAMERA_2_WIDTH   640     // width of sub camera's frame
#define BUFLEN 512  //Max length of buffer
#define PORT 2391  //The port on which to listen for incoming data
#define WRONG_RPY 2000	// wrong for roll, pitch, yaw

class Config
{
public:
	// Camera config
	static int priFPS;

	// Config for IMU-camera calibration

	// Config for detect feature algorithm
	//static char* featureDetector;
	//static char* featureExtractor;
	//static char* featureMatcher;

	// Config for tracking
	static int maxFeatureTrack;
	static int threshFeatureTrack;

	// mono threshold for outliers 
	static int minFeatureLiveChange;
	static int minFeatureOutlierChange;

	// stereo threshold for outliers rejection
	static double rejectStereoThresh;

	//! RANSAC for outliers reject monocular pose estimation
	static int loopRANSAC;          // RANSAC iterations
	static int numPointsRANSAC;     // num of points using for RANSAC
	static double threshErrRANSAC;        // threshold for choose inliers RANSAC
	static double threshSumErrRANSAC;     // err need to be choose
	static int threshNumOfInliersRANSAC;  // number of inliers need to be choose

	//! Monocular for feature pos estimation
	static float threshQualityFeature;  //! minimum quality of estimate feature's pos

	//! IMU thresh
	static float degreeThresh;

	//! failure recovery
	static int threshPointWarning;
	static double threshGammaMax;
	static double threshGammaMin;

};

