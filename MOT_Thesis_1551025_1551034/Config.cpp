#include "Config.h"

// Camera config
int Config::priFPS = 40; //40

//! Config for detect feature algorithm
//! Config for tracking
int Config::maxFeatureTrack = 500; // maximum feature to track
int Config::threshFeatureTrack = 400;

//! stereo threshold for outliers rejection
double Config::rejectStereoThresh = 5; //0.05? //0.5?

// 300 150 50 2 0.0001 0.01 10 0.001
//! RANSAC for outliers reject monocular pose estimation
int Config::loopRANSAC = 100;			// best: 100
int Config::numPointsRANSAC = 2;       
double Config::threshErrRANSAC = 0.0001;    // best: 0.0001; // 0.0005 // now: 0.0007                
double Config::threshSumErrRANSAC = 0.03; //best: 0.01	// now: 0.026	//0.05: endless run
int Config::threshNumOfInliersRANSAC = 20;	//best: 20

float Config::threshQualityFeature = 0.0005; //0.0005 

//! IMU thresh
float Config::degreeThresh = 0.05;

//! mono threshold for outliers 
int Config::minFeatureLiveChange = 40;
int Config::minFeatureOutlierChange = 41;

// failure recovery
int Config::threshPointWarning = 50;
double Config::threshGammaMax = 1.1;//1.25;
double Config::threshGammaMin = 0.9;//0.75;

//
int  Config::MINIMUM_RANSAC_LOOP = 10;
int Config::Y_COORD_ADDON = 200;