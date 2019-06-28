#pragma once
//class to save one feature
#include"Types.h"
class FeatureStorage
{
public:
	int firstFrameID; //first Frame ID
	Point3DVector featureObservation; //rotated frame
	int poseStereoQuality; // quality of stereo pose
	Point3D featurePoseStereo; //feature pose by stereo 
	Point3D featurePoseMono; //feature pose by monocular system
	bool isStereoPose; //check if feature correspon
	bool isMonoPose;		//check whether feature has position from monocular triangulation
	bool isCull;			//check whether feature lost track
	bool isInliers;			//feature is inlier?
	int nOutliers;			//Outliers?
};

//Feature storage vector
typedef vector<FeatureStorage, aligned_allocator<FeatureStorage>> FeatureStorageVector;