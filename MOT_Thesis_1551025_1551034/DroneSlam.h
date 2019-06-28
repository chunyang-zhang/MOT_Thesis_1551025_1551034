#pragma once
#include "InputStreamer.h"
#include "Types.h"
#include "FeatureDetectTrack.h"
#include "StereoTriangulate.h"
#include"CameraIMUStreamer.h"
#include"FeatureStorage.h"
#include"CameraParamters.h"
#include "KeyPointConversion.h"
#include<WinSock2.h>
#include<Eigen\src\Core\IO.h>
#include"YOLOObjectDetection.h"
#include"BoundingBoxHelper.h"
#include"BoundingBox.h"
#pragma comment(lib,"ws2_32.lib") //Winsock Library

class DroneSlam
{
	
private:
	CameraIMUStreamer* stream;
	CamerasIMUFrame::Ptr frame; //reference frame

	//Local Features Map
	FeatureStorageVector localMap;
	//lost track features
	FeatureStorageVector cullFeatures;
	//vector of camera positions 3D
	Point3DVector cameraPos;
	//FeaturePoints from previous frame
	vector<cv::Point2f> prevKeyP; //lastKeyP
	
	//current rotation matrix
	Mat3x3 currR;
	//Previous Rotation Matrix
	Mat3x3 prevR;
	//allR
	Mat3x3Vector allR;
	//last RPY to prevent orientation drift.
	RPY lastRPY;
	//save all RPY
	vector<RPY> allRPY;
	//current IMU id
	int curIMUId;
	// process module
	//Tracker and detect
	FeatureDetectTrack* detectAndTracker;
	//stereo image system to find the 3D pos
	StereoTriangulate* triangulateStereo;

	//Compare result vs GPS
	cv::Mat result1, result2, result3;

	bool isReinstall;

	//IMU begin RPY
	RPY beginRPY;
	Mat3x3 RW2IO;//?

	//gpsPose
	Point3DVector GPSPose;
	//ErrorPose
	Point3DVector errPose;
	//Keypoint Conversion helper
	KeyPointConversion keyPointConversion;
	YOLOObjectDetection* objectDetection;
private:
	//init map for stereo with keypoint, 3D local space, 
	void addFeaturesToLocalMap(vector<cv::KeyPoint>& keyPoints1, vector<cv::DMatch>& stereoMatches, Point3DVector& point3DStereo, vector<float>& allQuality,int frameId,const CameraParameters& camParams );
	//num of features < thresh -> track more and add to local map
	void addMoreFeaturesToLocalMap(const vector<cv::Point2f>& keyPoints1, int FrameId, const CameraParameters& camParams);

	//update map when have stereo
	void updateLocalMapStereo(Point3DVector& point3D, vector<float>& allQuality, vector<cv::DMatch>& map3Dto2D, Mat3x3& currR, Point3D& currCameraPos, vector<int>& mapKP2Local);
	//compute feature position important for object tracking this function.
	//using feature from cam1, cam2 
	Point3D calculateFeaturePos(const Point3D& firstCameraPos, const Point3D& firstObsv, const Point3D& secondCameraPos, const Point3D& secondObsv, float& quality, Mat3x3& R);
	//calculate Camera Pose based on preCameraPos, local map, numOfInliers
	Point3D calculateCameraPose(FeatureStorageVector& localMap, Point3D& preCameraPos, int& numOfInliers);
	//Using Ransac for CameraPos Filter
	Point3D ransac2PCameraPose(const Mat3x3Vector& allA, const Point3DVector& allAxp, const Point3DVector& ObsVectors, const Point3DVector& p3DFeature, vector<int>& resultInliers);
	//Find Inliers and Error 
	void computeErrAndInliers(const Point3D& cameraPose, const Mat3x3Vector& allA, const Point3DVector& allAxp, vector <int>& inliers, double& error);
	//important compute Pose
	Point3D computePose(const Mat3x3Vector& allA, const Point3DVector& allAxp, const vector <int>& points);
	//compute inliers and errors
	void computeErrAndInliers2(const Point3D& cameraPose, const Point3DVector& ObsVectors, const Point3DVector& p3DFeature, vector <int>& inliers, double& error);
	//Using curr KeyPoints and Pre Keypoints to remove outliers
	void checkAndDeleteCullTrack(vector <cv::Point2f>& currKeyP, vector <cv::Point2f>& preKeyP, vector <uchar>& status);
	//Remove outliers
	void deleteCullTrack(vector <cv::Point2f>& currKeyP);
	//From point to vector
	cv::Mat toFeatureObsv(vector<cv::Point2f> points);
	//reinstall Camera Pose Stereo
	Point3D reinstallCameraPoseStereo(FeatureStorageVector& localMap, Point3D& preCameraPos);
	// IMU reader
	bool readIMU(float& roll, float& pitch, float& yaw, double& time);
	bool readAllIMU();


	// compute rotation matrix from IMU
	Mat3x3 getRotationMatrix(float roll, float pitch, float yaw);
	 void drawGPSResult(cv::Mat& result);
public:
	DroneSlam();
	~DroneSlam();
	//main call this to process
	void processFrame();
};
