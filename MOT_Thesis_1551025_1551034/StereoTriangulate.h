#pragma once
#include"Types.h"
#include "CameraParamters.h"
class StereoTriangulate
{
public:
	StereoTriangulate(CameraParameters camParams);
	//Using Stereo SGBM algoritithm by modified H. Hirschmuller algorithm 
	void stereoDepthMapSGBM(const cv::Mat& img1, const cv::Mat&, cv::Mat_<cv::Vec3f>& xyz);
	//Triangulate opencv Point from keypoint 1 and 2
	Point3DVector opencvTriangulatePoints(vector<cv::KeyPoint>& keyPoints1, vector<cv::KeyPoint>& keyPoints2, vector<cv::DMatch>& stereoMatches);
	Point3DVector myTriangulatePoints(vector<cv::KeyPoint>& keyPoints1, vector<cv::KeyPoint>& keyPoints2, vector<cv::DMatch>& stereoMatches, vector <float>& qualityList);
	//Calculate stereo feature positions
	Point3D calculateStereoFeaturePos(const Point3D& firstObsv, const Point3D& secondObsv, float& quality);
	bool checkStereoValidPoint(const Point3D& point);

private:
	CameraParameters camParams;
	cv::Ptr<cv::StereoSGBM> sgbm;
};

