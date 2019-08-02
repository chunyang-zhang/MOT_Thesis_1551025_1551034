#pragma once
#include"Config.h"
#include"Types.h"

using cv::xfeatures2d::BriefDescriptorExtractor;
using namespace cv::xfeatures2d;
class FeatureDetectTrack
{
private:
	//cv smart pointer to deallocate memory
	cv::Ptr<cv::Feature2D> detector;
	cv::Ptr<StarDetector>starDetector;
	cv::Ptr<BriefDescriptorExtractor> briefDescriptor;
	cv::Ptr<DAISY> daisyDescriptor;
	cv::Ptr<cv::AgastFeatureDetector> agastDetector;
	cv::Ptr<cv::MSER> mserDetector;
	cv::Ptr<LATCH> latchDescriptor;
	cv::Ptr<cv::Feature2D> descriptor;
	cv::Ptr<FREAK> freakDescriptor;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	cv::Ptr<cv::ORB> orb;
	cv::Ptr<cv::FastFeatureDetector> fastDetector;
	cv::Ptr<cv::BRISK> brisk;
	string detectM;
	string descriptM;
public:
	FeatureDetectTrack();
	void setDetectDescriptorMethod(string detectM, string descriptM);
	//detect Key Points from image
	vector<cv::KeyPoint> detectKeyPoints(const cv::Mat& img, const cv::Mat& mask);
	
	//compute Description of keyPoints on image
	cv::Mat computeDescriptors(const cv::Mat&img, vector<cv::KeyPoint> &keyPoints);

	//match two image with their coressponding keypoints descriptors
	vector<cv::DMatch> matchTwoImage(const cv::Mat& desc1, const cv::Mat& desc2);
	
	//reject outlier of stereo correspondence with epipolar constraints
	vector<cv::DMatch> rejectStereoOutliers(vector<cv::KeyPoint>& keyPoints1, vector<cv::KeyPoint>& keyPoints2, vector<cv::DMatch>& matches, const Mat3x3& F);

	//add feature to image
	vector<cv::Point2f> detectAddFeature(const cv::Mat& img, const cv::Mat& mask, int& numAddFeature);
};

