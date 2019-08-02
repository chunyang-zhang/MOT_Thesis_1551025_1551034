#include "FeatureDetectTrack.h"
using namespace cv;

FeatureDetectTrack::FeatureDetectTrack()
{
	//Initialize GFTT detector
	//detector =	GFTTDetector::create(Config::maxFeatureTrack,0.001,10);
	//detector = cv::FastFeatureDetector::create(10, true);
	//detector = SURF::create(300);
	//havent init brief and brute force yet
	//descriptor = xfeatures2d::BriefDescriptorExtractor::create();
	//orb = ORB::create(Config::maxFeatureTrack);
	//brisk = BRISK::create();
	//akaze = AKAZE::create();
	//detector = AgastFeatureDetector::create();
	//detector = MSER::create();
	//descriptor = DAISY::create();
	//descriptor = FREAK::create();
	//detector = StarDetector::create();
	//descriptor = LUCID::create();
	//descriptor = LATCH::create();
	//init a matcher should test using KNN?
	matcher = BFMatcher::create();
	//flann based created
	//matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

}

void FeatureDetectTrack::setDetectDescriptorMethod(string detectM, string descriptM)
{
	this->descriptM = descriptM;
	this->detectM = detectM;
	//Detector
	if (detectM.compare("gftt") == 0)
	{
		detector = GFTTDetector::create(Config::maxFeatureTrack, 0.001, 10);
	}
	else if(detectM.compare("agast") ==0)
	{
		agastDetector = AgastFeatureDetector::create();
	}
	else if (detectM.compare("mser") == 0)
	{
		mserDetector = MSER::create();
	}
	else if (detectM.compare("star") == 0)
	{
		starDetector = StarDetector::create();
	}
	else if (detectM.compare("orb") == 0)
	{
		orb = ORB::create();
	}
	else if (detectM.compare("fast") == 0)
	{
		fastDetector = FastFeatureDetector::create();
	}
	else if (detectM.compare("brisk") == 0)
	{
		brisk = BRISK::create();
	}
	//Descriptor
	if (descriptM.compare("brief") ==0)
	{
		briefDescriptor = xfeatures2d::BriefDescriptorExtractor::create();
	}
	else if (descriptM.compare("daisy") == 0)
	{
		daisyDescriptor = DAISY::create();
	}
	else if (descriptM.compare("latch") == 0)
	{
		latchDescriptor = LATCH::create();
	}
	else if (descriptM.compare("freak") == 0)
	{
		freakDescriptor = FREAK::create();
	}
}

vector<cv::KeyPoint> FeatureDetectTrack::detectKeyPoints(const cv::Mat& img, const cv::Mat& mask)
{
	vector<KeyPoint> keyPoints;
	//detector->detect(img, keyPoints, mask);
	if (detectM.compare("gftt") == 0)
	{
		detector->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("agast") == 0)
	{
		agastDetector->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("mser") == 0)
	{
		mserDetector->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("star") == 0)
	{
		starDetector->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("orb") == 0)
	{
		orb->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("fast") == 0)
	{
		fastDetector->detect(img, keyPoints, mask);
	}
	else if (detectM.compare("brisk") == 0)
	{
		brisk->detect(img, keyPoints, mask);
	}
	KeyPointsFilter::retainBest(keyPoints, Config::maxFeatureTrack);
	//detect keypoints
	return keyPoints;
}

Mat FeatureDetectTrack::computeDescriptors(const cv::Mat& img, vector<cv::KeyPoint>& keyPoints)
{
	Mat descriptors;
	//descriptor->compute(img, keyPoints, descriptors);
	if (descriptM.compare("brief") == 0)
	{
		briefDescriptor->compute(img, keyPoints, descriptors);
	}
	else if (descriptM.compare("daisy") == 0)
	{
		daisyDescriptor->compute(img, keyPoints, descriptors);
	}
	else if (descriptM.compare("latch") == 0)
	{
		latchDescriptor->compute(img, keyPoints, descriptors);
	}
	else if (descriptM.compare("freak") == 0)
	{
		freakDescriptor->compute(img, keyPoints, descriptors);
	}
	else if (descriptM.compare("orb") == 0)
	{
		orb->compute(img, keyPoints, descriptors);
	}
	else if (descriptM.compare("brisk")==0)
	{
		brisk->compute(img, keyPoints, descriptors);
	}
	//return descriptors of keypoints
	return descriptors;
}

vector<DMatch> FeatureDetectTrack::matchTwoImage(const cv::Mat& desc1, const cv::Mat& desc2)
{	
	vector<DMatch> goodMatches;

	//vector<DMatch> matches;
	matcher->match(desc1, desc2, goodMatches);
	//return matches of 2 descriptos
	//return matches;
	

	//matcher->match(desc1, desc2, goodMatches);
	//vector< vector<DMatch> > knn_matches;
	//matcher->knnMatch(desc1, desc2, knn_matches,2);
	////-- Filter matches using the Lowe's ratio test
	//const float ratio_thresh = 0.7f;
	//std::vector<DMatch> good_matches;
	//for (size_t i = 0; i < knn_matches.size(); i++)
	//{
	//	if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
	//	{
	//		good_matches.push_back(knn_matches[i][0]);
	//	}
	//}

	return goodMatches;
}

vector<cv::DMatch> FeatureDetectTrack::rejectStereoOutliers(vector<cv::KeyPoint>& keyPoints1, vector<cv::KeyPoint>& keyPoints2, vector<cv::DMatch>& matches, const Mat3x3& F)
{

	//matches after reject outliers
	vector<DMatch> bestMatches = vector<DMatch>();

	//number of matches
	int n = matches.size();
	if (n == 0) {
		return bestMatches;
	}
	Point3D tmp1;
	Point3D tmp2;
	//P2 * F(p1)
	double result;

	for (int i = 0;i < n;i++)
	{
		tmp1 = Point3D(keyPoints1[matches[i].queryIdx].pt.x, keyPoints1[matches[i].queryIdx].pt.y, 1);
		tmp2 = Point3D(keyPoints2[matches[i].trainIdx].pt.x, keyPoints2[matches[i].trainIdx].pt.y, 1);
		
		result = tmp2.transpose() * F * tmp1;//t2T *F & t1
		if (abs(result) < Config::rejectStereoThresh)//5?
		{
			bestMatches.push_back(matches[i]);
		}
	}
	return bestMatches;
}

vector<cv::Point2f> FeatureDetectTrack::detectAddFeature(const cv::Mat& img, const cv::Mat& mask, int& numAddFeature)
{
	vector<Point2f> keyPoints;
	
	goodFeaturesToTrack(img, keyPoints, numAddFeature, 0.001, 10, mask);
	//return good keypoints to track
	return keyPoints;
}
