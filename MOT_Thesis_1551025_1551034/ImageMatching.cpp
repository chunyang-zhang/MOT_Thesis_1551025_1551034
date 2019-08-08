#include "ImageMatching.h"
using namespace cv;

int ImageMatching::getBoundingBoxImageMatching(const cv::Mat& preBBoxFrame, const cv::Mat& detectFrame, vector<BoundingBox>& bboxList)
{
	const int MIN_MATCH_COUNT = 8;
	Mat objectFrame;
	vector<KeyPoint> keyPoints1;
	vector<KeyPoint> keyPoints2;
	Mat descriptor1;
	Mat descriptor2;
	vector<vector<DMatch> > knn_matches;
	const float ratio_thresh = 0.9f;
	Rect newBox;
	bool isDetect = false;
	for (size_t i = 0;i < bboxList.size();i++)
	{
		newBox = boxHelper.normalizeCroppedBox(bboxList[i].getRegion(), detectFrame.cols, detectFrame.rows);
		objectFrame = detectFrame(newBox);
		if (!isDetect)
		{
			detector->detect(preBBoxFrame, keyPoints1);
			if (keyPoints1.size() == 0)
			{
				continue;
			}
			briefDescriptor->compute(preBBoxFrame, keyPoints1, descriptor1);
			if (descriptor1.empty())
			{
				continue;
			}
			isDetect = true;
		}
		detector->detect(objectFrame, keyPoints2);
		if ( keyPoints2.size() == 0) {
			continue;
		}
		briefDescriptor->compute(objectFrame, keyPoints2, descriptor2);
		if (descriptor2.empty()) {
			continue;
		}
		matcher->knnMatch(descriptor1, descriptor2, knn_matches, 2);
		vector<DMatch> good_matches;
		for (size_t i = 0; i < knn_matches.size(); i++)
		{
			if (knn_matches[i].size() < 2)
			{
				continue;
			}
			if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
			{
				good_matches.push_back(knn_matches[i][0]);
			}
		}
		//Mat img_matches;
		//drawMatches(preBBoxFrame, keyPoints1, objectFrame, keyPoints2, good_matches, img_matches, Scalar::all(-1),
		//	Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		////-- Show detected matches
		//imshow("Good Matches", img_matches);
		cout << "Matching Result Value: " << good_matches.size() << endl;
		if (good_matches.size() >= MIN_MATCH_COUNT)
		{
			return i;
		}
	}
	//release Mat
	if (!objectFrame.empty())
	{
		objectFrame.release();
	}
	if (!descriptor1.empty())
	{
		descriptor1.release();
	}
	if (!descriptor2.empty())
	{
		descriptor2.empty();
	}
	return -1;
}

ImageMatching::ImageMatching()
{
	detector = GFTTDetector::create(Config::maxFeatureTrack, 0.001, 10);
	briefDescriptor = xfeatures2d::BriefDescriptorExtractor::create();
	matcher = BFMatcher::create();

}

ImageMatching::~ImageMatching()
{
	if(detector)
	{ 
		detector.release();
	}
	if (briefDescriptor)
	{
		briefDescriptor.release();
	}
	if (matcher)
	{
		matcher.release();
	}
}
