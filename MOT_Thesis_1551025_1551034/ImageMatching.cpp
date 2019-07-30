#include "ImageMatching.h"
using namespace cv;

cv::Rect ImageMatching::normalizeCroppedBox(cv::Rect oriBox, float width, float height)
{
	int x = oriBox.x;
	int y = oriBox.y;
	int bWidth = oriBox.width;
	int bHeight = oriBox.height;
	int right = x +  bWidth;
	int bottom = y + bHeight;
	//width, height is max of the dimension of Mat
	if (right > width)
	{
		bWidth = width - x -1;
	}
	if (bottom > height)
	{
		bHeight = height - y- 1;
	}
	Rect newBox(x, y, bWidth, bHeight);
	return newBox;
}

int ImageMatching::getBoundingBoxImageMatching(const cv::Mat& preBBoxFrame, const cv::Mat& detectFrame,int firstDetectedId, vector<BoundingBox>& bboxList)
{
	const int MIN_MATCH_COUNT = 10;
	Mat objectFrame;
	vector<KeyPoint> keyPoints1;
	vector<KeyPoint> keyPoints2;
	Mat descriptor1;
	Mat descriptor2;
	vector<vector<DMatch> > knn_matches;
	const float ratio_thresh = 0.9f;
	Rect newBox;
	for (size_t i = 0;i < bboxList.size();i++)
	{
		if ((firstDetectedId == 2 && bboxList[i].getClassId() ==7)||firstDetectedId != bboxList[i].getClassId()) 
		{
			newBox = normalizeCroppedBox(bboxList[i].getRegion(), detectFrame.cols, detectFrame.rows);
			objectFrame = detectFrame(newBox);
			detector->detect(preBBoxFrame, keyPoints1);
			detector->detect(objectFrame, keyPoints2);
			if (keyPoints1.size() == 0 || keyPoints2.size() == 0) {
				continue;
			}
			briefDescriptor->compute(preBBoxFrame, keyPoints1, descriptor1);
			briefDescriptor->compute(objectFrame, keyPoints2, descriptor2);
			if (descriptor1.empty() || descriptor2.empty()) {
				continue;
			}
			// FLANN BASED knn matcher
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
	}
	return -1;
}

ImageMatching::ImageMatching()
{
	detector = GFTTDetector::create(Config::maxFeatureTrack, 0.001, 10);
	briefDescriptor = xfeatures2d::BriefDescriptorExtractor::create();
	matcher = BFMatcher::create();

}
