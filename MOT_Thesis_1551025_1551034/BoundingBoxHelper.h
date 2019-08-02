#pragma once
#include "Utils.h"
#include "BoundingBox.h"
class BoundingBoxHelper
{
public:
	cv::Rect getNewBoundingBox(cv::Rect bRect, float ratio, int height, int width);
	cv::Rect getOriginalBoundingBox(cv::Rect bRect, int x, int y);
	cv::Rect normalizeCroppedBox(cv::Rect oriBox, float width, float height);

};

