#include "BoundingBoxHelper.h"
using namespace cv;
Rect BoundingBoxHelper::getNewBoundingBox(Rect bRect, float ratio, int height, int width)
{
	int centerX = bRect.x + bRect.width / 2;
	int centerY = bRect.y + bRect.height / 2;
	int newWidth = ratio * bRect.width;
	int newHeight = ratio * bRect.height;
	int x = centerX - newWidth / 2;
	int y = centerY - newHeight / 2;
	if (x < 0)
	{
		newWidth -= abs(x);
		x = 0;
	}
	if (y < 0)
	{
		newHeight -= abs(y);
		y = 0;
	}
	if (x + newWidth > width)
	{
		newWidth = width - x;
	}
	if (y + newHeight > height)
	{
		newHeight = height - y;
	}
	return Rect(x, y, newWidth, newHeight);

}

Rect BoundingBoxHelper::getOriginalBoundingBox(Rect bRect, int x, int y)
{
	int newX = bRect.x + x;
	int newY = bRect.y + y;
	return Rect(newX,newY,bRect.width,bRect.height);
}

cv::Rect BoundingBoxHelper::normalizeCroppedBox(cv::Rect oriBox, float width, float height)
{

	int x = oriBox.x;
	int y = oriBox.y;
	int bWidth = oriBox.width;
	int bHeight = oriBox.height;
	//Normalize x,y first
	if (x < 0)
	{
		bWidth -= abs(x);
		x = 0;
	}
	if (y < 0)
	{
		bHeight -= abs(y);
		y = 0;
	}

	int right = x + bWidth;
	int bottom = y + bHeight;
	
	if (right >= width)
	{
		bWidth = width - x - 1;
	}
	if (bottom >= height)
	{
		bHeight = height - y - 1;
	}
	Rect newBox(x, y, bWidth, bHeight);
	return newBox;
}


