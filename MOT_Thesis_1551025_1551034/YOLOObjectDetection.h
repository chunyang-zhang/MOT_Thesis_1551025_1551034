#pragma once
#include "Utils.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core/types_c.h>
#include "BoundingBox.h"
#include "BoundingBoxHelper.h"
class YOLOObjectDetection
{
private:
	float confThreshold;//Confidence threshold
	float nmsThreshold;//Non-maximum suppression threshold
	int inpWidth;
	int inpHeight;
	vector<string>classes;
	static string classesFile;
	static cv::String modelConfiguration; 
	static cv::String modelWeights; 
	cv::dnn::Net net;
	
	// Remove bounding box with low confidence using non-maxima suppression
	//output frame and outs bounding boxes
	void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);


	//Get the name of output layers
	vector<cv::String> getOutputNames(const cv::dnn::Net& net);
	vector<int> classIds;
	vector<float> confidences;
	vector<cv::Rect> boxes;
	vector<int> indices;
	BoundingBox bbox;
	void clearResult();
	float iouThreshold;
	bool checkValidId(int firstDetectedId, int Id);
public:

	void setIoUThreshold(float iouRatio);
	//Draw predicted bounding box
	void drawPrediction(int classId, float conf, int left, int top, int right, int bottom,cv::Scalar color, cv::Mat& frame);
	void drawPrediction(cv::Rect bbox, cv::Mat& frame,cv::Scalar scalar);
	BoundingBox getBestBoundingBox();
	void drawPrediction(cv::Mat& output,cv::Scalar color);
	YOLOObjectDetection(float confThreshold, float nmsThreshold, float inpWidth, float inpHeight);
	YOLOObjectDetection();
	//call after object detection
	void getAllBoundingBox(vector<BoundingBox>& bboxList);
	bool objectDetect(cv::Mat& output );
	bool getRelatedBoundingBox(int classId,const cv::Rect& bRect, const cv::Rect& processedBounding, BoundingBox& bb);
	float calculateIoU(const cv::Rect& boxA, const cv::Rect& boxB);
	string getNameOfClass(int classId);
	void getRelatedBoundingBoxes(vector<BoundingBox>& bboxList, const cv::Rect& processedBounding,int classId);
};

