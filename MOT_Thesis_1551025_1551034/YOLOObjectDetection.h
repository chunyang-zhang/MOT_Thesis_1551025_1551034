#pragma once
#include"Utils.h"
#include<opencv2/dnn.hpp>
#include<opencv2/core/types_c.h>
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

	//Draw predicted bounding box
	void drawPrediction(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

	//Get the name of output layers
	vector<cv::String> getOutputNames(const cv::dnn::Net& net);
	vector<int> classIds;
	vector<float> confidences;
	vector<cv::Rect> boxes;
	vector<int> indices;
	void clearResult();

public:
	void drawPrediction(cv::Mat& output);
	YOLOObjectDetection(float confThreshold, float nmsThreshold, float inpWidth, float inpHeight);
	YOLOObjectDetection();
	void objectDetect(cv::Mat& output );
};

