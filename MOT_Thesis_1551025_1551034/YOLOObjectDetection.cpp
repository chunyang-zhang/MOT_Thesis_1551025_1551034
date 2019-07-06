#include "YOLOObjectDetection.h"
using namespace cv;
using namespace dnn;
string YOLOObjectDetection::classesFile = "coco.names";
String YOLOObjectDetection::modelConfiguration = "yolov3.cfg";
String YOLOObjectDetection::modelWeights = "yolov3.weights";
bool YOLOObjectDetection::objectDetect (Mat& output)
{
	//convert to blob datatype to feed into network
	Mat blob;
	blobFromImage(output,blob,1/255.0, CvSize(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);
	net.setInput(blob);

	// Runs the forward pass to get output of the output layers
	vector<Mat> outs;
	net.forward(outs, getOutputNames(net));
	//imshow("image", output);
	//waitKey(0);
	//Remove bounding boxes with low confidence and overlapping bounding boxes.
	postprocess(output, outs);
	
	//Put efficiency information. The function getPerProfile return the total processed time.
	//vector<double> layersTimes;
	//double freq = getTickFrequency() / 1000;
	//double t = net.getPerfProfile(layersTimes) / freq;
	//string label = format("Inference time for a frame : %.2f ms", t);
	//putText(output, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
	//There is no object available in frame
	drawPrediction(output);
	//imshow("image", output);
	//waitKey(0);
	if (indices.size()==0)
	{
		return false;
	}
	return true;

}
bool YOLOObjectDetection::getRelatedBoundingBox(int classId, const Rect& bRect, const Rect& processedBounding, BoundingBox &bb)
{

	int idx;
	int bestIdx = 0;
	//get the first relevant bounding box
	//store all the relevant bounding box
	vector<int> relevantIdx;
	vector<float>relevantIoUList;
	float bestIoU = 0;
	int bestRelevantIoU = 0;
	float iou;
	BoundingBoxHelper helper;
	Rect tmpBox;

	if (indices.size() != 0)
	{
		//Max(
		for (size_t i = 0; i < indices.size(); ++i)
		{
			idx = indices[i];
			tmpBox = helper.getOriginalBoundingBox(boxes[idx], processedBounding.x, processedBounding.y);
			iou = calculateIoU(tmpBox, bRect);
			//Find box with the best IOU 
			if (iou > bestIoU &&iou>0.7)
			{
				bestIoU = iou;
				bestIdx = idx;
			}
			if (classIds[idx] == classId)
			{
				if (iou > iouThreshold)
				{
					relevantIdx.push_back(i);
					relevantIoUList.push_back(iou);
				}
			}
		}
		//get one with the highest IoU, but with the same class of the original
		for (size_t i = 0;i < relevantIdx.size();i++)
		{
			idx = indices[relevantIdx[i]];
			iou = relevantIoUList[i];
			if (iou > bestRelevantIoU && iou>iouThreshold)
			{
				bestIoU = iou;
				bestIdx = idx;
			}
		}
		//there is not any IoU and no relevant class
		if (bestIoU <= 0 && relevantIdx.size() == 0)
		{
			return false;
		}
		bb.setClassId(classIds[bestIdx]);
		bb.setConfidence(confidences[bestIdx]);
		bb.setRegion(boxes[bestIdx]);
		cout <<"The best IOU:"<< bestIoU << endl;
	}
	return true;
}

float YOLOObjectDetection::calculateIoU(const Rect& boxA, const Rect& boxB)
{
	int x1 = MAX(boxA.x, boxB.x);
	int y1 = MAX(boxA.y, boxB.y);
	int x2 = MIN(boxA.x + boxA.width, boxB.x + boxB.width);
	int y2 = MIN(boxA.y + boxA.height, boxB.y + boxB.height);
	
	int interArea = MAX(0, x2 - x1 + 1) * MAX(0, y2 - y1 + 1);
	int area1 = boxA.width * boxB.height;
	int area2 = boxB.width * boxB.height;
	float iou = 1.0 * interArea / (area1 + area2 - interArea);
	return iou;
}
void YOLOObjectDetection::postprocess(cv::Mat& frame, const vector<cv::Mat>& outs)
{
	clearResult();
	// Scan through all the bounding boxes output from the network 
	for (size_t i = 0; i < outs.size(); ++i)
	{
		//data for each guess
		float* data = (float*)outs[i].data;
		//Only select one with confidence higher than threshold
		//one guess = many bounding boxes
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
			//get scores
			Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			Point classIdPoint;
			double confidence;
			// Get the value and location of the maximum score
			//Only keep class with the highest confidence score
			minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
			if (confidence > confThreshold)
			{
				int centerX = (int)(data[0] * frame.cols);
				int centerY = (int)(data[1] * frame.rows);
				int width = (int)(data[2] * frame.cols);
				int height = (int)(data[3] * frame.rows);
				int left = centerX - width / 2;
				int top = centerY - height / 2;

				classIds.push_back(classIdPoint.x);
				confidences.push_back((float)confidence);
				boxes.push_back(Rect(left, top, width, height));
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

}
void YOLOObjectDetection::clearResult()
{
	classIds.clear();
	confidences.clear();
	boxes.clear();
	indices.clear();
}
BoundingBox YOLOObjectDetection::getBestBoundingBox()
{
	BoundingBox bb;
	int idx;
	float confidence;
	int bestIdx;
	//get index of max confidence
	if (!indices.empty())
	{
		idx = indices[0];
		bestIdx = idx;
		confidence = confidences[idx];
		//Max(
		for (size_t i = 1; i < indices.size(); ++i)
		{
			idx = indices[i];
			if (confidence < confidences[idx])
			{
				confidence = confidences[idx];
				bestIdx = idx;
			}
		}
		bb.setClassId(classIds[bestIdx]);
		bb.setConfidence(confidence);
		bb.setRegion(boxes[bestIdx]);
	}
	return bb;
}
void YOLOObjectDetection::drawPrediction(cv::Mat& output)
{
	if (indices.size() != 0)
	{
		for (size_t i = 0; i < indices.size(); ++i)
		{
			int idx = indices[i];
			Rect box = boxes[idx];
			drawPrediction(classIds[idx], confidences[idx], box.x, box.y,
				box.x + box.width, box.y + box.height, output);
		}

	}
}

void YOLOObjectDetection::setIoUThreshold(float iouRatio)
{
	iouThreshold *= iouRatio;
}

void YOLOObjectDetection::drawPrediction(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
	//Draw rectangle displaying the bounding box
	rectangle(frame, Point(left, top), Point(right, bottom),Scalar(255,178,50),3);
	//Get the label for the class name and its confidence
	string label = format("%.2f", conf);
	if (!classes.empty())
	{
		//class Id smaller than the total size
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":"+ label;
	}
	//Display the label at the top of the bounding box
	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - round(1.5 * labelSize.height)), Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0));
}
//get the last layers name for forwarding into network
vector<String> YOLOObjectDetection::getOutputNames(const Net& net)
{
	static vector<String> names;
	//Get the indices of the output layers, i.e. the layers with unconnected outputs
	vector<int> outLayers = net.getUnconnectedOutLayers();

	//get the names of all the layers in the network
	vector<String> layersNames = net.getLayerNames();

	// Get the names of the output layers in names
	names.resize(outLayers.size());
	for (size_t i = 0; i < outLayers.size(); ++i)
		names[i] = layersNames[outLayers[i] - 1];

	return names;
}
YOLOObjectDetection::YOLOObjectDetection(float confThreshold, float nmsThreshold, float inpWidth, float inpHeight):
confThreshold(confThreshold), nmsThreshold(nmsThreshold), inpWidth(inpWidth), inpHeight(inpHeight)
{
	iouThreshold = 0.65f;
	//Get class names
	ifstream ifs(classesFile.c_str());
	string line;
	while (getline(ifs, line)) classes.push_back(line);
	//load the net work
	net = readNetFromDarknet(modelConfiguration, modelWeights);
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	//to use for cpu, GPU: DNN_TARGET_OPENCL
	net.setPreferableTarget(DNN_TARGET_CPU);

}
YOLOObjectDetection::YOLOObjectDetection():
confThreshold(0.5),nmsThreshold(0.5),inpWidth(416),inpHeight(416)
{
	//Get class names
	ifstream ifs(classesFile.c_str());
	string line;
	while (getline(ifs, line)) classes.push_back(line);
	//load the net work
	net = readNetFromDarknet(modelConfiguration, modelWeights);
	net.setPreferableBackend(DNN_BACKEND_OPENCV);
	//to use for cpu, GPU: DNN_TARGET_OPENCL
	net.setPreferableTarget(DNN_TARGET_CPU);
	iouThreshold = 0.65f;

}