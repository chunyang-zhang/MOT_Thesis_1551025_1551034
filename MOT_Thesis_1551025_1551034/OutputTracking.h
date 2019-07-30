#pragma once
#include <string>
#include "OutputFile.h"
using namespace std;
class OutputTracking :
	public OutputFile
{
private:
	string name;
	float iou75;
	float iou50;
	float aed;
	float time;
public:
	void setIoU75(float iou75);
	void setIoU50(float iou50);
	void setAeD(float aed);
	void setObjectName(string name);
	void setTime(float time);
	float getIoU75();
	float getIoU50();
	float getAED();
	string getObjectName();
	float getTime();
	OutputTracking(string name, float iou50, float iou75, float aed,float time);
	OutputTracking();
	// Inherited via OutputFile
	void output(ofstream& fout) override;
};

