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
	float time;
	int id;
	bool isEmpty;
public:
	void setIoU75(float iou75);
	void setIoU50(float iou50);
	void setObjectName(string name);
	void setTime(float time);
	void setId(float Id);
	float getIoU75();
	float getIoU50();
	int getId();
	bool getIsEmpty();
	void setIsEmpty(bool empty);
	string getObjectName();
	float getTime();
	OutputTracking(int id, string name, float iou50, float iou75, float time);
	OutputTracking();
	// Inherited via OutputFile
	void output(ofstream& fout) override;
};

