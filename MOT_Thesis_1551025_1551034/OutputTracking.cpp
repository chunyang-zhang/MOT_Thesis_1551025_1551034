#include "OutputTracking.h"

void OutputTracking::setIoU75(float iou75)
{
	this->iou75 = iou75;
}

void OutputTracking::setIoU50(float iou50)
{
	this->iou50 = iou50;
}

void OutputTracking::setAeD(float aed)
{
	this->aed = aed;
}

void OutputTracking::setObjectName(string name)
{
	this->name = name;
}

void OutputTracking::setTime(float time)
{
	this->time = time;
}

float OutputTracking::getIoU75()
{
	return iou75;
}

float OutputTracking::getIoU50()
{
	return iou50;;
}

float OutputTracking::getAED()
{
	return aed;
}

string OutputTracking::getObjectName()
{
	return name;
}

float OutputTracking::getTime()
{
	return time;
}

OutputTracking::OutputTracking(string name, float iou50, float iou75, float aed, float time):
	name(name),iou50(iou50),iou75(iou75),aed(aed),time(time)
{
}

OutputTracking::OutputTracking()
{
}

void OutputTracking::output(ofstream& fout)
{
	fout << name << " " << iou50 << " " << iou75 << " " << aed <<" "<< time << endl;
}

