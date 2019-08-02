#include "OutputTracking.h"

void OutputTracking::setIoU75(float iou75)
{
	this->iou75 = iou75;
}

void OutputTracking::setIoU50(float iou50)
{
	this->iou50 = iou50;
}

void OutputTracking::setObjectName(string name)
{
	this->name = name;
}

void OutputTracking::setTime(float time)
{
	this->time = time;
}

void OutputTracking::setId(float Id)
{
	this->id = Id;
}

float OutputTracking::getIoU75()
{
	return iou75;
}

float OutputTracking::getIoU50()
{
	return iou50;;
}

int OutputTracking::getId()
{
	return id;
}

bool OutputTracking::getIsEmpty()
{
	return isEmpty;
}

void OutputTracking::setIsEmpty(bool empty)
{
	this->isEmpty = empty;
}



string OutputTracking::getObjectName()
{
	return name;
}

float OutputTracking::getTime()
{
	return time;
}

OutputTracking::OutputTracking(int id, string name, float iou50, float iou75, float time):
	id(id), name(name),iou50(iou50),iou75(iou75),time(time)
{
}

OutputTracking::OutputTracking()
{
}                                                                        

void OutputTracking::output(ofstream& fout)
{
	fout << id<<" "<<name << " " << iou50 << " " << iou75 << " "<< time << endl;
}

