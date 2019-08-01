#include "OutputObjectPose.h"

void OutputObjectPose::setName(string name)
{
	this->name = name;
}

void OutputObjectPose::setId(int id)
{
	this->id = id;
}

void OutputObjectPose::setAED(float aed)
{
	this->aed = aed;
}

void OutputObjectPose::setErrPose(Point3D errP)
{
	this->errPose = errP;
}

float OutputObjectPose::getAED()
{
	return aed;
}

string OutputObjectPose::getName()
{
	return name;
}

int OutputObjectPose::getId()
{
	return id;
}

Point3D OutputObjectPose::getErrPose()
{
	return errPose;
}

void OutputObjectPose::output(ofstream& fout)
{
	fout <<id<<" "<< name <<" " << errPose.x()<< " " <<errPose.y() <<" " <<errPose.z()<<" "<< aed << endl;
}

OutputObjectPose::OutputObjectPose(int id, string name, Point3D errPose, float aed):
	id(id),name(name),errPose(errPose), aed(aed)
{
}

OutputObjectPose::OutputObjectPose()
{
}

bool OutputObjectPose::getIsEmpty()
{
	return isEmpty;
}

void OutputObjectPose::setIsEmpty(bool empty)
{
	this->isEmpty = empty;
}
