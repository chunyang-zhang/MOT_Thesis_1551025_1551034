#pragma once
#include "OutputFile.h"
#include"Types.h"
class OutputObjectPose :
	public OutputFile
{
private:
	Point3D errPose;
	float aed;
	string name;
	int id;
	bool isEmpty;
public:
	void setName(string name);
	void setId(int id);
	void setAED(float aed);
	void setErrPose(Point3D errP);
	float getAED();
	string getName();
	int getId();
	Point3D getErrPose();
	// Inherited via OutputFile
	void output(ofstream& fout) override;
	OutputObjectPose(int id, string name, Point3D errPose, float aed);
	OutputObjectPose();
	bool getIsEmpty();
	void setIsEmpty(bool empty);
};

