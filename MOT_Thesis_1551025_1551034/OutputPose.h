#pragma once
#include"Types.h"
#include"OutputFile.h"
class OutputPose:public OutputFile
{

private:
	float time;
	int numPics;
	float distance;
	float velocity;
	Point3D errPose;
	float avgFeatureTime;
	float aed;
	bool isEmpty;
public:
	OutputPose();
	//Set value
	void setTime(float t);
	void setNumPics(int numP);
	void setDistance(float dist);
	void setVelocity(float v);
	void setErrorPose(Point3D errP);
	void setAvgFeatureTime(float avgFeatureTime);
	void setAED(float aed);
	//get Value
	float getTime();
	int getNumPics();
	float getDistance();
	float getVelocity();
	Point3D getErrorPose();
	float getAvgFeatureTime( );

	bool getIsEmpty();
	void setIsEmpty(bool empty);
	// Inherited via OutputFile
	void output(ofstream& fout) override;

};

