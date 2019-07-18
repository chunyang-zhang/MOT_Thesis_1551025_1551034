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
public:
	//Set value
	void setTime(float t);
	void setNumPics(int numP);
	void setDistance(float dist);
	void setVelocity(float v);
	void setErrorPose(Point3D errP);
	void setAvgFeatureTime(float avgFeatureTime);
	//get Value
	float getTime();
	int getNumPics();
	float getDistance();
	float getVelocity();
	Point3D getErrorPose();
	float getAvgFeatureTime( );


	// Inherited via OutputFile
	void output(ofstream& fout) override;

};

