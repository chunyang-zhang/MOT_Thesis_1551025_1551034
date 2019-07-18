#include "OutputPose.h"


void OutputPose::setTime(float t)
{
	time = t;
}

void OutputPose::setNumPics(int numP)
{
	numPics = numP;
}

void OutputPose::setDistance(float dist)
{
	distance = dist;
}

void OutputPose::setVelocity(float v)
{
	velocity = v;
}

void OutputPose::setErrorPose(Point3D errP)
{
	errPose = errP;
}

void OutputPose::setAvgFeatureTime(float avgFeatureTime)
{
	this->avgFeatureTime = avgFeatureTime;
}

float OutputPose::getTime()
{
	return time;
}

int OutputPose::getNumPics()
{
	return numPics;
}

float OutputPose::getDistance()
{
	return distance;
}

float OutputPose::getVelocity()
{
	return velocity;
}

Point3D OutputPose::getErrorPose()
{
	return errPose;
}

float OutputPose::getAvgFeatureTime()
{
	return avgFeatureTime;
}

void OutputPose::output(ofstream& fout)
{
	if (fout.is_open())
	{
		fout << "Time(s), Number Of Pics, Distance (m), Velocity(m/s), MSE x, MSE y, MSE z, FeatureTime (s)" << endl;
		fout << time << ", " << numPics << ", " << distance << ", " << velocity << ", " << errPose[0] << ", " << errPose[1] << ", " << errPose[2]  <<", "<<avgFeatureTime<<endl;
		fout.close();
	}
}
