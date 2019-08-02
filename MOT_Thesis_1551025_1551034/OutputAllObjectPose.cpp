#include "OutputAllObjectPose.h"

void OutputAllObjectPose::setOutputObjectPoseList(vector<OutputObjectPose> outputObjectPoseList)
{
	this->outputObjectPoseList = outputObjectPoseList;
}

void OutputAllObjectPose::output(ofstream& fout)
{
	if (fout.is_open())
	{
		string name = "Avg";
		float avgMSEx = 0;
		float avgMSEy = 0;
		float avgMSEz = 0;
		Point3D ObjPoseError;
		float aed = 0;
		int n = outputObjectPoseList.size();
		float processTime = 0;
		fout << "ID, ObjName, MSE x, MSE y, MSE z, AED" << endl;
		for (size_t i = 0; i < n;i++)
		{
			outputObjectPoseList[i].output(fout);
			aed += outputObjectPoseList[i].getAED();
			ObjPoseError = outputObjectPoseList[i].getErrPose();
			avgMSEx += ObjPoseError.x();
			avgMSEy += ObjPoseError.y();
			avgMSEz += ObjPoseError.z();

		}
		OutputObjectPose outputObjPose(-1, name, Point3D(avgMSEx, avgMSEy, avgMSEz), aed);
		outputObjPose.output(fout);
	}
	fout.close();
}
