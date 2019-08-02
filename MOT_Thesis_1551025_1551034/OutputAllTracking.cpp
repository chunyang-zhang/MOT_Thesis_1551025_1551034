#include "OutputAllTracking.h"

void OutputAllTracking::setOutputTrackingList(vector<OutputTracking> outputTrackingList)
{
	this->outputTrackingList = outputTrackingList;
}

void OutputAllTracking::output(ofstream& fout)
{
	if (fout.is_open())
	{
		string name = "Avg";
		float avgIoU50 = 0;
		float avgIoU75 = 0;
		float avgAED = 0;
		int n = outputTrackingList.size();
		float processTime = 0;
		fout << "ID, ObjName, IoU50, IoU75, processTime(s)" << endl;

		for (size_t i = 0; i < n;i++)
		{
			outputTrackingList[i].output(fout);
			avgIoU50 += outputTrackingList[i].getIoU50();
			avgIoU75 += outputTrackingList[i].getIoU75();
			processTime += outputTrackingList[i].getTime();
		}
		OutputTracking outputTracking( -1, name, avgIoU50 / n, avgIoU75 / n,processTime/n);
		outputTracking.output(fout);
	}
	fout.close();
}
