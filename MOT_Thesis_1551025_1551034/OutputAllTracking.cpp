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
		for (size_t i = 0; i < n;i++)
		{
			outputTrackingList[i].output(fout);
			avgIoU50 += outputTrackingList[i].getIoU50();
			avgIoU75 += outputTrackingList[i].getIoU75();
			avgAED += outputTrackingList[i].getAED();
			processTime += outputTrackingList[i].getTime();
		}
		OutputTracking outputTracking(name, avgIoU50 / n, avgIoU75 / n, avgAED / n,processTime/n);
		outputTracking.output(fout);
	}
	fout.close();
}
