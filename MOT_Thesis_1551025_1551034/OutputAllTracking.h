#pragma once
#include "OutputTracking.h"
#include "OutputFile.h"
#include <string>
using namespace std;
class OutputAllTracking:public OutputFile 
{
private:
	vector<OutputTracking> outputTrackingList;
public:
	void setOutputTrackingList(vector<OutputTracking> outputTrackingList);
	// Inherited via OutputFile
	 void output(ofstream& fout) override;
};

