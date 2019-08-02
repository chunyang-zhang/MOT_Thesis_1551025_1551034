#pragma once
#include "OutputObjectPose.h"
#include "OutputFile.h"
#include <string>
using namespace std;
class OutputAllObjectPose :
	public OutputFile
{
private:
	vector<OutputObjectPose> outputObjectPoseList;
public:
	void setOutputObjectPoseList(vector<OutputObjectPose> outputObjectPoseList);
	// Inherited via OutputFile
	void output(ofstream& fout) override;
};

