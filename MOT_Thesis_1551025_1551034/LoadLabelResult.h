#pragma once
#include<fstream>
#include"Types.h"
#include"TrackingGroundTruth.h"
using namespace std;
class LoadLabelResult
{
public:
	vector<TrackingGroundTruth> getTrackingGroundTruth(string name);
	vector<vector<TrackingGroundTruth>> getTrackingGroundByOrder(vector<TrackingGroundTruth>groundTruthList);
};

