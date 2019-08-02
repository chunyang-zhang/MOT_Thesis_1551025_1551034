#include "LoadLabelResult.h"
using namespace cv;
vector<TrackingGroundTruth> LoadLabelResult::getTrackingGroundTruth(string name)
{
	ifstream labelFile(name);
	int frameId;
	int index;
	string labelName;
	float x;
	float y;
	float width;
	float height;
	float posx;
	float posy;
	float posz;
	vector<TrackingGroundTruth> groundTruthList;
	if (labelFile.is_open())
	{
		while (!labelFile.eof())
		{
			labelFile >> frameId >> index >> labelName >> x >> y >> width >> height >> posx >> posy >> posz;
			Rect bbox = Rect(x, y, width, height);
			Point3D pos = Point3D(posx, posy, posz);
			TrackingGroundTruth groundTruth(frameId, index, labelName, bbox, pos);
			groundTruthList.push_back(groundTruth);
			labelFile.ignore(10000, '\n');
		}
	}
	else {
		cout << "Require Label File For Ground Truth" << endl;
		terminate();
	}
	labelFile.close();
	return groundTruthList;
}

vector<vector<TrackingGroundTruth>> LoadLabelResult::getTrackingGroundByOrder(vector<TrackingGroundTruth>groundTruthList)
{
	vector<TrackingGroundTruth> objectGroundTruth;
	vector<vector<TrackingGroundTruth>> objectsGroundTruth;
	for (size_t i = 0; i < groundTruthList.size();i++)
	{
		objectGroundTruth.push_back(groundTruthList[i]);
		if (i + 1 == groundTruthList.size()||groundTruthList[i].getId() != groundTruthList[i + 1].getId())
		{
			objectsGroundTruth.push_back(objectGroundTruth);
			objectGroundTruth.clear();
		}
	}
	return objectsGroundTruth;
}
