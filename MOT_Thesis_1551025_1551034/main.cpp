#include"Utils.h"
#include"StereoCalibration.h"
#include"DroneSlam.h"
#include"OutputPose.h"
#include"LoadLabelResult.h"
#include"TrackingGroundTruth.h"
#include"OutputTracking.h"
#include"OutputAllTracking.h""
using namespace cv;
int main() {
	//Mat img;
	//VideoCapture vc;
	//vc.open(0);
	//while (vc.read(img))
	//{
	//	cvtColor(img, img, COLOR_RGB2GRAY);
	//	imshow("frame",img);
	//	waitKey(1);
	//}
	//return 0;
	//vector<string> detector = {"brisk","orb", "star","agast"};

	//vector<string> descriptor = { "brief","freak"};
	//vector<string> detector = { "brisk","orb"};
	//vector<string> descriptor = { "agast"};
	vector<TrackingGroundTruth> groundTruthList;
	LoadLabelResult labelResult;
	vector<string> detector = {"gftt" };
	vector<vector<TrackingGroundTruth>> groundTruthOrderList;
	vector<string> descriptor = { "brief" };
	string outputFile;
	string det;
	string des;
	string matchingMethod;
	vector<string> trackingMethodVector;
	string outFolder = "Output/";
	string outCamPose = outFolder+ "cam_pose.txt";
	string outObjectPose = outFolder+ "object_pose";
	string runMethod = "test";
	string outTrackingResult=outFolder;
	vector<OutputTracking> outputTrackingList;
	OutputAllTracking outputAllTracking;
	if (matchingMethod.empty())
	{
		trackingMethodVector =  { "ImageMatching", "IoUMatching", "IoU" };
	}
	else {
		trackingMethodVector.push_back(matchingMethod);
	}
	for (size_t i = 0; i < detector.size();i++)
	{
		for (size_t j = 0; j < descriptor.size();j++)
		{
			if (detector[i].compare("fast") == 0 && descriptor[j].compare("brief") != 0)
			{
				continue;
			}
			if (detector[i].compare("orb") == 0 || detector[i].compare("brisk") == 0)
			{
				det = des = detector[i];
			}
			else
			{
				det = detector[i];
				des = descriptor[j];
			}

			outputFile = outFolder+"output_error_";
			outputFile += det + "_" + des + ".txt";
			cout << "Method: " << outputFile << endl;
			groundTruthList = labelResult.getTrackingGroundTruth("./DATA/label.txt");
			groundTruthOrderList = labelResult.getTrackingGroundByOrder(groundTruthList);
			for (size_t j = 0;j < trackingMethodVector.size();j++)
			{
				outObjectPose.append("_" +trackingMethodVector[i]+ ".txt");
				outTrackingResult.append(trackingMethodVector[i] + "_result.txt");
				for (size_t i = 0; groundTruthOrderList.size();i++)
				{
					clock_t start = clock();
					DroneSlam* slam = new DroneSlam(outCamPose, outObjectPose);
					OutputPose outputPose;
					slam->setDetectDescriptorMethod(det, des);
					slam->setGroundTruthValue(groundTruthOrderList[i]);
					slam->setTrackingMethod(trackingMethodVector[j]);
					slam->setRunningMethod(runMethod);
					slam->processFrame();
					
					outputPose = slam->getOutputPose();
					Point3D mse = slam->getMSE();
					outputPose.setErrorPose(mse);
					float aed = slam->getAED();

					outputPose.setAED(aed);
					ofstream fout(outputFile);
					outputPose.output(fout);
					//tracking value result

					OutputTracking outputTracking = slam->getTrackingResult();
					outputTrackingList.push_back(outputTracking);
					cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
					delete slam;
					if (detector[i].compare("orb") == 0 || detector[i].compare("brisk") == 0)
					{
						break;
					}
				}
				//outtracking result of 1 method
				outputAllTracking.setOutputTrackingList(outputTrackingList);
				ofstream fout(outTrackingResult);
				outputAllTracking.output(fout);
				outputTrackingList.clear();
			}
		}
	}
	//waitKey(0);
}