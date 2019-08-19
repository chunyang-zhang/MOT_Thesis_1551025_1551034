#include"Utils.h"
#include"StereoCalibration.h"
#include"DroneSlam.h"
#include"OutputPose.h"
#include"LoadLabelResult.h"
#include"TrackingGroundTruth.h"
#include"OutputTracking.h"
#include"OutputAllTracking.h""
#include"OutputObjectPose.h"
#include"OutputAllObjectPose.h"
using namespace cv;
int main(int argc, char** argv) {
	//Argument
	int counter;
	cout<<"Program Name Is: %s"<< argv[0]<<endl;
	if (argc == 1)
		cout << "No Extra Command Line Argument Passed. Used Default Set Up" << endl;
	if (argc >= 2 && argc <6)
	{
		if (strcmp(argv[1], "demo")!=0 && strcmp(argv[1], "test")!=0)
		{
			cout << "There are 2 options: demo | test." << endl;
			return 0;
		}
		if (strcmp(argv[2], "IoU")!=0 && strcmp(argv[2], "ImageMatching")!=0 && strcmp(argv[2], "IoUMatching")!=0 && strcmp(argv[2], "all")!=0)
		{
			cout << "There are 4 options for tracking: IoU | IoUMatching | ImageMatching | all." << endl;
			return 0;
		}		
		if (strcmp(argv[3],"gftt")!=0 && strcmp(argv[3],"agast")!=0 && strcmp(argv[3], "star")!=0 && strcmp(argv[3], "brisk")!=0 && strcmp(argv[3], "orb")!=0 && !strcmp(argv[3],"fast")!=0)
		{
			cout << "There are 6 options for detector: gftt | agast | star | brisk | orb | fast." << endl;
			return 0;
		}
		if (strcmp(argv[4], "brief")!=0 && strcmp(argv[4], "daisy")!=0 && strcmp(argv[4], "freak")!=0 && strcmp(argv[4],"latch")!=0 && strcmp(argv[4], "brisk")!=0 && strcmp(argv[4],"orb")!=0)
		{
			cout << "There are 6 options: brief | daisy | freak | latch | brisk | orb." << endl;
			return 0;
		}
	}
	else if(argc>=6)
	{
		cout << "Your argument list is out of range!" << endl;
		return 0;
	}
	vector<TrackingGroundTruth> groundTruthList;
	vector<vector<TrackingGroundTruth>>groundTruthOrderList;
	LoadLabelResult labelResult;
	string det;
	string des;
	string matchingMethod;
	vector<string> trackingMethodVector;
	string outFolder = "Output/";
	string outCamPose = "cam_pose.txt";
	string outObjectPose = outFolder+ "object_pose";
	string outErrorObjectPose = outFolder + "error_object_pose";
	string outTrackingResult = outFolder + "tracking_result_";
	string runMethod = "test";

	//OutputTracking Info
	vector<OutputTracking> outputTrackingList;
	OutputAllTracking outputAllTracking;
	//Output Pose Info
	vector<OutputObjectPose> outputObjectPoseList;
	OutputAllObjectPose outputAllPose;
	bool isTracked = false;
	string chosenDetector = "gftt";
	string chosenDescriptor = "brief";
	string trackingMethod = "all";//"IoUMatching";
	//vector<string> detector = {  "star", "brisk" ,"orb", "fast" };
	//vector<string> detector = { "gftt","brisk","orb","fast","star","agast"};
	//vector<string> descriptor = {"brief", "daisy", "freak","latch" };
	vector<string> detector = { "gftt" };
	vector<string> descriptor = { "brief" };
	if (argc == 2)
	{
		runMethod = argv[1];
	}
	else if (argc == 3)
	{
		runMethod = argv[1];
		trackingMethod = argv[2];
		cout << "TrackingMethod" << trackingMethod << endl;;
	}
	else if (argc == 4)
	{
		cout << "Please input a descriptor for the detector" << endl;
		return 0;
	}
	else if (argc == 5)
	{
		if (argv[3] == "fast" &&  argv[4]!= "brief")
		{
			cout << "This option is not available!" << endl;
			cout << "Please try with brief descriptor." << endl;
			return 0;
		}
		if ((argv[3] == "brisk" && argv[4] != "brisk") ||( argv[3] != "brisk" && argv[4] == "brisk"))
		{
			cout << "This option is not available!" << endl;
			cout << "Please try with brisk." << endl;
			return 0;
		}
		if ((argv[3] == "orb" && argv[4] != "orb") || (argv[3] != "orb" && argv[4] == "orb"))
		{
			cout << "This option is not available!" << endl;
			cout << "Please try with orb." << endl;
			return 0;
		}
		runMethod = argv[1];
		trackingMethod = argv[2];
		chosenDetector = argv[3];
		chosenDescriptor = argv[4];

	}
	if (trackingMethod.compare("all")==0)
	{
		trackingMethodVector =  { "IoUMatching", "IoU","ImageMatching" };
	}
	else {
		trackingMethodVector.push_back(trackingMethod);
	}
	//Load ground truth file
	//groundTruthList = labelResult.getTrackingGroundTruth("./DATA/label.txt");
	//groundTruthOrderList = labelResult.getTrackingGroundByOrder(groundTruthList);
	//for (size_t j = 0;j < trackingMethodVector.size();j++)
	//{
	//	//object Pose
	//	string outErrorObjectPoseTmp = outErrorObjectPose + "_" + trackingMethodVector[j] + ".txt";
	//	string outTrackingResultTmp =  outTrackingResult + trackingMethodVector[j] + ".txt";

	//	for (size_t i = 0; i < groundTruthOrderList.size();i++)
	//	{
	//		string outputFileTmp = outFolder + trackingMethodVector[j] + "_" + to_string(i) + "_"+"output_error_" + chosenDetector + "_" + chosenDescriptor + ".txt";
	//		string outObjectPoseTmp = outObjectPose + "_" + groundTruthOrderList[i][0].getName()+ to_string(i)+"_" +trackingMethodVector[j] + ".txt";

	//		cout << "Method: " << outputFileTmp << endl;
	//		cout << "File Object Pose at each Frame: " << outObjectPoseTmp << endl;
	//		clock_t start = clock();
	//		DroneSlam* slam = new DroneSlam(outCamPose, outObjectPoseTmp);
	//		OutputPose outputPose;
	//		slam->setDetectDescriptorMethod(chosenDetector, chosenDescriptor);
	//		slam->setGroundTruthValue(groundTruthOrderList[i]);
	//		slam->setTrackingMethod(trackingMethodVector[j]);
	//		slam->setRunningMethod(runMethod);
	//		slam->processFrame();
	//		isTracked = slam->getIsTracked();
	//		if (!isTracked)
	//		{
	//			continue;
	//		}
	//		outputPose = slam->getOutputPose();

	//		
	//		ofstream fout(outputFileTmp);
	//		outputPose.output(fout);
	//		//tracking value result

	//		OutputTracking outputTracking = slam->getTrackingResult();
	//		outputTrackingList.push_back(outputTracking);
	//		OutputObjectPose outputObjectPose = slam->getOutputObjectPose();
	//		if (!outputObjectPose.getIsEmpty())
	//		{
	//			outputObjectPoseList.push_back(outputObjectPose);
	//		}
	//		cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
	//		delete slam;

	//	}
	//	cout << "Finish Processing - Write to file!" << endl;
	//	//out tracking result of 1 method
	//	outputAllTracking.setOutputTrackingList(outputTrackingList);
	//	ofstream fout(outTrackingResultTmp);
	//	outputAllTracking.output(fout);
	//	outputTrackingList.clear();
	//	//out object pose result of 1 method
	//	outputAllPose.setOutputObjectPoseList(outputObjectPoseList);
	//	ofstream foutError(outErrorObjectPoseTmp);
	//	outputAllPose.output(foutError);
	//	outputObjectPoseList.clear();
	//}
	//Test Detector Descriptor
	string outputFile;
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
			
			outputFile = "output_error_" + det + "_" +des + ".txt";
			cout << "Method: " << outputFile << endl;
			clock_t start = clock();
			DroneSlam* slam = new DroneSlam(outCamPose);
			slam->setDetectDescriptorMethod(det, des);
			//slam->setMotionCompensation(true);
			slam->processFrame();

			ofstream fout(outputFile);
			OutputPose outputPose = slam->getOutputPose();

			outputPose.output(fout);
			if (detector[i].compare("orb") == 0 || detector[i].compare("brisk") == 0)
			{
				break;
			}
			cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
			delete slam;

		}
	}
	//waitKey(0);
}