#include"Utils.h"
#include"StereoCalibration.h"
#include"DroneSlam.h"
#include"OutputPose.h"
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
	//vector<string> detector = { "star","gftt","agast","mser"};
	vector<string> detector = {"gftt" };

	vector<string> descriptor = { "brief"};
	string outputFile; 
	for (size_t i = 0; i < detector.size();i++)
	{
		for (size_t j = 0; j < descriptor.size();j++)
		{
			outputFile = "output_error_";
			outputFile += detector[i] + "_" + descriptor[j] + ".txt";
			cout << "Method: "<< outputFile << endl;
			clock_t start = clock();
			DroneSlam* slam = new DroneSlam();
			OutputPose outputPose;
			slam->setDetectDescriptorMethod(detector[i], descriptor[j]);
			slam->processFrame();

			outputPose = slam->getOutputPose();

			Point3D mse = slam->getMSE();
			outputPose.setErrorPose(mse);
			ofstream fout(outputFile);
			outputPose.output(fout);
			cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
			delete slam;
		}
	}
	//waitKey(0);
}