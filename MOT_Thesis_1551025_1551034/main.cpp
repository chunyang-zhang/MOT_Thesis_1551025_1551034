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
	//vector<string> detector = { "brisk","orb","fast","star","gftt","agast"};

	//vector<string> descriptor = { "brief","daisy","latch","freak"};
	vector<string> detector = { "agast"};
	vector<string> descriptor = { "latch"};
	//vector<string> detector = { "fast","gftt" };
	//vector<string> descriptor = { "brief" };
	string outputFile;
	string det;
	string des;
	for (size_t i = 0; i < detector.size();i++)
	{
		for (size_t j = 0; j < descriptor.size();j++)
		{
			if(detector[i].compare("fast") == 0 && descriptor[j].compare("brief") != 0)
			{
				continue;
			}
			if (detector[i].compare("orb") == 0|| detector[i].compare("brisk") == 0)
			{
				det = des = detector[i];
			}
			else 
			{ 
				det = detector[i];
				des = descriptor[j];
			}

			outputFile = "output_error_";
			outputFile += det+ "_" + des + ".txt";
			cout << "Method: "<< outputFile << endl;
			clock_t start = clock();
			DroneSlam* slam = new DroneSlam();
			OutputPose outputPose;
			slam->setDetectDescriptorMethod(det, des);
			slam->processFrame();

			outputPose = slam->getOutputPose();

			Point3D mse = slam->getMSE();
			outputPose.setErrorPose(mse);
			ofstream fout(outputFile);
			outputPose.output(fout);
			cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
			delete slam;
			if (detector[i].compare("orb") == 0 || detector[i].compare("brisk") == 0)
			{
				break;
			}
		}
	}
	//waitKey(0);
}