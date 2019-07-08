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
	clock_t start = clock();
	DroneSlam* slam = new DroneSlam();
	OutputPose outputPose;
	slam->processFrame();

	outputPose = slam->getOutputPose();
	Point3D mse = slam->getMSE();
	outputPose.setErrorPose(mse);
	ofstream fout("output_error_gftt_lucid.txt");
	outputPose.output(fout);
	cout << endl << "printf ALL TOTAL: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
	delete slam;
	waitKey(0);
}