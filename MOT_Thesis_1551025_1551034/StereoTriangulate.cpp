#include "StereoTriangulate.h"
using namespace cv;
StereoTriangulate::StereoTriangulate(CameraParameters camParams):camParams(camParams)
{
	sgbm= cv::StereoSGBM::create();
	sgbm->setMinDisparity(-39);	
	sgbm->setNumDisparities(144);
	sgbm->setBlockSize(3);
	sgbm->setPreFilterCap(63);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setP1(216); //8*channel*sgbmWinSize*sgbmWinsize
	sgbm->setP2(864);//32*channel*sgbmWinSize*sgbmWinsize
	sgbm->setMode(false);
}   

void StereoTriangulate::stereoDepthMapSGBM(const Mat& img1, const Mat& img2, Mat_<Vec3f>& xyz)
{
	//undistortion and rectification for img1, img2 -> imgU1, imgU2
	Mat imgU1, imgU2;
	remap(img1, imgU1, camParams.map1x, camParams.map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(img2, imgU2, camParams.map2x, camParams.map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	//Compute disparity map from rectified img1 img2
	Mat disp, disp8;
	sgbm->compute(imgU1, imgU2, disp);
	disp.convertTo(disp, CV_32F, 1. / 16);
	//projection disparity -> depth map
	xyz = Mat(disp.rows, disp.cols, CV_32F);
	Mat_<float> tmp(4, 1);//changed
	for (int y = 0;y < disp.rows;y++)
	{
		for (int x = 0;x < disp.cols;x++) 
		{
			tmp(0) = x;
			tmp(1) = y;
			tmp(2) = disp.at<float>(y, x);
			tmp(3) = 1;
			tmp = camParams.q *tmp;
			tmp /= tmp(3);
			//changed
			Vec3f& point = xyz.at<Vec3f>(y, x);//get the point of xyz matrix
			point[0] = tmp(0);
			point[1] = tmp(1);
			point[2] = tmp(2);

		}
	}
}

Point3DVector StereoTriangulate::opencvTriangulatePoints(vector<KeyPoint>& keyPoints1, vector<KeyPoint>& keyPoints2, vector<DMatch>& stereoMatches)
{
	//Feature points of img1, img2
	vector<Point2f> fp1;
	vector<Point2f> fp2;
	//Convert Keypoint to Point2f
	int sizeOfP = stereoMatches.size();
	for (int i = 0;i < sizeOfP;i++)
	{
		fp1.push_back(keyPoints1[stereoMatches[i].queryIdx].pt);
		fp2.push_back(keyPoints2[stereoMatches[i].trainIdx].pt);
	}
	//Convert from vector to double 2 channel
	Mat points1(fp1, CV_64FC2);
	Mat points2(fp2, CV_64FC2);
	//Triangulate Point
	Mat points4D; //(x,y,z,w)
	triangulatePoints(camParams.p1, camParams.p2, points1, points2, points4D);
	float x, y, z, w;
	Point3DVector point3DStereo;
	for (int i = 0; i < sizeOfP; i++)
	{
		w = points4D.at<double>(3, i);
		x = points4D.at<double>(0, i) / w;
		y = points4D.at<double>(1, i) / w;
		z = points4D.at<double>(2, i) / w;
		point3DStereo.push_back(Point3D(x, y, z));
	}
	
	return point3DStereo;
}

Point3DVector StereoTriangulate::myTriangulatePoints(vector<KeyPoint>& keyPoints1, vector<KeyPoint>& keyPoints2, vector<DMatch>& stereoMatches, vector<float>& qualityList)
{
	Point3DVector point3DStereo;
	vector<Point2f>fp1, fp2;
	int sizeOfP = stereoMatches.size();
	for (int i = 0;i < sizeOfP;i++)
	{
		fp1.push_back(keyPoints1[stereoMatches[i].queryIdx].pt);
		fp2.push_back(keyPoints2[stereoMatches[i].trainIdx].pt);
	}
	//Convert from vector to double 2 channel
	Mat points1(fp1);
	Mat points2(fp2);
	//Undistort points for observation in normalized image coordinates
	undistortPoints(points1, points1, camParams.cm1, camParams.d1);
	undistortPoints(points2, points2, camParams.cm2, camParams.d2);
	int stereoSize = stereoMatches.size();
	vector<DMatch> matchesStereoPoint;
	for (int i = 0;i < stereoSize;i++) {
		//get key point from man frame
		Point2f tmpFP = points1.at<Point2f>(i, 0);
		//2D -> 3D feature point
		Point3D ob1(tmpFP.x, tmpFP.y, 1);
		//get key point from sub frame
		tmpFP = points2.at<Point2f>(i, 0);
		Point3D ob2(tmpFP.x, tmpFP.y, 1);
		ob1.normalize();
		ob2.normalize();
		float quality;
		//diff p1.x - p2.x > 4
		float diff = abs(keyPoints1[stereoMatches[i].queryIdx].pt.x - keyPoints2[stereoMatches[i].trainIdx].pt.x);
		if (diff > 4) 
		{
			//compute feature pos 3D based on 2 observation vector
			Point3D featurePos = calculateStereoFeaturePos(ob1, ob2, quality);
			//check valid point
			if (checkStereoValidPoint(featurePos))
			{
				//add valid feature pos
				point3DStereo.push_back(featurePos);
				//update point with matched stereo points
				matchesStereoPoint.push_back(stereoMatches[i]);
				qualityList.push_back(quality);

			}
		}
		//updated stereo matches point
	}
	stereoMatches = matchesStereoPoint;
	return point3DStereo;
}
//Calculate Stereo Feature Position by Linear Triangulation Method
Point3D StereoTriangulate::calculateStereoFeaturePos(const Point3D& firstObsv, const Point3D& secondObsv, float& quality)
{
	//Compute unrotated feature observation (vector observation)
	Point3D firstU, secondU;
	firstU = firstObsv;
	secondU = secondObsv;

	//Convert to unit vector
	firstU = firstU / firstU.norm();
	secondU = secondU / secondU.norm();

	//II3 - V*VT;
	Mat3x3 A1 = Mat3x3::Identity() - firstU * firstU.transpose();
	Mat3x3 A2 = Mat3x3::Identity() - secondU * secondU.transpose();
	Mat3x3 A = A1 + A2;

	//Compute eigen values using 
	float c = A(0, 0) * A(2, 2) - A(2, 0) * A(0, 2) - A(1, 0) * A(0, 1) +
		A(1, 1) * A(2, 2) + A(0, 0) * A(1, 1) - A(1, 2) * A(2, 1) - 4;
	float lam1 = 2;
	float lam2 = 1 + sqrt(1 - c);
	float lam3 = 4 - lam1 - lam2;
	float inv_det = 1 / lam1 * lam2 * lam3;
	float ratio = lam3 / lam1;

	Mat3x3 Ainv = A.inverse(); // inverse of A
	//quality of feature pos
	quality = fabs(ratio);
	//calculate position of feature 3D
	Point3D camera2pos(+0.53700, 0, 0);
	Point3D pos = Ainv * (A2 * camera2pos);
	return pos;
}

bool StereoTriangulate::checkStereoValidPoint(const Point3D& point)
{
	if (point(0) == 0 && point(1) == 0 && point(2) == 0)
	{
		return false;
	}
	if (point(2) < 0 || point(2) > 30)
		return false;
	return true;
}
