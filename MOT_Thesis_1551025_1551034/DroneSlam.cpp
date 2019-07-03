#include "DroneSlam.h"
using namespace cv;
Point seed;
int seedId = -1;
void onMouse(int event, int x, int y, int, void*);
void onMouse(int event, int x, int y, int, void*)
{
	if (event != EVENT_LBUTTONDOWN)
		return;

	seed = Point(x, y);
}
/*
Write: 
Feature Observation 

*/

void DroneSlam::addFeaturesToLocalMap(vector<KeyPoint>& keyPoints1, vector<DMatch>& stereoMatches, Point3DVector& point3DStereo, vector<float>& allQuality, int frameId, const CameraParameters& camParams)
{
	// update to World coordinate
	//new 3D = Rotation Matrix * cur3D  + cameraPos
	for (int i = 0; i < stereoMatches.size(); i++)
	{
		point3DStereo[i] = currR * point3DStereo[i] + cameraPos.back();
	}

	// Features Observation
	// KeyPoint -> Point2f
	vector <Point2f> p2f = keyPointConversion.KeyPoint2Point2f(keyPoints1);


	// Point2f -> Mat
	Mat pMat(p2f);

	// Undistort Points for observation in normalized image coordinates
	undistortPoints(pMat, pMat, camParams.cm1, camParams.d1);

	// Update feature observation into local map
	Point2f tmpP2f;
	FeatureStorage tmpFStorage;

	tmpFStorage.firstFrameID = frameId;
	tmpFStorage.isStereoPose = false;
	tmpFStorage.isMonoPose = false;
	tmpFStorage.isCull = false;
	tmpFStorage.nOutliers = 0;
	for (int i = 0; i < p2f.size(); i++)
	{
		//Access point at point Mat
		tmpP2f = pMat.at<Point2f>(i, 0);
		//save the feature point in to feature storage
		tmpFStorage.featureObservation.push_back((Point3D(tmpP2f.x, tmpP2f.y, 1)).normalized());
		localMap.push_back(tmpFStorage);
		tmpFStorage.featureObservation.pop_back();
	}

	// Store 3D position into localmap using stereoMatches to query index
	size_t sizeOfStereoMatches = stereoMatches.size();
	for (size_t i = 0; i < sizeOfStereoMatches; i++)
	{
		localMap[stereoMatches[i].queryIdx].isStereoPose = true;
		localMap[stereoMatches[i].queryIdx].featurePoseStereo = point3DStereo[i];
		localMap[stereoMatches[i].queryIdx].poseStereoQuality = allQuality[i];
	}

}
/*
Write:
Feature Observation

*/
void DroneSlam::addMoreFeaturesToLocalMap(const vector<cv::Point2f>& keyPoints1, int FrameId, const CameraParameters& camParams)
{
	//Add more features to the current local map
	vector <Point2f> keyP = keyPoints1;
	int sizeOfKeyP1 = keyPoints1.size();

	// Point2f -> Mat for undistorting point
	Mat matPoints(keyP);

	// Undistort Points for observation in normalized image coordinates
	undistortPoints(matPoints, matPoints, camParams.cm1, camParams.d1);

	// Add feature observation into local map
	Point2f tmpP2f;
	FeatureStorage tmpFStorage;
	tmpFStorage.firstFrameID = FrameId;
	tmpFStorage.isStereoPose = false;
	tmpFStorage.isMonoPose = false;
	tmpFStorage.isCull = false;
	tmpFStorage.nOutliers = 0;

	for (int i = 0; i < sizeOfKeyP1; i++)
	{
		tmpP2f = matPoints.at<Point2f>(i, 0);
		tmpFStorage.featureObservation.push_back((Point3D(tmpP2f.x, tmpP2f.y, 1)).normalized());
		localMap.push_back(tmpFStorage);
		tmpFStorage.featureObservation.pop_back();
	}

}
//this one neccesarry?
void DroneSlam::updateLocalMapStereo(Point3DVector& point3D, vector<float>& allQuality, vector<DMatch>& map3Dto2D, Mat3x3& currR, Point3D& currCameraPos, vector<int>& mapKP2Local)
{
	// TO WORLD coordinate
	//new 3D = Rotation Matrix * cur3D  + cameraPos
	int sizeStereo3DP = map3Dto2D.size();
	for (int i = 0; i < sizeStereo3DP; i++)
	{
		point3D[i] = currR * point3D[i] + currCameraPos;
	}

	// ADD to local map
	for (int i = 0; i < sizeStereo3DP; i++)
	{
		//if the current feature is stereo Pose
		if (localMap[mapKP2Local[map3Dto2D[i].queryIdx]].isStereoPose)
		{
			// check quality of two
			if (allQuality[i] < 0)
			{
				localMap[mapKP2Local[map3Dto2D[i].queryIdx]].featurePoseStereo = point3D[i];

				//DEBUG
				cout << mapKP2Local[map3Dto2D[i].queryIdx] << "\t ||";
			}
		}
		//if the current pose havent checked stereo pose
		else
		{
			localMap[mapKP2Local[map3Dto2D[i].queryIdx]].isStereoPose = true;
			localMap[mapKP2Local[map3Dto2D[i].queryIdx]].poseStereoQuality = allQuality[i];
			localMap[mapKP2Local[map3Dto2D[i].queryIdx]].featurePoseStereo = point3D[i];

			//DEBUG
			cout << mapKP2Local[map3Dto2D[i].queryIdx] << "\t ---";
		}
	}
}

Point3D DroneSlam::calculateFeaturePos(const Point3D& firstCameraPos, const Point3D& firstObsv, const Point3D& secondCameraPos, const Point3D& secondObsv, float& quality, Mat3x3& R)
{
	//Havent understand yet
	// calculate unrotated feature observation
	Point3D firstU, secondU;
	firstU = R * firstObsv;
	secondU = currR * secondObsv;


	firstU = firstU.normalized();
	secondU = secondU.normalized();

	//Find the quality of feature pose
	Mat3x3 A1 = Mat3x3::Identity() - firstU * firstU.transpose();
	Mat3x3 A2 = Mat3x3::Identity() - secondU * secondU.transpose();
	Mat3x3 A = A1 + A2;

	// eigenvalues
	float c = A(0, 0) * A(2, 2) - A(2, 0) * A(0, 2) - A(1, 0) * A(0, 1) +
		A(1, 1) * A(2, 2) + A(0, 0) * A(1, 1) - A(1, 2) * A(2, 1) - 4;

	float lam1 = 2;
	float lam2 = 1 + sqrt(1 - c);
	float lam3 = 4 - lam1 - lam2;
	float inv_det = 1 / (lam1 * lam2 * lam3);
	float ratio = lam3 / lam1;

	Mat3x3 Ainv = A.inverse();

	// update quality of feature pos
	quality = fabs(ratio);
	//  calculate position
	Point3D pos = Ainv * (A1 * firstCameraPos + A2 * secondCameraPos);
	return pos;
}
//Find camera Position
Point3D DroneSlam::calculateCameraPose(FeatureStorageVector& localMap, Point3D& preCameraPos, int& numOfInliers)
{
	// Rotation from IMU curr time

	// camera pos
	Point3D currCameraPos = preCameraPos;

	// check if have localMap, only produce new pose if got local map

	int size = localMap.size();
	if (size == 0)
	{
		return currCameraPos;
	}
	// check if feature have 3D pos

	//Store ID of feature have 3D pos
	vector <int> featuresHave3D;
	//Vector Store the distance from feature pose - preCamera pose
	vector <float> distanceOfFeature;

	Point3DVector p3D_OfFeature;
	for (int i = 0; i < size; i++)
	{
		//if current  local Map is stereo
		if (localMap[i].isStereoPose)
		{

			// calculate d = ||R(t-1) - Wpi ||
			featuresHave3D.push_back(i);
			float tmp = (localMap[i].featurePoseStereo - preCameraPos).norm();
			distanceOfFeature.push_back(tmp);

			// 3D point of feature
			p3D_OfFeature.push_back(localMap[i].featurePoseStereo);
		}
		else//Mono Pose
		{
			if (localMap[i].isMonoPose)
			{
				// calculate d = || p - (r)t-1 ||
				featuresHave3D.push_back(i);
				float tmp = (localMap[i].featurePoseMono - preCameraPos).norm();
				distanceOfFeature.push_back(tmp);

				//! 3D point of feature
				p3D_OfFeature.push_back(localMap[i].featurePoseMono);
			}
		}
	}
	//cout << "Number of stereo: " << nStereo << endl;
	//cout << "Number of mono: " << nMono << endl;
	// calculate A = II3 - uij*uij^T / d of all features have 3D pos
	// all A for RANSAC to Calculate sum Of A
	Mat3x3Vector allA;

	// all A*p for RANSAC
	Point3DVector allAxp;
	Point3DVector allObsVector;
	//contain list of index of local Map that feature have 3D 
	size = featuresHave3D.size();
	for (int i = 0; i < size; i++)
	{
		// calculate feature observation unrotated frame
		Point3D u_ij;
		//Kit = R*k(i->c)
		//Current Rotation Matrix * FeatureObservation from Image to Camera
		u_ij = currR * localMap[featuresHave3D[i]].featureObservation.back();

		// to unit vector
		u_ij = u_ij / u_ij.norm();

		// Compute A
		Mat3x3 A = Mat3x3::Identity() - u_ij * u_ij.transpose();
		A /= distanceOfFeature[i];

		// all_A
		allA.push_back(A);

		// A*Wp (World Point)
		allAxp.push_back(A * p3D_OfFeature[i]);

		// all_uij
		allObsVector.push_back(u_ij);
	}
	               
	vector <int> resultInliers;

	// 2-points RANSAC to get camera 3D pose
	currCameraPos = ransac2PCameraPose(allA, allAxp, allObsVector, p3D_OfFeature, resultInliers);

	//For Each feature local map, init Inliers = false, 
	for (size_t i = 0; i < localMap.size(); i++)
	{
		localMap[i].isInliers = false;
		localMap[i].nOutliers++;
	}
	//check the valid inliers and update it. in local map (feature storage)
	for (size_t i = 0; i < resultInliers.size(); i++)
	{

		localMap[featuresHave3D[resultInliers[i]]].isInliers = true;
		localMap[featuresHave3D[resultInliers[i]]].nOutliers = 0;
		//resultInliers[i] = featuresHave3D[resultInliers[i]];
		//cout<< featuresHave3D[resultInliers[i]]<< "=\t";
	}
	numOfInliers = resultInliers.size();

	return currCameraPos;
}

Point3D DroneSlam::ransac2PCameraPose(const Mat3x3Vector& allA, const Point3DVector& allAxp, const Point3DVector& ObsVectors, const Point3DVector& p3DFeature, vector<int>& resultInliers)
{
	srand(time(NULL));
	//get the latest camera pos
	Point3D cameraPosRes = cameraPos.back();

	// best_err and inliers
	double error = 1E10;
	vector <int> inliers = vector<int>();

	// points for RANSAC sample
	vector <int> curPoints;

	// number of sample points
	int nSamples = allA.size();
	//The number of samples > 2
	if (nSamples < Config::numPointsRANSAC)
	{
		cout << endl << "DEBUG-- localMap.size()<2";
		return cameraPosRes;
	}

	//! for all RANSAC iterations, do:
	//loop = 50
	for (int i = 0; i < Config::loopRANSAC; i++)
	{
		double curErr = 0;
		vector <int> curInliers;
		Point3D curPos;

		curPoints.clear();
		// get random sample points
		for (int j = 0; j < Config::numPointsRANSAC; j++)
		{
			curPoints.push_back(rand() % nSamples);
		}

		// calculate camera pose from random sample points
		curPos = computePose(allA, allAxp, curPoints);

		// from camera pose compute err and inliers
		computeErrAndInliers2(curPos, ObsVectors, p3DFeature, curInliers, curErr);

		// Do not need to continue the current loop, create next samples
		//If the error is larger, or number of inliers smaller than threshold, continue to next loop
		if (curErr > Config::threshSumErrRANSAC || curInliers.size() < Config::threshNumOfInliersRANSAC)
		{
			continue;
		}

		// use all inliers to compute camera pose
		//update pos with new inliers
		curPos = computePose(allA, allAxp, curInliers);

		// compute err and inliers
		computeErrAndInliers2(curPos, ObsVectors, p3DFeature, curInliers, curErr);

		// check again, if can not reject outlier -> continue

		if (curErr > Config::threshSumErrRANSAC || curInliers.size() < Config::threshNumOfInliersRANSAC)
		{
			continue;
		}

		// compare with best result
		if (curErr < error && curInliers.size() > inliers.size())
		{
			error = curErr;
			inliers = curInliers;
			cameraPosRes = curPos;
		}
	}
	cout << endl << "RANSAC error = " << error;
	cout << endl << "Number of RANSAC points = " << inliers.size();
	//result list index of inliers
	resultInliers = inliers;

	return cameraPosRes;
}

void DroneSlam::computeErrAndInliers(const Point3D& cameraPose, const Mat3x3Vector& allA, const Point3DVector& allAxp, vector<int>& inliers, double& error)
{
	//! detect inliers and compute errors
	error = 0;
	inliers.clear();

	int size = allA.size();
	for (int i = 0; i < size; i++)
	{
		//Error value = Mat3x3 * 3x1 - 3x1
		double tmpErr = (allA[i] * cameraPose - allAxp[i]).norm();	
		if (tmpErr < Config::threshErrRANSAC)//0.0001
		{
			inliers.push_back(i);
			error += tmpErr; //sum error
		}
	}
	//if there are inliers
	if (inliers.size() > 0)
	{
		//RMS Error
		error /= inliers.size();
		error = sqrt(error);
	}
	else
	{
		error = 1E11;
	}
}

Point3D DroneSlam::computePose(const Mat3x3Vector& allA, const Point3DVector& allAxp, const vector<int>& points)
{
	size_t sizeP = points.size();

	// sum of A for all points
	Mat3x3 A = Mat3x3::Zero();
	Point3D Axp = Point3D(0, 0, 0);
	//Sum of all A
	//Sum of ALL A x world point
	for (size_t i = 0; i < sizeP; i++)
	{
		A += allA[points[i]];
		Axp += allAxp[points[i]];
	}
	// inverse of A
	Mat3x3 Ainv;
	Ainv = A.inverse();

	// camera pose return
	//3x3 * 3x1
	//Axp *Ainv
	Point3D cameraPose = Ainv * Axp;
	return cameraPose;

}

void DroneSlam::computeErrAndInliers2(const Point3D& cameraPose, const Point3DVector& ObsVectors, const Point3DVector& p3DFeature, vector<int>& inliers, double& error)
{
	// detect inliers and compute errors
	error = 0;
	inliers.clear();
	//Total observation vectors
	int size = ObsVectors.size();
	for (int i = 0; i < size; i++)
	{
		// cross product of  (camera position - feature 3D vector) norm and observation vector  
		double tmp_err;
		Point3D tmp = p3DFeature[i] - cameraPose;
		tmp = tmp / tmp.norm();
		tmp_err = (tmp.cross(ObsVectors[i])).squaredNorm();

		if (tmp_err < Config::threshErrRANSAC)
		{
			//cout << "best error:"<<error << endl;
			inliers.push_back(i);
			error += tmp_err;
		}
	}
	//rms error
	if (inliers.size() > 0)
	{
		//cout << inliers.size() << endl;
		error /= inliers.size();
		//cout << error << endl;
		error = sqrt(error);
	}
	else
	{
		error = 1E11;
	}
}
/*
Write:
Feature Observation
*/
void DroneSlam::checkAndDeleteCullTrack(vector<Point2f>& currKeyP, vector<Point2f>& preKeyP, vector<uchar>& status)
{
	// add feature observation to localMap
	int featureSize = preKeyP.size();
	Mat featuresObser = toFeatureObsv(currKeyP);
	Point2f tmpP;
	for (int k = 0; k < featureSize; k++)
	{
		// is not cull track
		if (status[k])
		{
			//get a feature observers
			tmpP = featuresObser.at<Point2d>(k, 0);
			//add the valid feature observations to local map
			localMap[k].featureObservation.push_back(Point3D(tmpP.x, tmpP.y, 1).normalized());
		}
		else //is cull
		{
			localMap[k].isCull = true;
		}

		// useless features
		//not either a mono pose or stereo pose and the current frame - first frame containing the feature is larger
		if (!localMap[k].isMonoPose && !localMap[k].isStereoPose && (frame->id - localMap[k].firstFrameID) > Config::minFeatureLiveChange)
		{
			localMap[k].isCull = true;
		}

		//Number of time it is a outliers > the maximum number of time 
		if (localMap[k].nOutliers > Config::minFeatureOutlierChange)
		{
			localMap[k].isCull = true;
		}
	}

	// delete all cull features in localMap + currKeyP
	int k = 0;
	while (k < localMap.size())
	{
		if (localMap[k].isCull)
		{
			if (k == localMap.size() - 1)//last element
			{
				localMap.pop_back();
				currKeyP.pop_back();
				preKeyP.pop_back();
			}
			else
			{
				// push cull feature
				cullFeatures.push_back(localMap[k]);
				// replace the current feature map with the back feature local map
				localMap[k] = localMap.back();
				localMap.pop_back();

				// currKeyP, then check with current new key point
				currKeyP[k] = currKeyP.back();
				currKeyP.pop_back();
				preKeyP[k] = preKeyP.back();
				preKeyP.pop_back();
			}
		}
		else
		{
			k++;
		}
	}
}

void DroneSlam::deleteCullTrack(vector<Point2f>& currKeyP)
{
	// delete all cull features in localMap + currKeyP
	int k = 0;
	while (k < localMap.size())
	{
		if (localMap[k].isCull)
		{
			if (k == localMap.size() - 1)
			{
				localMap.pop_back();
				currKeyP.pop_back();
			}
			else
			{
				// store cull features
				cullFeatures.push_back(localMap[k]);
				//store back local map to check
				localMap[k] = localMap.back();
				localMap.pop_back();

				// currKeyP
				currKeyP[k] = currKeyP.back();
				currKeyP.pop_back();
			}
		}
		else
		{
			k++;
		}
	}
}
//using undistortPoint  convert points to pointsMat
cv::Mat DroneSlam::toFeatureObsv(vector<cv::Point2f> points)
{
	//convert vector point -> matrix (n,2)

	Mat pointsMat(points);
	pointsMat.convertTo(pointsMat, CV_64FC2);
	//undistored Points for observation in normalized image coordinate
	undistortPoints(pointsMat, pointsMat, stream->getCamParams().cm1, stream->getCamParams().d1);
	return pointsMat;
}

Point3D DroneSlam::reinstallCameraPoseStereo(FeatureStorageVector& localMap, Point3D& preCameraPos)
{
	//reinstall new camera with local map and previous camera pos
	Point3D result;
	int size = localMap.size();
	vector <int> featuresHave3D;
	vector <float> distanceOfFeature;

	Point3DVector p3D_OfFeature;

	for (int i = 0; i < size; i++)
	{
		if (localMap[i].isStereoPose)
		{
			//! calculate d = || Wp - R(t-1) || world point - pre camera position
			featuresHave3D.push_back(i);
			float tmp = (localMap[i].featurePoseStereo - preCameraPos).norm();
			distanceOfFeature.push_back(tmp);

			//! 3D point of feature
			p3D_OfFeature.push_back(localMap[i].featurePoseStereo);

		}
	}

	// calculate A = I3 - uij*uij^T / d of all features have 3D pos
	// all A for RANSAC for computing sum of A
	Mat3x3Vector allA;
	vector <int> points;
	// all A*p for RANSAC for sum of A*p
	Point3DVector allAxp;
	Point3DVector all_uij;
	size = featuresHave3D.size();
	for (int i = 0; i < size; i++)
	{
		// calculate feature observation unrotated frame
		Point3D u_ij;

		//Observation Vector from Camera -> World = Rotation Mat * feature vector (I->C)
		u_ij = currR * localMap[featuresHave3D[i]].featureObservation.back();

		// to unit vector
		u_ij = u_ij / u_ij.norm();

		// A
		Mat3x3 A = Mat3x3::Identity() - u_ij * u_ij.transpose();
		A /= distanceOfFeature[i];

		// all_A
		allA.push_back(A);

		// A*p
		allAxp.push_back(A * p3D_OfFeature[i]);

		//all_uij
		all_uij.push_back(u_ij);

		//
		points.push_back(i);
	}
	result = computePose(allA, allAxp, points);
	return result;
}

bool DroneSlam::readIMU(float& roll, float& pitch, float& yaw, double& time)
{
	if (allRPY.size() == 0)
	{
		return false;
	}
	//convert to degree.
	//get the current frame id of rpy and time stamp.
	roll = (allRPY[frame->id].roll*1.0 * 180) / M_PI;
	pitch = (allRPY[frame->id].pitch*1.0 * 180) / M_PI;
	yaw = (allRPY[frame->id].yaw*1.0 * 180) / M_PI;
	time = allRPY[frame->id].timestamp;
	return true;
}

bool DroneSlam::readAllIMU()
{
	union float_ {
		float f;
		byte b[4];
	}IMU;

	// dataset offline reading IMU
	string pathIMU = "./DATA/all_IMU.txt";
	ifstream file(pathIMU);
	if (!file.is_open()) 
	{
		cout << "File is not available!" << endl;
	}
	string str;
	
	RPY firstRPY;
	int id = 0;
	while (!file.eof())
	{
		//float buffer
		float_ buff[BUFLEN];     
		//clear the buffer by filling null, it might have previously received data
		memset(buff, '\0', BUFLEN);

		double tmp;
		double time;
		int num = 31;
		//recieve data 1 at a time
		// there are 31 IMU value
		for (int i = 0; i < num; i++)
		{
			file >> tmp; //each time read a double value
			if (i == 0)
			{
				time = tmp;
			}
			if (i == 4)//roll
			{
				buff[4].f = tmp;
			}
			if (i == 5)//pitch
			{
				buff[5].f = tmp;
			}
			if (i == 6)//yaw
			{
				buff[6].f = tmp;
			}
		}

		// R_W2I : rotation from W to I
		RPY tmpRPY;
		tmpRPY.timestamp = time;
		tmpRPY.roll = buff[4].f;
		tmpRPY.pitch = buff[5].f;
		tmpRPY.yaw = buff[6].f;
		
		if (id == 0)
		{
			firstRPY = tmpRPY;
			//save the first rpy
			this->beginRPY = firstRPY;
			tmpRPY.roll = tmpRPY.pitch = tmpRPY.yaw = 0;

		}
		else
		{
			//first rpy - current rpy
			tmpRPY.roll = + firstRPY.roll - tmpRPY.roll;
			tmpRPY.pitch = + firstRPY.pitch - tmpRPY.pitch;
			tmpRPY.yaw = + firstRPY.yaw - tmpRPY.yaw;
		}

		id++;
		allRPY.push_back(tmpRPY);
	}
	return true;
}

string float2string(float& number)
{
	ostringstream buff;
	buff << number;
	return buff.str();
}

void DroneSlam::processFrame()
{
	/*namedWindow("Tracker");
	setMouseCallback( "Tracker", onMouse, 0 );*/

	//! param for tracking
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size subPixWinSize(10, 10);
	Size winSize(10, 10);
	int numInliers = 0;
	float roll, pitch, yaw;
	//! run Img streamer thread
	thread thread_Img_stream(&CameraIMUStreamer::threadReadImg, this->stream);
	BoundingBox currBoundingBox;
	Rect2d bbox;
	Rect processBounding;
	BoundingBoxHelper boxHelper;
	BoundingBox croppedBoxResult;
	Point3DVector pointsInside2DBox;
	Rect originalBoundingBox;
	Mat detectFrame;
	bool checkDetect = false;
	float ratio = 1.2f;
	float tmpRatio = ratio;

	int left, top, right, bottom;
	int firstDetectedId =0;
	//Testing Tracker CSRT
	Ptr<Tracker> tracker = TrackerCSRT::create();
	//sleep 1 s
	this_thread::sleep_for(std::chrono::milliseconds(1000));
	while (1)
	{
		if (!isReinstall)
		{
			// camera streamer read frame
			if (stream == NULL || !stream->read(frame)) return;

			// IMU streamer

			double time;
			//roll = pitch = yaw = 0;
			readIMU(roll, pitch, yaw, time);

			lastRPY.pitch = pitch;
			lastRPY.roll = roll;
			lastRPY.yaw = yaw;
			lastRPY.timestamp = time;
			cout << "Time:"<<time<<"\n R = " << roll << " ,P = " << pitch << "  ,Y = " << yaw <<endl;
			
			// RW2I
			Mat3x3 R = keyPointConversion.RPY2Rotation(roll, pitch, yaw);

			// RI2W*RC2I
			Mat3x3 RC2I;
			RC2I << 0.000998747016376511, 0.00841690127419730, 0.99996410828547, -0.999990478045969, -0.00425082353508072, 0.00103455365356484, 0.00425937626100360, -0.999955514625927, 0.00841257580145709;
			//RC2I << 0.001, 0.0084, 1, -1, -0.0043, 0.0010, 0.0043, -1, 0.0084;

			R = R * RC2I;

			/*cout<< endl<< "R = "<<frame->id<<" : "<< R;
			waitKey(0);*/

			prevR = currR;
			currR = R;
			allR.push_back(R);
		}
		// Visual SLAM
		if (frame->id == 0 || isReinstall) //fire frame or set up again 
		{
			//! at t=0, Pos = (0, 0, 0)
			if (frame->id == 0)
			{
				cameraPos.push_back(Point3D(0, 0, 0));
			}
			else
			{
				cout << endl << "====================================================================="<<endl;
				isReinstall = false;
			}

			Mat mask = Mat::zeros(frame->mainFrame.size(), CV_8UC1);  // type of mask is CV_8U
			Mat roi(mask, cv::Rect(50, 50, 1142, 275));
			roi = Scalar(255);

			//! Shi-Tomasi conners for stereo corresponding  Ok
			vector<KeyPoint> keyPoints1 = detectAndTracker->detectKeyPoints(frame->mainFrame, mask);
			vector<KeyPoint> keyPoints2 = detectAndTracker->detectKeyPoints(frame->subFrame, mask);

			vector <Point2f> keyCurrP = keyPointConversion.KeyPoint2Point2f(keyPoints1);
			cornerSubPix(frame->mainFrame, keyCurrP, Size(10, 10), Size(-1, -1), termcrit);
			keyPoints1 = keyPointConversion.Point2f2KeyPoint(keyCurrP);
			keyCurrP.clear();

			keyCurrP = keyPointConversion.KeyPoint2Point2f(keyPoints2);
			cornerSubPix(frame->subFrame, keyCurrP, Size(10, 10), Size(-1, -1), termcrit);
			keyPoints2 = keyPointConversion.Point2f2KeyPoint(keyCurrP);

			// BRIEF description
			Mat descriptor1 = detectAndTracker->computeDescriptors(frame->mainFrame, keyPoints1);
			Mat descriptor2 = detectAndTracker->computeDescriptors(frame->subFrame, keyPoints2);

			// Brute Force matcher
			vector<DMatch> matches = detectAndTracker->matchTwoImage(descriptor1, descriptor2);


			// reject outlier by epipolar constraints
			vector<DMatch> stereoMatches;
			stereoMatches = detectAndTracker->rejectStereoOutliers(keyPoints1, keyPoints2, matches, stream->getCamParams().F_f);
			cout <<"Size of Stereo Matches"<< stereoMatches.size() << endl;
			//Reject outliers draw
			//Mat output;
			//drawMatches(frame->mainFrame, keyPoints1, frame->subFrame, keyPoints2, stereoMatches, output);
			//imshow("EPIPOLAR REJECT", output);
			//waitKey(0);


			// calculate features position from stereo
			vector <float> all_quality;
			Point3DVector point3Dstereo;
			point3Dstereo = triangulateStereo->myTriangulatePoints(keyPoints1, keyPoints2, stereoMatches, all_quality);
			
			//Show 3D points
			//int size_of_stereo = point3Dstereo.size();
			//cout << "Size of Stereo Points: " << size_of_stereo << endl;
			//Mat outputZ = frame->mainFrame;
			//vector <KeyPoint> tmpKeyP;

			//for (int ik = 0; ik < size_of_stereo; ik++)
			//{
			//	tmpKeyP.push_back(keyPoints1[stereoMatches[ik].queryIdx]);
			//	drawKeypoints(outputZ, tmpKeyP, outputZ);
			//	float x = point3Dstereo[ik][0];
			//	float y = point3Dstereo[ik][1];
			//	float z = point3Dstereo[ik][2];

			//	string out = "("+float2string(x) + ", " +float2string(y) + ", " + float2string(z) + ")";
			//	putText(outputZ, out, keyPoints1[stereoMatches[ik].queryIdx].pt, 1, 1, Scalar(0, 0, 255));
			//	tmpKeyP.pop_back();
			//	//
			//	cout<< endl<< "HERE"<<keyPoints1[stereoMatches[ik].queryIdx].pt<< "\t" << point3Dstereo[ik].transpose();
			//}
			//imshow("Z", outputZ);
			//waitKey(0);
			


			// add all points detect camera primary to local map

			addFeaturesToLocalMap(keyPoints1, stereoMatches, point3Dstereo, all_quality, frame->id, stream->getCamParams());

			// save last feature for next tracking
			vector <Point2f> currKeyP;
			currKeyP = keyPointConversion.KeyPoint2Point2f(keyPoints1);
			//Object Detection
			//first detected for full frame
			Mat image = frame->mainFrame;
			cvtColor(image, image, CV_GRAY2RGB);
			if (frame->id == 0)
			{
				objectDetection->objectDetect(image);
				currBoundingBox = objectDetection->getBestBoundingBox();
				firstDetectedId = currBoundingBox.getClassId();
			}
			//update if got reinstall still use the current surrounding bounding box
			else {
				int idxRatio = 0;
				Rect bRect = currBoundingBox.getRegion();
				while (!checkDetect)
				{
					tmpRatio += 0.1 * idxRatio;
					//get small surrounding frame
					processBounding = boxHelper.getNewBoundingBox(bRect, tmpRatio, image.rows, image.cols);
					//Surrounding Object For Detection
					detectFrame = image(processBounding);
					//Tracking
					//bool ok = tracker->update(image, bbox);
					//Detect
					checkDetect = objectDetection->objectDetect(detectFrame);
					idxRatio += 2;
				}
				checkDetect = false;
				//Get bounding box
				//get box with the same class Id with the original and also best Confidence.
				croppedBoxResult = objectDetection->getRelatedBoundingBox(firstDetectedId);
				//Convert to original size
				//width height the same
				originalBoundingBox = boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y);
				currBoundingBox.setRegion(originalBoundingBox);
			}
			//Rect tmpRect = currBoundingBox.getRegion();
			//findFeaturePoints(pointsInside2DBox, tmpRect);

			//bbox = Rect2d(tmpRect.x,tmpRect.y,tmpRect.width,tmpRect.height);
			//tracker->init(image, bbox);
			cout <<"Class ID:"<< firstDetectedId << endl;
			swap(currKeyP, prevKeyP);
		}
		else //next tracking frame
		{
			std::clock_t start = std::clock();
			//! tracking for monocular
			//! check last feature
			if (!prevKeyP.empty())
			{
				vector <Point2f> currKeyP;
				vector <uchar> status;
				vector <float> err;

				// Roi for detect feature
				Mat mask = Mat::zeros(frame->mainFrame.size(), CV_8UC1);  // type of mask is CV_8U
				Mat roi(mask, cv::Rect(50, 50, 1142, 275));
				roi = Scalar(255);

				//! tracking by KLT
				calcOpticalFlowPyrLK(frame->preMainFrame, frame->mainFrame, prevKeyP, currKeyP, status, err, winSize, 3, termcrit, 0, 0.001);
				Mat image = frame->mainFrame;
				//Detection with only the surrounding bounding box regions.
				cvtColor(image, image, CV_GRAY2RGB);
				Rect bRect = currBoundingBox.getRegion();
				//If the output is none
				//Increase the ratio of processing bounding box -> 1.2 1.4
				int idxRatio = 0;
				while (!checkDetect)
				{
					tmpRatio += 0.1*idxRatio;
					//get small surrounding frame
					processBounding = boxHelper.getNewBoundingBox(bRect, tmpRatio, image.rows, image.cols);
					//Surrounding Object For Detection
					detectFrame = image(processBounding);
					//Tracking
					//bool ok = tracker->update(image, bbox);
					//Detect
					checkDetect = objectDetection->objectDetect(detectFrame);
					idxRatio+=2;
				}
				checkDetect = false;

				//Get bounding box
				//get box with the same class Id with the original and also best Confidence.
				croppedBoxResult = objectDetection->getRelatedBoundingBox(firstDetectedId);
				//Convert to original size
				//width height the same
				originalBoundingBox = boxHelper.getOriginalBoundingBox(croppedBoxResult.getRegion(), processBounding.x, processBounding.y);
				//New bounding box
				//Set new bounding to currboundingBox(tracking)
				//currBoundingBox.setRegion(Rect(bbox));
				//Detection
				currBoundingBox.setRegion(originalBoundingBox);
				left = originalBoundingBox.x;
				top = originalBoundingBox.y;
				right = originalBoundingBox.x + originalBoundingBox.width;
				bottom = originalBoundingBox.y + originalBoundingBox.height;
				

				objectDetection->drawPrediction(currBoundingBox.getClassId(),currBoundingBox.getConfidence(),left,top,right,bottom,image);
				
				for (int k = 0; k < currKeyP.size(); k++)
				{
					if (!status[k])
						continue;
					//Green COlor
					if (localMap[k].isStereoPose)
					{
						circle(image, currKeyP[k], 3, Scalar(0, 255, 0), -1, 8);
					}
					//R color
					if (localMap[k].isMonoPose)
					{
						circle(image, currKeyP[k], 3, Scalar(0, 0, 255), -1, 8);
					}
					//White Color
					if (localMap[k].isStereoPose && localMap[k].isMonoPose)
					{
						circle(image, currKeyP[k], 3, Scalar(255, 255, 255), -1, 8);
					}
					//Black
					if (!localMap[k].isMonoPose && !localMap[k].isStereoPose)
					{
						circle(image, currKeyP[k], 3, Scalar(0, 0, 0), -1, 8);
					}
					//White 
					if (localMap[k].isInliers && (localMap[k].isStereoPose || localMap[k].isMonoPose))
					{
						circle(image, currKeyP[k], 5, Scalar(255, 255, 255), 2, 4);
					}
				}

				string RPYtext = "(" + to_string(roll) + ", " + to_string(pitch) + ", " + to_string(yaw) + ")";
				putText(image, RPYtext, Point(50, 50), 1, 2, Scalar(100, 255, 0), 2, 2);
				imshow("Tracker", image);
				waitKey(1);
			
				int size_Features;
				// add feature observation to localMap
				checkAndDeleteCullTrack(currKeyP, prevKeyP, status);

				// check if num of feature < thresh = 400, add features 
				if (currKeyP.size() < Config::threshFeatureTrack)
				{

					Mat Mask = Mat::zeros(frame->mainFrame.size(), CV_8UC1);  // type of mask is CV_8U
					Mat pRoi(Mask, cv::Rect(50, 50, 1142, 275));
					pRoi = Scalar(255);

					uchar zeroToMask = 0;
					cv::Rect rect(cv::Point(), Mask.size());

					for (int k = 0; k < currKeyP.size(); k++)
					{
						if (rect.contains(currKeyP[k]) && rect.contains(Point(currKeyP[k].x - 10, currKeyP[k].y - 10)))
						{
							pRoi = Mask(cv::Rect(currKeyP[k].x - 10, currKeyP[k].y - 10, 10, 10));
							//Mask.at<uchar>(Point(currKeyP[k].x,currKeyP[k].y)) = zeroToMask;
							pRoi.setTo(0);
						}
					}
					int n_addFeature = Config::maxFeatureTrack - currKeyP.size();
					vector<Point2f> addFeature = detectAndTracker->detectAddFeature(frame->mainFrame, Mask, n_addFeature);
					addMoreFeaturesToLocalMap(addFeature, frame->id, stream->getCamParams());
					currKeyP.insert(currKeyP.end(), addFeature.begin(), addFeature.end());
				}


				// calculate camera position at t = frame->id
				// using RANSAC

				Point3D curr_CameraPos = calculateCameraPose(localMap, cameraPos.back(), numInliers);
				int count = 0;
				while (numInliers == 0)
				{
					count++;
					curr_CameraPos = calculateCameraPose(localMap, cameraPos.back(), numInliers);
					if (count > 10)
					{
						localMap.clear();
						currKeyP.clear();
						isReinstall = true;
						break;
					}

				}
				if (isReinstall)
				{
					swap(prevKeyP, currKeyP);
					continue;
				}
				//waitKey(0);

				//! new camera pose
				cameraPos.push_back(curr_CameraPos);
				cout<<("\nID = %d:\t %f %f %f", frame->id, curr_CameraPos[0], curr_CameraPos[1], curr_CameraPos[2]);
				cout<<("\nGPS  =:\t\t  %f %f %f", GPSPose[frame->id][0], GPSPose[frame->id][1], GPSPose[frame->id][2]);
				
				Point3D err_p;
				for (int i = 0; i < 3; i++)
				{
					err_p[i] = abs(curr_CameraPos[i] - GPSPose[frame->id][i]);
				}
				errPose.push_back(err_p);
				//print error
				cout<<("\nERROR = :\t\t %f %f %f \n\n\n", err_p[0], err_p[1], err_p[2]);
				circle(result1, Point(curr_CameraPos[0], curr_CameraPos[1] + 150), 1, Scalar(0, 255, 0), 1, 8);
				//result2.at<Vec3b>(Point(frame->id*2, (curr_CameraPos[0]-gps_pose[frame->id][0])*10+150)) = Vec3b(0, 255, 0);
				stream->outGPSandPose << curr_CameraPos[0] << "," << curr_CameraPos[1] << "," << curr_CameraPos[2] << endl;
				imshow("result", result1);
				waitKey(1);
				//waitKey(0);
				// have current camera pos, calculate stereo pos if have stereo img
				if (frame->isStereo)
				{
					cout << "\n" << "STEREO O==O";
					vector<KeyPoint> keyP1, keyP2;
					// check keyP in currKeyP, if keyP can't extract BRIEF -> reject
					vector <int> map;
					for (int ik = 0; ik < currKeyP.size(); ik++)
					{
						//(50,50,1142,275));
						//! out of extract range
						if (currKeyP[ik].x < 50 || currKeyP[ik].x > 1142 || currKeyP[ik].y < 50 || currKeyP[ik].y > 275)
						{
						}
						else
						{
							keyP1.push_back(KeyPoint(currKeyP[ik], 1.0f));
							map.push_back(ik);
						}
					}
					// already have keyPoint for Primary img, detect keyPoint for Secondary img
					//keyP1 = Point2f2KeyPoint(currKeyP);
					keyP2 = detectAndTracker->detectKeyPoints(frame->subFrame, mask);

					// compute BRIEF description
					Mat descriptor1 = detectAndTracker->computeDescriptors(frame->mainFrame, keyP1);
					Mat descriptor2 = detectAndTracker->computeDescriptors(frame->subFrame, keyP2);

					// matcher
					vector<DMatch> matches = detectAndTracker->matchTwoImage(descriptor1, descriptor2);

					// reject outlier by epipolar constrains
					vector<DMatch> stereoMatches;
					stereoMatches = detectAndTracker->rejectStereoOutliers(keyP1, keyP2, matches, stream->getCamParams().F_f);

					//// DEBUG----outliers reject //////////////////////////////////////////////////////////////////////////
					/*Mat output;
					drawMatches(frame->img_Primary, keyP1, frame->img_Stereo, keyP2, stereoMatches, output);
					imshow("EPIPOLAR REJECT", output);
					waitKey(0);*/
					////////////////////////////////////////////////////////////////////////////////////////////////////////

					// calculate features pose from stereo
					Point3DVector point3Dstereo;
					vector <float> all_quality;
					point3Dstereo = triangulateStereo->myTriangulatePoints(keyP1, keyP2, stereoMatches, all_quality);

					//Move to world coordinate and update map
					updateLocalMapStereo(point3Dstereo, all_quality, stereoMatches, currR, cameraPos.back(), map);

				}

				// have current camera pos, Compute monocular pos
				size_Features = localMap.size();
				for (int k = 0; k < size_Features; k++)
				{
					if (!localMap[k].isMonoPose && localMap[k].firstFrameID != frame->id)
					{
						float quality;
						Point3D featurePoint;
						featurePoint = calculateFeaturePos(cameraPos[localMap[k].firstFrameID], localMap[k].featureObservation.front(), cameraPos[frame->id], localMap[k].featureObservation.back(), quality, allR[localMap[k].firstFrameID]);
						if (quality > Config::threshQualityFeature && !cvIsInf(quality))
						{
							localMap[k].isMonoPose = true;
							localMap[k].featurePoseMono = featurePoint;
						}
					}
				}

				// check error
				cout << endl << "Local Map Size = " << size_Features;
				int haveStereo3D = 0;
				int haveMono3D = 0;
				for (int i = 0; i < size_Features; i++)
				{
					if (localMap[i].isStereoPose)
					{
						haveStereo3D++;
					}
					if (localMap[i].isMonoPose)
					{
						haveMono3D++;
					}
				}
				cout << endl << "Have STEREO = " << haveStereo3D;
				cout << endl << "Have MONO = " << haveMono3D;

				// check if have error in system
				int khi = 0;
				double err_gamma = 0;
				for (int i = 0; i < localMap.size(); i++)
				{
					if (localMap[i].isMonoPose && localMap[i].isStereoPose)
					{
						err_gamma += (localMap[i].featurePoseMono - cameraPos.back()).norm() / (localMap[i].featurePoseStereo - cameraPos.back()).norm();
						khi++;
					}
				}
				if (khi > Config::threshPointWarning)
				{
					err_gamma /= khi;

					//! if err_gamma > thresh -> failure
					if (err_gamma > Config::threshGammaMax || err_gamma < Config::threshGammaMin)
					{
						localMap.clear();
						currKeyP.clear();

						// reinstall map
						isReinstall = true;

					}
				}

				//! Feature points save
				swap(prevKeyP, currKeyP);
			}
			std::cout << endl << "printfTOTAL: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << '\n';

		}
	}
	thread_Img_stream.join();
	//IMU_thread.join();
}

void DroneSlam::drawGPSResult(Mat& result)
{
	string pathIMU = "./DATA/all_GPS_pose.txt";
	ifstream file(pathIMU);
	string str;
	if (!file.is_open ()) {
		cout << "File is not valid!" << endl;
	}
	Point2f point;
	Point2f prePoint(0, 0);
	float z; // value

	while (!file.eof()) {
		file >> point.x >> point.y >> z;
		GPSPose.push_back(Point3D(point.x,point.y,z));
		//draw point on map
		circle(result, Point(point.x, point.y + 150),1,Scalar(255,0,0));//
	}
	file.close();
}


void DroneSlam::findFeaturePoints(Point3DVector& pointsInside2DBox, Rect boundingBox)
{
	//A point inside the dimension of a 2D bounding box
	// Only investigate the x,y dimension
	int left = boundingBox.x;
	int top = boundingBox.y;
	int right = boundingBox.x + boundingBox.width;
	int bottom = boundingBox.y + boundingBox.height;
	int size = localMap.size();
	for (int i = 0;i < size;i++)
	{
		if (localMap[i].isInliers)
		{
			
		}
	}


	//Point3DVector p3D_OfFeature;
	//int nStereo = 0;
	//int nMono = 0;
	//for (int i = 0; i < size; i++)
	//{
	//	//if current  local Map is stereo
	//	if (localMap[i].isStereoPose)
	//	{

	//		// calculate d = ||R(t-1) - Wpi ||
	//		featuresHave3D.push_back(i);
	//		float tmp = (localMap[i].featurePoseStereo - preCameraPos).norm();
	//		distanceOfFeature.push_back(tmp);

	//		// 3D point of feature
	//		p3D_OfFeature.push_back(localMap[i].featurePoseStereo);
	//		nStereo++;
	//	}
	//	else//Mono Pose
	//	{
	//		if (localMap[i].isMonoPose)
	//		{
	//			// calculate d = || p - (r)t-1 ||
	//			featuresHave3D.push_back(i);
	//			float tmp = (localMap[i].featurePoseMono - preCameraPos).norm();
	//			distanceOfFeature.push_back(tmp);

	//			//! 3D point of feature
	//			p3D_OfFeature.push_back(localMap[i].featurePoseMono);
	//			nMono++;
	//		}
	//	}
	//}
}

DroneSlam::DroneSlam()
{
	//streamer from Cameras and IMU
	//get information of camera parameter
	stream = new CameraIMUStreamer();
	stream->connect();
	//init Stereo Triangulate with Camera Params
	triangulateStereo = new StereoTriangulate(stream->getCamParams());
	// tracker and detector feature
	detectAndTracker = new FeatureDetectTrack();
	//init new frame
	frame.reset(new CamerasIMUFrame());

	prevR = Mat3x3::Identity();
	currR = Mat3x3::Identity();

	//Detector
	objectDetection = new YOLOObjectDetection();
	// initial RPY
	lastRPY.roll = lastRPY.pitch = lastRPY.yaw = 180;

	curIMUId = 0;
	// read all data from IMU
	readAllIMU();

	result1 = Mat(333, 333, CV_8UC3, Scalar(0, 0, 0));
	/*result2 = Mat(1000,1000, CV_8UC3, Scalar(0, 0, 0));*/
	drawGPSResult(result1);

	isReinstall = false;
}

DroneSlam::~DroneSlam()
{
	delete objectDetection;
	delete detectAndTracker;
	delete triangulateStereo;
}
