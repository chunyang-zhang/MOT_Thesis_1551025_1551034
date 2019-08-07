#include "DroneSlam.h"
using namespace cv;

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
	size =(int) featuresHave3D.size();
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
	numOfInliers =(int) resultInliers.size();

	return currCameraPos;
}

Point3D DroneSlam::calculate3DObjectPos(const FeatureStorageVector& localMap, const Point3D& cameraPos, vector<int>& pInsideBoxIndex, Point3DVector& points3DInsideBox)
{

	//Vector Store the distance from feature pose - preCamera pose
	vector <float> distanceOfFeature;
	points3DInsideBox.clear();
	float distance;
	for (int i = 0; i < pInsideBoxIndex.size(); i++)
	{
		//if current  local Map is stereo
		if (localMap[pInsideBoxIndex[i]].isStereoPose)
		{

			// calculate d = ||R(t-1) - Wpi ||

			distance = (localMap[pInsideBoxIndex[i]].featurePoseStereo - cameraPos).norm();
			distanceOfFeature.push_back(distance);

			// 3D point of feature
			points3DInsideBox.push_back(localMap[pInsideBoxIndex[i]].featurePoseStereo);
		}
		else//Mono Pose
		{
			if (localMap[pInsideBoxIndex[i]].isMonoPose)
			{
				// calculate d = || p - (r)t-1 ||
				distance = (localMap[pInsideBoxIndex[i]].featurePoseMono - cameraPos).norm();
				distanceOfFeature.push_back(distance);

				//! 3D point of feature
				points3DInsideBox.push_back(localMap[pInsideBoxIndex[i]].featurePoseMono);
			}
		}
	}
	//Using IQR to remove outliers of distance
	if (pInsideBoxIndex.size() > 1)
	{
		removeOutliersIQR(pInsideBoxIndex, distanceOfFeature, points3DInsideBox);
	}
	Point3D objectPosition = Point3D(0, 0, 0);
	for (size_t i = 0;i < points3DInsideBox.size();i++)
	{
		objectPosition = objectPosition + points3DInsideBox[i];
	}
	objectPosition /= points3DInsideBox.size();
	return objectPosition;
}

void DroneSlam::removeOutliersIQR(vector<int>& pInsideBoxIndex, vector<float>& distanceOfFeature, Point3DVector& p3DOfFeature)
{
	//sort the distance of feature along with the Index of it
	sortPointByDistance(pInsideBoxIndex, distanceOfFeature, p3DOfFeature,0,pInsideBoxIndex.size()-1);
	int n = pInsideBoxIndex.size();
	int m;
	int q1;
	int q3;
	float IQR;
	float Q1;
	float Q3;
	float leftB;
	float rightB;
	if (n % 2 == 0)
	{
		m = n / 2;
		if (m % 2 == 0)
		{
			q1 = m / 2;
			q3 = m + q1;
			Q1 = (distanceOfFeature[q1 - 1] + distanceOfFeature[q1]) / 2;
			Q3 = (distanceOfFeature[q3 - 1] + distanceOfFeature[q3]) / 2;

		}
		else
		{
			q1 = (m + 1) / 2;
			q3 = m + q1;
			Q1 = distanceOfFeature[q1 - 1];
			Q3 = distanceOfFeature[q3 - 1];
		}
	}
	else
	{
		m = (n + 1) / 2;
		if (m % 2 == 0)
		{
			q1 = m / 2;
			q3 = m + q1;
			Q1 = distanceOfFeature[q1 - 1];
			Q3 = distanceOfFeature[q3 - 1];
		}
		else
		{
			q1 = m / 2;
			q3 = m + q1;
			Q1 = (distanceOfFeature[q1 - 1] + distanceOfFeature[q1]) / 2;
			Q3 = (distanceOfFeature[q3 - 1] + distanceOfFeature[q3]) / 2;

		}
	}
	IQR = Q3 - Q1;
	leftB = Q1 - 1.5 * IQR;
	rightB = Q3 + 1.5 * IQR;
	//remove outlier by IQR
	size_t i = 0;
	while (i < pInsideBoxIndex.size())
	{
		if (distanceOfFeature[i] < leftB|| distanceOfFeature[i] > rightB)
		{
			distanceOfFeature.erase(distanceOfFeature.begin() + i);
			pInsideBoxIndex.erase(pInsideBoxIndex.begin() + i);
			p3DOfFeature.erase(p3DOfFeature.begin() + i);
		}
		else
		{
			i++;
		}
	}
}

void DroneSlam::sortPointByDistance(vector<int>& pInsideBoxIndex, vector<float>& distanceOfFeature, Point3DVector& p3DOfFeature,int low, int high)
{
	int i = low;
	int j = high;
	float pivot = distanceOfFeature[(low + high) / 2];
	while(i<j)
	{
		while (distanceOfFeature[i] < pivot)
		{
			i++;
		}
		while (distanceOfFeature[j] > pivot)
		{
			j--;
		}
		if (i <= j)
		{
			swap(distanceOfFeature[i], distanceOfFeature[j]);
			swap(pInsideBoxIndex[i], pInsideBoxIndex[j]);
			swap(p3DOfFeature[i], p3DOfFeature[j]);
			i++;
			j--;
		}
		
	}
	if (low < j)
	{
		sortPointByDistance(pInsideBoxIndex, distanceOfFeature, p3DOfFeature, low, j);
	}
	if (i < high)
	{
		sortPointByDistance(pInsideBoxIndex, distanceOfFeature, p3DOfFeature, i, high);
	}
}

void DroneSlam::getBoundingBox3DWCS(const Point3DVector& p3DOfFeature, Point3DVector& bbox3D)
{
	bbox3D.clear();
	float minX, maxX;
	minX =maxX = p3DOfFeature[0].x();
	float minY, maxY;
	minY = maxY = p3DOfFeature[0].y();
	float minZ, maxZ;
	minZ = maxZ = p3DOfFeature[0].z();
	float x, y, z;//point 3D
	for (size_t i = 1;i < p3DOfFeature.size();i++)
	{
		x = p3DOfFeature[i].x();
		y = p3DOfFeature[i].y();
		z = p3DOfFeature[i].z();
		if (x > maxX)
		{
			maxX = x;
		}
		if(x < minX)
		{
			minX = x;
		}
		if (y > maxY)
		{
			maxY = y;
		}
		if (y < minY)
		{
			minY = y;
		}
		if (z > maxZ)
		{
			maxZ = z;
		}
		if (z < minZ)
		{
			minZ = z;
		}
	}
	//1
	bbox3D.push_back(Point3D(minX, maxY, minZ));
	//2
	bbox3D.push_back(Point3D(minX, maxY, maxZ));
	//3
	bbox3D.push_back(Point3D(maxX, maxY, maxZ));
	//4
	bbox3D.push_back(Point3D(maxX, maxY, minZ));
	//5
	bbox3D.push_back(Point3D(minX, minY, minZ));
	//6
	bbox3D.push_back(Point3D(minX, minY, maxZ));
	//7
	bbox3D.push_back(Point3D(maxX, minY, maxZ));
	//8
	bbox3D.push_back(Point3D(maxX, minY, minZ));

}

void DroneSlam::convert3DBoxto2DBox(const Point3DVector& bbox3D, vector<cv::Point2f>& box2D, const Mat3x3& R,const int& imgWidth,const int& imgHeight)
{
	box2D.clear();
	Point2f tmpPoint;
	for (size_t i = 0;i < bbox3D.size();i++)
	{
		computePixelCoordinates(bbox3D[i],R,2,2,imgWidth,imgHeight,tmpPoint);
		box2D.push_back(tmpPoint);
	}
}

void DroneSlam::draw2DBoundingBox(Mat& image, vector<cv::Point2f>& box2D)
{
	Scalar color = (0, 255, 255);
	int thickness = 2;
	line(image, box2D[0], box2D[1], color, thickness);
	line(image, box2D[0], box2D[3], color, thickness);
	line(image, box2D[0], box2D[4], color, thickness);
	line(image, box2D[1], box2D[2], color, thickness);
	line(image, box2D[1], box2D[5], color, thickness);
	line(image, box2D[2], box2D[3], color, thickness);
	line(image, box2D[2], box2D[6], color, thickness);
	line(image, box2D[3], box2D[7], color, thickness);
	line(image, box2D[4], box2D[5], color, thickness);
	line(image, box2D[4], box2D[7], color, thickness);
	line(image, box2D[5], box2D[6], color, thickness);
	line(image, box2D[6], box2D[7], color, thickness);

}

bool DroneSlam::computePixelCoordinates(const Point3D& pWorld, const Mat3x3& cameraToWorld, const float& canvasWidth, const float& canvasHeight, const int& imgWidth, const int& imgHeight, Point2f& pPixel)
{
	//compute the inverse of cameraToWorld
	Point3D pCamera;
	Mat3x3 worldToCamera = cameraToWorld.inverse();
	pCamera = worldToCamera * pWorld;
	//Coordinates of the point on the canva
	Point2f pScreen;
	pScreen.x = pCamera.x() / -pCamera.z();
	pScreen.y = pCamera.y() / -pCamera.z();

	//check with the canvas width and height, if the x or y coordinate absolute value is
	//greater than the canvas size, the point is not visible.
	if (abs(pScreen.x) > canvasWidth || abs(pScreen.y) > canvasHeight)
		return false;
	//Normalized coordinate will in range [0,1]
	Point2f pNDC;
	pNDC.x = (pScreen.x + canvasWidth / 2) / canvasWidth;
	pNDC.y = (pScreen.y + canvasHeight / 2) / canvasHeight;
	//Convert to pixel coordiante
	pPixel.x = floor(pNDC.x * imgWidth);
	pPixel.y = floor((1 - pNDC.y) * imgHeight);
	return true;
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
	size =(int) featuresHave3D.size();
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

Point3D DroneSlam::convertFromCameraToWorld(const Point3D& camPos, const Mat3x3& cameraToWorld)
{
	return cameraToWorld * camPos;
}

OutputPose DroneSlam::getOutputPose()
{
	return outputPose;
}

OutputTracking DroneSlam::getTrackingResult()
{
	return trackingResult;
}

vector<TrackingGroundTruth> DroneSlam::getObjectLocalizationResult()
{
	return objectLocalizationResult;
}

void DroneSlam::setGroundTruthValue(vector<TrackingGroundTruth> groundTruthValue)
{
	groundTruthList = groundTruthValue;
}

void DroneSlam::setTrackingMethod(string trackingMethod)
{
	this->trackingMethod = trackingMethod;
}

void DroneSlam::setRunningMethod(string runMethod)
{
	this->runMethod = runMethod;
}

bool DroneSlam::getIsTracked()
{
	return isTracked;
}

bool DroneSlam::getNoTracked()
{
	return noTracked;
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

	//Detection parameters
	BoundingBox currBoundingBox;
	BoundingBoxHelper boxHelper;
	
	Rect originalBoundingBox;

	bool checkDetect = false;
	int left, top, right, bottom;
	int firstDetectedId = 0;
	int countLost = 0;
	vector<int> pInsideBoxIndex;
	vector<float>featureDistances;

	//Object Pos
	Point3D objectPos;
	//Point3DVector bbox3D;
	Point3DVector points3DInsideBox;
	//vector<Point2f> pointsImage3DBox;
	//Descriptor Matrix
	Mat descriptor1;
	Mat descriptor2;
	//groundTruth Value
	Mat image;
	Mat groundTruthMat;
	Mat preBBoxFrame;
	TrackingStrategy* trackingStrategy = NULL;
	TrackingController trackingController;
	//GroundTruthResult
	TrackingGroundTruth trackingGroundTruth;
	int countIoU75 = 0;
	int countIoU50 = 0;
	int countObjPos = 0;

	//tracking result
	//IoU and AED of Pos
	float iou50 = 0;
	float iou75 = 0;
	float error = 0;
	int countTracking = 0;
	int countProcessTime = 0;
	float trackingTime = 0;
	//Output value 
	//Time, Number of Pic, Distance, Velocity, MSEx, MSEy,MSEz
	float time = 0;
	int numPics = stream->getNumFrames();
	float distance;
	float velocity;
	float avgFeatureTime = 0;
	float xMSE = 0;
	float yMSE = 0;
	float zMSE = 0;
	float aedCamPos;
	//Object Pose Error Result
	float aedPos = 0;
	float MSEPosx = 0;
	float MSEPosy = 0;
	float MSEPosz = 0;

	time = getTotalProcessTime();
	distance = getTotalDistance();
	velocity = distance / time;
	cout << "Time" << time << endl;
	cout << "Distance: " << distance << endl;
	cout << "Velocity: " << velocity << endl;
	cout << "Num of Pics: " << numPics << endl;

	outputPose.setTime(time);
	outputPose.setDistance(distance);
	outputPose.setNumPics(numPics);
	outputPose.setVelocity(velocity);
	//Testing Tracker CSRT
	Ptr<Tracker> tracker = TrackerCSRT::create();

	bool validReadFrame;
	//important only start tracking when meet this frame and so on
	int startTrackingFrame;
	if (groundTruthList.size())
	{
		startTrackingFrame = groundTruthList[0].getFrameId();
		isTracked = false;
	}
	else
	{
		return;
	}
	bool startTracking = false;
	string name;
	if (trackingMethod.compare("IoUMatching")==0)
	{
		name = "IoU + Image Matching";
	}
	else
	{
		name = trackingMethod;
	}
	//sleep 1 s
	this_thread::sleep_for(std::chrono::milliseconds(1000));
	while (1)
	{
		pInsideBoxIndex.clear();
		checkDetect = false;
		countProcessTime++;
		if (!isReinstall)
		{
			// camera streamer read frame

			validReadFrame = stream->read(frame);
			cout << "Frame ID: " << frame->id << endl;
			if (runMethod.compare("demo") == 0 && frame->id == groundTruthList.size() + startTrackingFrame)
			{
				startTracking = false;
			}
			if (stream == NULL || !(validReadFrame) ||(runMethod.compare("test") == 0 && frame->id == groundTruthList.size() + startTrackingFrame))
			{
				cout << "Finish Tracking Object" << endl;
				//Output Error Pose
				xMSE /= errPose.size();
				yMSE /= errPose.size();
				zMSE /= errPose.size();
				avgFeatureTime /= (double)CLOCKS_PER_SEC;
				avgFeatureTime /= countProcessTime;
				aedCamPos = getAED();
				outputPose.setErrorPose(Point3D(xMSE, yMSE, zMSE));
				outputPose.setAED(aedCamPos);
				outputPose.setAvgFeatureTime(avgFeatureTime);
				
				//Output iou tracking
				iou50 = 1.0* countIoU50 / groundTruthList.size();
				iou75 = 1.0* countIoU75 / groundTruthList.size();
				trackingTime /= (double)CLOCKS_PER_SEC;
				trackingTime /= countTracking;
				trackingResult.setIoU50(iou50);
				trackingResult.setIoU75(iou75);
				trackingResult.setObjectName(objectDetection->getNameOfClass(firstDetectedId));
				trackingResult.setTime(trackingTime);
				trackingResult.setId(trackingGroundTruth.getId());

				//Object Pose Error
				if (countObjPos > 0)
				{
					aedPos /= countObjPos;
					MSEPosx /= countObjPos;
					MSEPosy /= countObjPos;
					MSEPosz /= countObjPos;
					//cout << "AED pos: " << aedPos << endl;
					//cout << "MSE Pos x: " << MSEPosx << endl;
					//waitKey(0);
					outputObjPose.setId(trackingGroundTruth.getId());
					outputObjPose.setAED(aedPos);
					outputObjPose.setErrPose(Point3D(MSEPosx, MSEPosy, MSEPosz));
					outputObjPose.setName(objectDetection->getNameOfClass(firstDetectedId));
					outputObjPose.setIsEmpty(false);
				}
				else
				{
					outputObjPose.setIsEmpty(true);
				}
				//stop tracking, only continue camera localization
				//startTracking = false;
				isTracked = true;
				stream->setStopTrack(isTracked);
				//Release Matrix
				if (!preBBoxFrame.empty())
				{
					preBBoxFrame.release();
				}
				delete trackingStrategy;
				break;
			}
			if (startTracking)
			{
				int frameCount = frame->id - startTrackingFrame;
				if (frameCount < groundTruthList.size())
				{
					trackingGroundTruth = groundTruthList[frameCount];
				}
			}
			// IMU streamer

			double time;
			//roll = pitch = yaw = 0;
			readIMU(roll, pitch, yaw, time);

			lastRPY.pitch = pitch;
			lastRPY.roll = roll;
			lastRPY.yaw = yaw;
			lastRPY.timestamp = time;
			cout << "Time:" << time << endl;
			cout<<"R = " << roll << ", P = " << pitch << ", Y = " << yaw <<endl;
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
			clock_t start = clock();

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
			//convert to bgr for  lucid
			//cvtColor(frame->mainFrame, frameColor, COLOR_GRAY2BGR);
			//cvtColor(frame->subFrame, subFrameColor, COLOR_GRAY2BGR);

			//Mat descriptor1 = detectAndTracker->computeDescriptors(frameColor, keyPoints1);
			//Mat descriptor2 = detectAndTracker->computeDescriptors(subFrameColor, keyPoints2);
			descriptor1 = detectAndTracker->computeDescriptors(frame->mainFrame, keyPoints1);
			descriptor2 = detectAndTracker->computeDescriptors(frame->subFrame, keyPoints2);

			// Brute Force matcher
			vector<DMatch> matches = detectAndTracker->matchTwoImage(descriptor1, descriptor2);
			avgFeatureTime += clock() - start;

			// reject outlier by epipolar constraints
			vector<DMatch> stereoMatches;
			stereoMatches = detectAndTracker->rejectStereoOutliers(keyPoints1, keyPoints2, matches, stream->getCamParams().F_f);
			cout <<"Size of Stereo Matches: "<< stereoMatches.size() << endl;
			
			//Reject outliers draw
			//Mat output;
			//drawMatches(frame->mainFrame, keyPoints1, frame->subFrame, keyPoints2, stereoMatches, output);
			//imshow("EPIPOLAR REJECT", output);
			//waitKey(0);


			// calculate features position from stereo
			vector <float> all_quality;
			Point3DVector point3Dstereo;
			point3Dstereo = triangulateStereo->myTriangulatePoints(keyPoints1, keyPoints2, stereoMatches, all_quality);
			
			//Show 3D points max min
			int size_of_stereo = point3Dstereo.size();
			cout << "Size of Stereo Points: " << size_of_stereo << endl;


			// add all points detect camera primary to local map

			addFeaturesToLocalMap(keyPoints1, stereoMatches, point3Dstereo, all_quality, frame->id, stream->getCamParams());

			// save last feature for next tracking
			vector <Point2f> currKeyP;
			currKeyP = keyPointConversion.KeyPoint2Point2f(keyPoints1);
			//Object Detection
			//first detected for full frame
			image = frame->mainFrame;
			cvtColor(image, image, CV_GRAY2RGB);

			if (frame->id ==startTrackingFrame)
			{
				//cout << "Class ID:" << firstDetectedId << endl;
				//read: startTrackingFrame, image, 
				//out: originalBoundingBox
				//return: firstDetectedId
				
				isTracked = getGroundTruthForTracking(startTrackingFrame,image,firstDetectedId,trackingGroundTruth,currBoundingBox);
				if (!isTracked)
				{
					stream->setCanTrack(false);
					delete trackingStrategy;
					break;
				}
				//Set up first bounding box

				originalBoundingBox = currBoundingBox.getRegion();

				left = originalBoundingBox.x;
				top = originalBoundingBox.y;
				right = originalBoundingBox.x + originalBoundingBox.width;
				bottom = originalBoundingBox.y + originalBoundingBox.height;

				countIoU50++;
				countIoU75++;
				checkDetect = true;

				preBBoxFrame = image(originalBoundingBox);
				trackingStrategy = trackingController.selectTrackingStrategy(trackingMethod, preBBoxFrame, originalBoundingBox, firstDetectedId);
				startTracking = true;
			}

			swap(prevKeyP, currKeyP);
			//clear point
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

				// tracking by KLT
				calcOpticalFlowPyrLK(frame->preMainFrame, frame->mainFrame, prevKeyP, currKeyP, status, err, winSize, 3, termcrit, 0, 0.001);
				image = frame->mainFrame;
				//Detection with only the surrounding bounding box regions.
				cvtColor(image, image, CV_GRAY2RGB);
				
				if (frame->id == startTrackingFrame)
				{
					//Get groundth param for tracking
					isTracked = getGroundTruthForTracking(startTrackingFrame, image, firstDetectedId, trackingGroundTruth, currBoundingBox);
					if (!isTracked)
					{
						stream->setCanTrack(false);
						delete trackingStrategy;
						break;
					}
					//Set up first bounding box
					originalBoundingBox = currBoundingBox.getRegion();


					left = originalBoundingBox.x;
					top = originalBoundingBox.y;
					right = originalBoundingBox.x + originalBoundingBox.width;
					bottom = originalBoundingBox.y + originalBoundingBox.height;
					countIoU50++;
					countIoU75++;
					checkDetect = true;
					preBBoxFrame = image(originalBoundingBox);

					trackingStrategy = trackingController.selectTrackingStrategy(trackingMethod, preBBoxFrame,originalBoundingBox, firstDetectedId);

					startTracking = true;
				}
				if (startTracking)
				{
					//draw groundtruth box not matter

					objectDetection->drawPrediction(trackingGroundTruth.getBoundingBox(), image, Scalar(0, 0, 255));

					//For all tracker, input: Frame, and original bbox
					
					//Tracking
					clock_t startTracking = clock();
					if (trackingStrategy)
					{
						checkDetect = trackingStrategy->update(image, originalBoundingBox);
					}
					trackingTime += clock() - startTracking;
					//Detect
					//Result
					int btmText = image.rows - 25;
					if(checkDetect)
					{
						countTracking++;
						currBoundingBox.setRegion(originalBoundingBox);
						//get bounding
						left = originalBoundingBox.x;
						top = originalBoundingBox.y;
						right = originalBoundingBox.x + originalBoundingBox.width;
						bottom = originalBoundingBox.y + originalBoundingBox.height;
					
						float iouResult = objectDetection->calculateIoU(originalBoundingBox,trackingGroundTruth.getBoundingBox());
						//outside
						if (iouResult >= 0.5)
						{
							countIoU50++;
						}
						if (iouResult >= 0.75)
						{
							countIoU75++;
						}
						Point2f point;
						for (int k = 0; k < currKeyP.size(); k++)
						{
							if (!status[k])
								continue;
							////Green Color
							point = currKeyP[k];
							if (localMap[k].isStereoPose || localMap[k].isMonoPose)
							{
								if (checkDetect)
								{
									if (point.x > left && point.x<right && point.y>top && point.y < bottom)
									{
										pInsideBoxIndex.push_back(k);

									}
								}
							}
						}
						//Compute 3D object Pos

						//get 3D box from points 3D
						//getBoundingBox3DWCS(points3DInsideBox, bbox3D);
						//convert from 3D WCS to 2D Image
						//convert3DBoxto2DBox(bbox3D, pointsImage3DBox, currR, image.cols, image.rows);
						for (size_t i = 0;i < pInsideBoxIndex.size();i++)
						{
							circle(image, currKeyP[pInsideBoxIndex[i]], 5, Scalar(255, 0, 0), 2, 4);
						}
						//draw2DBoundingBox(image, pointsImage3DBox);

							//Calculate object Pos
						if (pInsideBoxIndex.size() != 0)
						{
							countObjPos++;
							objectPos = calculate3DObjectPos(localMap, cameraPos.back(), pInsideBoxIndex, points3DInsideBox);
							cout << "3D object Pos: " << objectPos.x() << " " << objectPos.y() << " " << objectPos.z() << endl;
							Point3D gtPos = convertFromCameraToWorld(trackingGroundTruth.getObjectPos(), currR);
							//minus from camera Pos
							//objectPos = objectPos - cameraPos.back();
							error = (objectPos - gtPos).norm();
							aedPos += error;
							Point3D err_p;
							for (int i = 0; i < 3; i++)
							{
								err_p[i] = abs(objectPos[i] - gtPos[i]);
							}
							MSEPosx += pow(err_p[0], 2);
							MSEPosy += pow(err_p[1], 2);
							MSEPosz += pow(err_p[2], 2);
							//output the object pos to file
							stream->outObjectPose << frame->id << " " << trackingGroundTruth.getId() << " " << objectDetection->getNameOfClass(firstDetectedId) << " " << left << " " << top << " " << right - left << " " << bottom - top << " " << objectPos.x() << " " << objectPos.y() << " " << objectPos.z() << endl;
							cout << "Object AED error: " << error << endl;
						}

						//Draw Predict
						objectDetection->drawPrediction(currBoundingBox.getClassId(), currBoundingBox.getConfidence(), left, top, right, bottom, Scalar(25, 255, 0), image);
						//Draw Object Pose
						string objPosText = "Obj Pose (" + roundValue(objectPos.x(), 4) + ", " + roundValue(objectPos.y(), 4) + ", " + roundValue(objectPos.z(), 4) + ")";
						putText(image, objPosText, Point(25, btmText), 1, 1.5, Scalar(0, 137, 255), 2, 2);

					}
					//keep the same bounding box if cant not find any.
				

					


					putText(image, "Ground Truth", Point(25, btmText - 25), 1, 1.5, Scalar(0, 0, 255), 2, 2);
					putText(image, name + " + YOLO", Point(25, btmText - 50), 1, 1.5, Scalar(25, 255, 0), 2, 2);
					//Object Pos Text
				}

				Point2f point;
				for (int k = 0; k < currKeyP.size(); k++)
				{
					if (!status[k])
						continue;
					////Green Color
					/*point = currKeyP[k];
					if (localMap[k].isStereoPose || localMap[k].isMonoPose)
					{
						if (checkDetect)
						{
							if (point.x > left && point.x<right && point.y>top && point.y < bottom)
							{
								pInsideBoxIndex.push_back(k);

							}
						}
					}*/
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

				
				//Camera Pose Text
				string camPos = "Cam Pose (" + roundValue(cameraPos.back().x(), 4) + ", " + roundValue(cameraPos.back().y(), 4)+ ", "+ roundValue(cameraPos.back().z(), 4)+ ")";
				string RPYtext = "RPY (" + roundValue(roll, 4) + ", " + roundValue(pitch, 4) + ", " + roundValue(yaw, 4) + ")";

				putText(image, RPYtext, Point(25, 25), 1, 1.5, Scalar(0, 0, 255), 2, 2);
				putText(image, camPos, Point(25, 50), 1, 1.5,Scalar(0, 255, 0), 2, 2);

				//Object Pos Text
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
					if (count > 30)
					{
						localMap.clear();
						currKeyP.clear();
						pInsideBoxIndex.clear();
						isReinstall = true;
						break;
					}

				}

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
				xMSE += pow(err_p[0],2);
				yMSE += pow(err_p[1],2);
				zMSE += pow(err_p[2],2);
				
				//print error
				cout<<("\nERROR = :\t\t %f %f %f \n\n\n", err_p[0], err_p[1], err_p[2]);
				circle(result1, Point(curr_CameraPos[0], curr_CameraPos[1] +150), 1, Scalar(0, 255, 0), 1, 8);
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
					clock_t start = clock();
					// already have keyPoint for Primary img, detect keyPoint for Secondary img
					//keyP1 = Point2f2KeyPoint(currKeyP);
					keyP2 = detectAndTracker->detectKeyPoints(frame->subFrame, mask);
					//convert to bgr for  lucid

					descriptor1 = detectAndTracker->computeDescriptors(frame->mainFrame, keyP1);
					descriptor2 = detectAndTracker->computeDescriptors(frame->subFrame, keyP2);
					// matcher
					vector<DMatch> matches = detectAndTracker->matchTwoImage(descriptor1, descriptor2);
					avgFeatureTime += clock() - start;
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
				size_Features =(int) localMap.size();
				for (int k = 0; k < size_Features; k++)
				{
					if (!localMap[k].isMonoPose && localMap[k].firstFrameID != frame->id)
					{
						float quality;
						Point3D featurePoint;
						//Error some time occur here
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
		if (!image.empty())
		{
			image.release();
		}
		if (!descriptor1.empty())
		{
			descriptor1.release();
		}
		if (!descriptor2.empty())
		{
			descriptor2.release();
		}
		if (!groundTruthMat.empty())
		{
			groundTruthMat.release();
		}

	}
	thread_Img_stream.join();
	//IMU_thread.join();

}

void DroneSlam::setDetectDescriptorMethod(string detector, string descriptor)
{
	if (detectAndTracker)
	{
		detectAndTracker->setDetectDescriptorMethod(detector, descriptor);
	}
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

float DroneSlam::getTotalProcessTime()
{
	float time = 0;
	for (size_t i = 0; i < allRPY.size() - 1;i++)
	{
		if (allRPY[i].timestamp < 60 && allRPY[i].timestamp>59 && allRPY[i + 1].timestamp < 1 && allRPY[i + 1].timestamp>0)
		{
			time+= allRPY[i + 1].timestamp+60 - allRPY[i].timestamp;
		}
		else
		{
			time += allRPY[i+1].timestamp - allRPY[i].timestamp;
		}
	}
	return time;
}

float DroneSlam::getTotalDistance()
{
	float distance = 0.f;
	for (size_t i = 0;i < GPSPose.size() - 1;i++)
	{
		Point3D pose = GPSPose[i + 1] - GPSPose[i];
		distance += pose.norm();
	}
	return distance;
}

string DroneSlam::roundValue(float value, int decimal)
{
	stringstream stream;
	stream << fixed << setprecision(decimal) << value;
	string s = stream.str();
	return s;
}

bool DroneSlam::getGroundTruthForTracking(int startTrackingFrame, cv::Mat& image, int& firstDetectedId, TrackingGroundTruth& trackingGroundTruth, BoundingBox& currBBox)
{
	BoundingBoxHelper boxHelper;
	int frameCount = frame->id - startTrackingFrame;
	Mat groundTruthMat;
	if (frameCount < groundTruthList.size())
	{
		trackingGroundTruth = groundTruthList[frameCount];
			
	}
	groundTruthMat = image(trackingGroundTruth.getBoundingBox());
	bool checkValue = objectDetection->objectDetect(groundTruthMat);
	if (!checkValue)
	{
		return false;
	}
	currBBox = objectDetection->getBestBoundingBox();
	firstDetectedId = currBBox.getClassId();
	if (firstDetectedId == 7 || firstDetectedId == 6)
	{
		firstDetectedId = 2;
		currBBox.setClassId(firstDetectedId);
	}
	if (firstDetectedId == 1)
	{
		firstDetectedId = 0;
		currBBox.setClassId(firstDetectedId);
	}
	Rect originalBox = boxHelper.normalizeCroppedBox(trackingGroundTruth.getBoundingBox(), image.cols, image.rows);
	currBBox.setRegion(originalBox);
	groundTruthMat.release();
	return true;
}


OutputObjectPose DroneSlam::getOutputObjectPose()
{
	return outputObjPose;
}


float DroneSlam::getAED()
{
	float result = 0;
	for(size_t i = 0; i<errPose.size();i++)
	{
		result += errPose[i].norm();
	}
	result /= errPose.size(); 
	return result;
}


DroneSlam::DroneSlam(string outCamPose, string outObjectPose)
{
	//streamer from Cameras and IMU
	//get information of camera parameter
	stream = new CameraIMUStreamer(outCamPose,outObjectPose);
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
	//imageMatching
	imageMatching = new ImageMatching();
	// initial RPY
	lastRPY.roll = lastRPY.pitch = lastRPY.yaw = 180;

	curIMUId = 0;
	// read all data from IMU
	readAllIMU();

	result1 = Mat(333, 333, CV_8UC3, Scalar(0, 0, 0));
	/*result2 = Mat(1000,1000, CV_8UC3, Scalar(0, 0, 0));*/
	drawGPSResult(result1);

	isReinstall = false;

	isTracked = false;
	noTracked = false;
}

DroneSlam::~DroneSlam()
{
	if (objectDetection)
	{
		delete objectDetection;
	}
	if (detectAndTracker)
	{
		delete detectAndTracker;
	}
	if (triangulateStereo)
	{
		delete triangulateStereo;
	}
	if(imageMatching)
	{ 
		delete imageMatching;
	}
	if (stream)
	{
		delete stream;
	}
	//release matrix
	result1.release();
	result2.release();
	result3.release();
}
