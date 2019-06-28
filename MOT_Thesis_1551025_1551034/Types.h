#pragma once
#include "Utils.h"
#include <Eigen/Core>
#include<Eigen/StdVector>
using namespace Eigen;
typedef Vector3f Point3D; //point3D of Vector3f
typedef Matrix3f Mat3x3; //Matrix 3x3
typedef vector<Point3D, aligned_allocator<Point3D>> Point3DVector;
typedef vector<Mat3x3, aligned_allocator<Mat3x3>> Mat3x3Vector;

