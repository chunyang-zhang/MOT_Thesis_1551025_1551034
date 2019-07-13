#pragma once
#include "BoundingBox.h"
#include "Types.h"
class BoundingBox3D :
	public BoundingBox
{

private:
	Point3DVector bbox3D;
public:
	void setBoundingBox3D(Point3DVector bbox3D);
	Point3DVector getBoundingBox3D();
};

