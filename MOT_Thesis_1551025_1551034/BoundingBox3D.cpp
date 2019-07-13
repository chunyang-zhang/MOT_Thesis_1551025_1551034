#include "BoundingBox3D.h"

void BoundingBox3D::setBoundingBox3D(Point3DVector bbox3D)
{
	this->bbox3D = bbox3D; 
}

Point3DVector BoundingBox3D::getBoundingBox3D()
{
	return bbox3D;
}
