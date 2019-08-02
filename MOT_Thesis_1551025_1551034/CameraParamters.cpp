#include "CameraParamters.h"

CameraParameters::~CameraParameters()
{
	cm1.release();
	cm2.release();
	invMat1.release();
	d1.release();
	d2.release();
	r.release();
	t.release();
	e.release();
	f.release();
	r1.release();
	r2.release();
	p1.release();
	p2.release();
	q.release();
	map1x.release();
	map1y.release();
	map2x.release();
	map2y.release();
}
