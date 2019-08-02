#include "CamerasIMUFrame.h"

void CamerasIMUFrame::releaseFrame()
{
	if (!preMainFrame.empty())
	{
		preMainFrame.release();
	}
	if (!mainFrame.empty())
	{
		mainFrame.release();
	}
	if (!subFrame.empty())
	{
		subFrame.release();
	}
}

CamerasIMUFrame::~CamerasIMUFrame()
{
	preMainFrame.release();
	mainFrame.release();
	subFrame.release();
}
