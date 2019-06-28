#pragma once
#include "InputStreamer.h"
#include"Types.h"
#include"RPY.h"
#include "Config.h"
#include<winsock2.h>

#pragma comment(lib,"ws2_32.lib")// Winsock Library
class IMUPackage
{
private:
	//IMU socket streamer
	SOCKET s;
	sockaddr_in server, si_other;
	int slen, recv_len;
	WSADATA wsa;
	clock_t start;

	queue<RPY> IMUqueue;
public:
	bool readIMU(float& roll, float& pitch, float& yaw);
	void threadReadIMU();
	IMUPackage();
	~IMUPackage();

};

