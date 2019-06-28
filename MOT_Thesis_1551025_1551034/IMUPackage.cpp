#include "IMUPackage.h"

IMUPackage::IMUPackage()
{
	//IMU streamer
	slen = sizeof(si_other);
	//Initialize winsock
	cout << "\nInitialising Winsock..." << endl;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		cout << "Failed. Error Code: "<<WSAGetLastError() << endl;
		exit(EXIT_FAILURE);
	}
	cout << "Initialized\n";
	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		cout << "Could not create socket :" << WSAGetLastError() << endl;;
	}
	cout<<"Socket created."<<endl;
	
	//Prepare sockaddr_in structure
	server.sin_family = AF_INET;
	//can be used for most tcp/ip call
	server.sin_addr.S_un.S_addr = INADDR_ANY;
	server.sin_port = htons(PORT);
	//Bind
	if (::bind(s, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("Bind done");
}
IMUPackage::~IMUPackage()
{
	closesocket(s);
	WSACleanup();
}
bool IMUPackage::readIMU(float& roll, float& pitch, float& yaw)
{
	if (IMUqueue.size() == 0) {
		return false;
	}
	//get IMU value
	RPY tmp = IMUqueue.back();
	roll = tmp.roll;
	pitch = tmp.pitch;
	yaw = tmp.yaw;
	return true;
}
void IMUPackage::threadReadIMU()
{
	union float_ {
		float f;
		byte b[4];
	}IMU;

	float_ buff2[BUFLEN];
	cout << "Waiting for data.." << endl;
	fflush(stdout);
	//clear the buffer by filling null, it might have previously received data
	memset(buff2, '\0', BUFLEN);
	//try to receive some data
	if ((recv_len = recvfrom(s, (char*)buff2, BUFLEN, 0, (struct sockaddr*) & si_other, &slen)) == SOCKET_ERROR)
	{
		cout<<"recvfrom() failed with error code : "<< WSAGetLastError();
		exit(EXIT_FAILURE);
	}
	//print details of the client and data received
	cout<<"Data: R: "<< buff2[1].f<<"P: "<< buff2[2].f<<"Y: "<< buff2[6].f;	//1 2 6
	RPY tmp;
	tmp.roll = buff2[1].f; //R
	tmp.pitch = buff2[2].f;
	tmp.roll = buff2[6].f;
	//IMU inputs smaller than RANSAC loop
	if (IMUqueue.size() < Config::loopRANSAC)
	{
		IMUqueue.push(tmp);
	}
	else {
		IMUqueue.pop();
		IMUqueue.push(tmp);
	}
}