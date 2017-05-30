// sender.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stddef.h>
#include <sstream>
#include <fstream>
#include <chrono>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <opencv2/core/core.hpp>;
#include <opencv2/highgui/highgui.hpp>;
#include <opencv2/imgproc/imgproc.hpp>;
#include <opencv/cv.h>

#include <WinSock2.h>
#include <Windows.h>
#include <ws2tcpip.h>

#include <zed/Camera.hpp>

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"
#define DEFAULT_WEB_PORT "5000"
#define SERVER_IP "192.168.2.98"
//variable used to decide if the images travel in RAW or Compressed format
#define encode true

int main()
{
	while (true) {
		//information for the socket that sends the pictures to the OCULUS PC
		WSADATA wsaData;
		SOCKET ConnectSocket = INVALID_SOCKET;
		struct addrinfo *result = NULL,
			*ptr = NULL,
			hints;
		int iResult;

		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed with error: %d\n", iResult);
			continue;
			//return 1;
		}

		// Resolve the server address and port for information socket
		iResult = getaddrinfo(SERVER_IP, DEFAULT_PORT, &hints, &result);
		if (iResult != 0) {
			printf("getaddrinfo failed with error: %d\n", iResult);
			WSACleanup();
			continue;
			//return 1;
		}
		// Attempt to connect to an address until one succeeds
		for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
			// Create a SOCKET for connecting to server
			ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
				ptr->ai_protocol);  
			if (ConnectSocket == INVALID_SOCKET) {
				printf("socket failed with error: %ld\n", WSAGetLastError());
				WSACleanup();
				break;
				//return 1;
			}
			// Connect to server.
			iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
			if (iResult == SOCKET_ERROR) {
				closesocket(ConnectSocket);
				ConnectSocket = INVALID_SOCKET;
				continue;
			}
			break;
		}
		freeaddrinfo(result);

		if (ConnectSocket == INVALID_SOCKET) {
			printf("Unable to connect to server!\n");
			WSACleanup();
			continue;
			//return 1;
		}

		// Initialize the ZED Camera
		sl::zed::Camera* zed = 0;
		zed = new sl::zed::Camera(sl::zed::VGA);// , 15.f);
		sl::zed::InitParams init_parameters(sl::zed::MODE::QUALITY);
		sl::zed::ERRCODE zederr = zed->init(init_parameters);
		if (zederr != sl::zed::SUCCESS) {
			std::cout << "ERROR: " << sl::zed::errcode2str(zederr) << std::endl;
			delete zed;
			closesocket(ConnectSocket);
			WSACleanup;
			continue;
			//return -1;
		}
		//get some parameters of the zed camera and send it to the server
		unsigned int zedWidth = zed->getImageSize().width;
		unsigned int zedHeight = zed->getImageSize().height;
		unsigned int focal = zed->getParameters()->LeftCam.fx;
		unsigned char *parameters = NULL;
		parameters = (unsigned char *)malloc((sizeof(unsigned int) * 3) + (1 * sizeof(unsigned char)));
		parameters[0] = 'A';
		memcpy(&parameters[1], &zedWidth, sizeof(unsigned int));
		memcpy(&parameters[5], &zedHeight, sizeof(unsigned int));
		memcpy(&parameters[9], &focal, sizeof(unsigned int));
		//we send some initial parameters from the camera
		iResult = send(ConnectSocket, (const char *)parameters, (sizeof(unsigned int) * 3) + (1 * sizeof(unsigned char)), 0);
		if (iResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(ConnectSocket);
			WSACleanup();
			continue;
			//return 1;
		}

		//variables used to treat the information of each frame before sending it
		unsigned char *bufferL = NULL;
		unsigned char *bufferR = NULL;
		unsigned char *infoL = NULL;
		unsigned char *infoR = NULL;
		bufferL = (unsigned char *)malloc(4000000);
		bufferR = (unsigned char *)malloc(4000000);

		unsigned int sizeL = zedWidth * zedHeight * 4;
		unsigned int sizeR = zedWidth * zedHeight * 4;
		unsigned int size = 0;
		unsigned int bytesToSend = 0;
		int end = 0;
		char testL[6];
		char testR[6];

		//variables to measure the time we use to pass the images to oculus
		std::chrono::time_point<std::chrono::system_clock> startChrono, endChrono;
		startChrono = std::chrono::system_clock::now();
		endChrono = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds;
		double retrieve[10000];
		double protocol[10000];
		double sendData[10000];
		double codedData[10000];
		double sizeCompressed[10000];
		int i = 0;
		std::ofstream fs("retrieveLatency.txt");
		std::ofstream fs2("sendLatency.txt");
		std::ofstream fs3("protocolLatency.txt");
		std::ofstream fs4("codingData.txt");
		std::ofstream fs5("sizeCompressed.txt");

		//variables to code the images
		std::vector<uchar> buffL;
		std::vector<uchar> buffR;
		unsigned int sizeBUFFERL = 0;
		unsigned int sizeBUFFERR = 0;
		unsigned int capacityLeft = 0;
		cv::Mat image(zedHeight, zedWidth, CV_8UC4, 1);
		cv::Mat depth(zedHeight, zedWidth, CV_8UC4, 1);
		unsigned int capacityRigth = 0;
	
		//Main loop
		while (!end) {
			startChrono = std::chrono::system_clock::now();
			if (!zed->grab(sl::zed::SENSING_MODE::STANDARD, false, false)) {
				endChrono = std::chrono::system_clock::now();
				infoL = zed->retrieveImage(sl::zed::SIDE::LEFT).data;
				//endChrono = std::chrono::system_clock::now();
				elapsed_seconds = endChrono - startChrono;
				if (i < 10000) {
					retrieve[i] = (double)elapsed_seconds.count() * 1000; //ms
					fs << retrieve[i] << std::endl;
				}
				if (encode) {
					startChrono = std::chrono::system_clock::now();
					memcpy(image.data, infoL, zedWidth*zedHeight * 4 * sizeof(uchar));
					cv::imencode(".jpg", image, buffL);
					capacityLeft = buffL.capacity();
					sizeBUFFERL = buffL.size();
					size = sizeBUFFERL;
					endChrono = std::chrono::system_clock::now();
					elapsed_seconds = endChrono - startChrono;
					if (i < 10000) {
						sizeCompressed[i] = sizeBUFFERL;
						codedData[i] = (double)elapsed_seconds.count() * 1000; //ms
						fs4 << codedData[i] << std::endl;
						fs5 << sizeCompressed[i] << std::endl;
					}
				}
				else {
					size = sizeL;
				}

				//the programs attaches the labels of the protocol to the image information
				startChrono = std::chrono::system_clock::now();
				if (encode) {
					bufferL = (unsigned char *)realloc(bufferL, (size + 12));
					bufferL[0] = 'a';
					bufferL[1] = 'a';
					bufferL[2] = 'L';
					bufferL[3] = 'Y';
					memcpy(&bufferL[4], &size, sizeof(unsigned int));
					memcpy(&bufferL[8], &capacityLeft, sizeof(unsigned int));
					memcpy(&bufferL[12], buffL.data(), sizeBUFFERL);
					bytesToSend = size + 12;
				}
				else {
					bufferL = (unsigned char *)realloc(bufferL, (size + 8));
					bufferL[0] = 'a';
					bufferL[1] = 'a';
					bufferL[2] = 'L';
					bufferL[3] = 'N';
					memcpy(&bufferL[4], &size, sizeof(unsigned int));
					memcpy(&bufferL[8], infoL, size);
					bytesToSend = size + 8;
				}

				endChrono = std::chrono::system_clock::now();
				elapsed_seconds = endChrono - startChrono;
				if (i < 10000) {
					protocol[i] = (double)elapsed_seconds.count() * 1000; //ms
					fs3 << protocol[i] << std::endl;
				}

				startChrono = std::chrono::system_clock::now();
				//we send the information of the Left side
				iResult = send(ConnectSocket, (const char *)bufferL, bytesToSend, 0);
				if (iResult == SOCKET_ERROR) {
					printf("send failed with error: %d\n", WSAGetLastError());
					closesocket(ConnectSocket);
					WSACleanup();
					break;
					//return 1;
				}
				endChrono = std::chrono::system_clock::now();
				elapsed_seconds = endChrono - startChrono;
				if (i < 10000) {
					sendData[i] = (double)elapsed_seconds.count() * 1000; //ms
					fs2 << sendData[i] << std::endl;
				}

				//now we do the same but with the Rigth image
				infoR = zed->retrieveImage(sl::zed::SIDE::RIGHT).data;
				if (encode) {
					memcpy(image.data, infoR, zedWidth*zedHeight * 4 * sizeof(uchar));
					cv::imencode(".jpg", image, buffR);
					capacityRigth = buffR.capacity();
					sizeBUFFERR = buffR.size();
					size = sizeBUFFERR;
				}
				else {
					size = sizeR;
				}
			
				//Again the program attaches the labels of the protocol to the information
				if (encode) {
					bufferR = (unsigned char *)realloc(bufferR, (size + 12));
					bufferR[0] = 'a';
					bufferR[1] = 'a';
					bufferR[2] = 'R';
					bufferR[3] = 'Y';
					memcpy(&bufferR[4], &size, sizeof(unsigned int));
					memcpy(&bufferR[8], &capacityRigth, sizeof(unsigned int));
					memcpy(&bufferR[12], buffR.data(), sizeBUFFERR);
					bytesToSend = size + 12;
				}
				else {
					bufferR = (unsigned char *)realloc(bufferR, (size +8));
					bufferR[0] = 'a';
					bufferR[1] = 'a';
					bufferR[2] = 'R';
					bufferR[3] = 'N';
					memcpy(&bufferR[4], &size, sizeof(unsigned int));
					memcpy(&bufferR[8], infoR, size);
					bytesToSend = size + 8;
				}

				//we send the information of the Left side
				iResult = send(ConnectSocket, (const char *)bufferR, bytesToSend, 0);
 				if (iResult == SOCKET_ERROR) {
					printf("send failed with error: %d\n", WSAGetLastError());
					closesocket(ConnectSocket);
					WSACleanup();
					break;
					//return 1;
				}


				i++;
			}

		}

		// shutdown the connection since no more data will be sent
		iResult = shutdown(ConnectSocket, SD_SEND);
		if (iResult == SOCKET_ERROR) {
			printf("shutdown failed with error: %d\n", WSAGetLastError());
			//return 1;
		}

		closesocket(ConnectSocket);
		WSACleanup();
		delete zed;
	}
    return 0;
}

