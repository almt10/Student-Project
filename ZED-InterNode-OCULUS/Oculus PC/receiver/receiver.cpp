// receiver.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <string.h>
#include <assert.h>

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <SDL.h>
#include <SDL_syswm.h>
#include <iostream>
#include <conio.h>
#include <fstream>


#include <GL/glew.h>

#include <stddef.h>

#include <SDL.h>
#include <SDL_syswm.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>
#include <sstream>
#include "Shader.hpp"
#include <list>
#include <chrono>

GLchar* OVR_ZED_VS =
"#version 330 core\n \
			layout(location=0) in vec3 in_vertex;\n \
			layout(location=1) in vec2 in_texCoord;\n \
			uniform float hit; \n \
			uniform uint isLeft; \n \
			out vec2 b_coordTexture; \n \
			void main()\n \
			{\n \
				if (isLeft == 1U)\n \
				{\n \
					b_coordTexture = in_texCoord;\n \
					gl_Position = vec4(in_vertex.x - hit, in_vertex.y, in_vertex.z,1);\n \
				}\n \
				else \n \
				{\n \
					b_coordTexture = vec2(1.0 - in_texCoord.x, in_texCoord.y);\n \
					gl_Position = vec4(-in_vertex.x + hit, in_vertex.y, in_vertex.z,1);\n \
				}\n \
			}";

GLchar* OVR_ZED_FS =
"#version 330 core\n \
			uniform sampler2D u_texturePG; \n \
			in vec2 b_coordTexture;\n \
			out vec4 out_color; \n \
			void main()\n \
			{\n \
				out_color = vec4(texture(u_texturePG, b_coordTexture).rgb,1); \n \
			}";

#include <sstream>
#undef UNICODE

#define WIN32_LEAN_AND_MEAN
#define MAX_FPS 100
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
//#pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 4000000
#define DEFAULT_PORT "30000"

using namespace std;

void resetAll(ovrSession session) {
	ovr_Destroy(session);
	ovr_Shutdown();
}


int main(int argc, char *argv[])
{
	// Initialize a boolean that will be used to stop the applications loop
	bool end = false;
	while (!end) {
	whilestart:
		//Initialize the socket information
		WSADATA wsaData;
		int iResult;
		SOCKET ListenSocket = INVALID_SOCKET;
		SOCKET ClientSocket = INVALID_SOCKET;  

		struct addrinfo *addr = NULL;
		struct addrinfo hints;

		//char recvbuf[DEFAULT_BUFLEN];
		char *recvbuf = NULL;
		char *sendbuf = NULL;
		recvbuf = (char *)malloc(4000000);
		sendbuf = (char *)malloc(10);
		int recvbuflen = DEFAULT_BUFLEN;

		// Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed with error: %d\n", iResult);
			continue;
			//return 1;
		}
		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;
		hints.ai_flags = AI_PASSIVE;

		// Resolve the server address and port for the data socket 
		iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &addr);
		if (iResult != 0) {
			printf("getaddrinfo failed with error: %d\n", iResult);
			WSACleanup();
			continue;
			//return 1;
		}

		// Create a SOCKET for connecting to the data 
		ListenSocket = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
		if (ListenSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			freeaddrinfo(addr);
			WSACleanup();
			continue;
			//return 1;
		}
		// Setup the TCP listening socket
		iResult = bind(ListenSocket, addr->ai_addr, (int)addr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			printf("bind failed with error: %d\n", WSAGetLastError());
			freeaddrinfo(addr);
			closesocket(ListenSocket);
			WSACleanup();
			continue;
			//return 1;
		}
		freeaddrinfo(addr);

		iResult = listen(ListenSocket, SOMAXCONN);
		if (iResult == SOCKET_ERROR) {
			printf("listen failed with error: %d\n", WSAGetLastError());
			closesocket(ListenSocket);
			WSACleanup();
			continue;
			//return 1;
		}

		// Accept a client socket
		ClientSocket = accept(ListenSocket, NULL, NULL);
		if (ClientSocket == INVALID_SOCKET) {
			printf("accept failed with error: %d\n", WSAGetLastError());
			closesocket(ListenSocket);
			WSACleanup();
			continue;
			//return 1;
		}
		// No longer need server socket
		closesocket(ListenSocket);

		// Initialize SDL2's context 
		//Use this function to initialize the SDL library. This must be called before using any other SDL function.
		SDL_Init(SDL_INIT_VIDEO);
		// Initialize Oculus' context
		ovrResult result = ovr_Initialize(nullptr);
		if (OVR_FAILURE(result)) {
			std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
			SDL_Quit();
			continue;
			//return -1;
		}

		ovrSession session;
		ovrGraphicsLuid luid;
		// Connect to the Oculus headset
		result = ovr_Create(&session, &luid);
		if (OVR_FAILURE(result)) {
			std::cout << "ERROR: Oculus Rift not detected" << std::endl;
			ovr_Shutdown();
			SDL_Quit();
			continue;
			//return -1;
		}

		int display_max = SDL_GetNumVideoDisplays();
		printf("Screens detected: %i", display_max);
		//SDL_Delay(5000);
		//we stablish the width and height of the window
		int winWidth = 1280;
		int winHeight = 720;
		//Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_FULLSCREEN;
		Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
		// Create SDL2 Window
		SDL_Window* window = SDL_CreateWindow("Oculus Mirror", SDL_WINDOWPOS_CENTERED_DISPLAY(display_max - 1), SDL_WINDOWPOS_CENTERED_DISPLAY(display_max - 1), winWidth, winHeight, flags);
		// Create OpenGL context
		SDL_GLContext glContext = SDL_GL_CreateContext(window);
		// Initialize GLEW
		glewInit();
		// Turn off vsync to let the compositor do its magic
		SDL_GL_SetSwapInterval(0);

		//we get some initial parameters from the cameras through the socket
		int zedWidth = 0;
		int zedHeight = 0;
		int resizeWidth = 0;
		int resizeHeight = 0;
		int resize_factor = 8;
		int focal = 0;
		char label2;
		iResult = recv(ClientSocket, recvbuf, 21, 0);
		if (iResult > 0) {
			label2 = recvbuf[0];
			zedWidth = *(int *)&recvbuf[1];
			zedHeight = *(int *)&recvbuf[5];
			focal = *(int *)&recvbuf[9];
			resizeHeight = *(int *)&recvbuf[13];
			resizeWidth = *(int *)&recvbuf[17];
		}
		else {
			printf("recv failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			continue;
			//return 1;
		}

		//we continue creating glew textures:
		GLuint zedTextureID_L, zedTextureID_R;
		// Generate OpenGL texture for left images of the ZED camera
		glGenTextures(1, &zedTextureID_L); //generate texture names
		glBindTexture(GL_TEXTURE_2D, zedTextureID_L); //bind a named texture to a texturing target
													  //specify a two-dimensional texture image, NULL for the firts image as we havent retrieve the zed camera image yet
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// Generate OpenGL texture for right images of the ZED camera
		glGenTextures(1, &zedTextureID_R);
		glBindTexture(GL_TEXTURE_2D, zedTextureID_R);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glBindTexture(GL_TEXTURE_2D, 0);

#if OPENGL_GPU_INTEROP
		cudaGraphicsResource* cimg_L;
		cudaGraphicsResource* cimg_R;
		cudaError_t errL, errR;
		errL = cudaGraphicsGLRegisterImage(&cimg_L, zedTextureID_L, GL_TEXTURE_2D, cudaGraphicsMapFlagsNone);
		errR = cudaGraphicsGLRegisterImage(&cimg_R, zedTextureID_R, GL_TEXTURE_2D, cudaGraphicsMapFlagsNone);
		if (errL != cudaSuccess || errR != cudaSuccess) {
			std::cout << "ERROR: cannot create CUDA texture : " << errL << "|" << errR << std::endl;
		}
#endif

		ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
		// Get the texture sizes of Oculus eyes
		ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], 1.0f);
		ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], 1.0f);
		// Compute the final size of the render buffer
		ovrSizei bufferSize;
		bufferSize.w = textureSize0.w + textureSize1.w;
		bufferSize.h = max(textureSize0.h, textureSize1.h);
		// Initialize OpenGL swap textures to render
		//A swap chain is a collection of buffers that are used for displaying frames to the user.
		ovrTextureSwapChain textureChain = nullptr;
		// Description of the swap chain
		ovrTextureSwapChainDesc descTextureSwap = {};
		descTextureSwap.Type = ovrTexture_2D;
		descTextureSwap.ArraySize = 1;
		descTextureSwap.Width = bufferSize.w;
		descTextureSwap.Height = bufferSize.h;
		descTextureSwap.MipLevels = 1;
		descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
		descTextureSwap.SampleCount = 1;
		descTextureSwap.StaticImage = ovrFalse;
		// Create the OpenGL texture swap chain
		result = ovr_CreateTextureSwapChainGL(session, &descTextureSwap, &textureChain);

		int length = 0;
		ovr_GetTextureSwapChainLength(session, textureChain, &length);
		if (OVR_SUCCESS(result)) {
			for (int i = 0; i < length; ++i) {
				GLuint chainTexId;
				//we get a specific buffer within the chain as a GL texture name
				ovr_GetTextureSwapChainBufferGL(session, textureChain, i, &chainTexId);
				glBindTexture(GL_TEXTURE_2D, chainTexId);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			}
		}
		else {
			std::cout << "ERROR: failed creating swap texture" << std::endl;
			ovr_Destroy(session);
			ovr_Shutdown();
			SDL_GL_DeleteContext(glContext);
			SDL_DestroyWindow(window);
			SDL_Quit();
			continue;
			//return -1;
		}

		// Generate frame buffer to render
		GLuint fboID;
		//glGenFramebuffers returns n framebuffer object names in ids.
		glGenFramebuffers(1, &fboID);
		// Generate depth buffer of the frame buffer
		GLuint depthBuffID;
		glGenTextures(1, &depthBuffID);
		glBindTexture(GL_TEXTURE_2D, depthBuffID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		GLenum internalFormat = GL_DEPTH_COMPONENT24;
		GLenum type = GL_UNSIGNED_INT;
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type, NULL);

		GLuint fboID2;
		//glGenFramebuffers returns n framebuffer object names in ids.
		glGenFramebuffers(1, &fboID2);
		// Generate depth buffer of the frame buffer
		GLuint depthBuffID2;
		glGenTextures(1, &depthBuffID2);
		glBindTexture(GL_TEXTURE_2D, depthBuffID2);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		GLenum internalFormat2 = GL_DEPTH_COMPONENT24;
		GLenum type2 = GL_UNSIGNED_INT;
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat2, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type2, NULL);

		// Create a mirror texture to display the render result in the SDL2 window
		ovrMirrorTextureDesc descMirrorTexture;
		//with memset we set the first num bytes of the block of memory pointed by ptr to the specified value
		memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
		descMirrorTexture.Width = winWidth; //size of the window already created
		descMirrorTexture.Height = winHeight;
		descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

		ovrMirrorTexture mirrorTexture = nullptr;
		//Creates a Mirror Texture which is auto-refreshed to mirror Rift contents produced by this application.
		result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
		if (!OVR_SUCCESS(result)) {
			std::cout << "ERROR: Failed to create mirror texture" << std::endl;
		}
		GLuint mirrorTextureId;
		ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

		GLuint mirrorFBOID;
		glGenFramebuffers(1, &mirrorFBOID);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
		//associate 
		glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
		glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		// Frame index used by the compositor
		// it needs to be updated each new frame
		long long frameIndex = 0;

		// FloorLevel will give tracking poses where the floor height is 0
		ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);

		// Initialize a default Pose
		ovrPosef eyeRenderPose[2];

		// Get the render description of the left and right "eyes" of the Oculus headset
		ovrEyeRenderDesc eyeRenderDesc[2];
		eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
		eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
		// Get the Oculus view scale description
		ovrVector3f hmdToEyeOffset[2];
		double sensorSampleTime;

		// Create and compile the shader's sources
		Shader shader(OVR_ZED_VS, OVR_ZED_FS);

		// Compute the ZED image field of view with the ZED parameters
		float zedFovH = atanf(zedWidth / (focal * 2.5f)) * 2.f;
		// Compute the Horizontal Oculus' field of view with its parameters
		float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
		// Compute the useful part of the ZED image
		unsigned int usefulWidth = zedWidth * ovrFovH / zedFovH;
		// Compute the size of the final image displayed in the headset with the ZED image's aspect-ratio kept
		float zoom = 1.0f;
		if (zedWidth == 1920) {
			zoom = 0.85f;
		}
		unsigned int widthFinal = bufferSize.w / 2;
		float heightGL = 1.f;
		float widthGL = 1.f;
		if (usefulWidth > 0.f) {
			unsigned int heightFinal = zedHeight * widthFinal / usefulWidth;
			// Convert this size to OpenGL viewport's frame's coordinates
			heightGL = (heightFinal) / (float)(bufferSize.h)* zoom;
			widthGL = ((zedWidth * (heightFinal / (float)zedHeight)) / (float)widthFinal)* zoom;
		}
		else {
			std::cout << "WARNING: ZED parameters got wrong values."
				"Default vertical and horizontal FOV are used.\n"
				"Check your calibration file or check if your ZED is not too close to a surface or an object."
				<< std::endl;
		}

		// Compute the Vertical Oculus' field of view with its parameters
		float ovrFovV = (atanf(hmdDesc.DefaultEyeFov[0].UpTan) + atanf(hmdDesc.DefaultEyeFov[0].DownTan));

		// Compute the center of the optical lenses of the headset
		float offsetLensCenterX = ((atanf(hmdDesc.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
		float offsetLensCenterY = ((atanf(hmdDesc.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;


		// Create a rectangle with the computed coordinates and push it in GPU memory.

		struct GLScreenCoordinates {
			float left, up, right, down;
		} screenCoord;
		screenCoord.up = heightGL + offsetLensCenterY;
		screenCoord.down = heightGL - offsetLensCenterY;
		screenCoord.right = widthGL + offsetLensCenterX;
		screenCoord.left = widthGL - offsetLensCenterX;

		float rectVertices[12] = { -screenCoord.left, -screenCoord.up, 0,
			screenCoord.right, -screenCoord.up, 0,
			screenCoord.right, screenCoord.down, 0,
			-screenCoord.left, screenCoord.down, 0 };
		GLuint rectVBO[3];
		glGenBuffers(1, &rectVBO[0]);
		glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

		float rectTexCoord[8] = { 0, 1, 1, 1, 1, 0, 0, 0 };
		glGenBuffers(1, &rectVBO[1]);
		glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

		unsigned int rectIndices[6] = { 0, 1, 2, 0, 2, 3 };
		glGenBuffers(1, &rectVBO[2]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		// Initialize hit value
		float hit = 0.02f;
		// Initialize a boolean that will be used to pause/unpause rendering
		bool refresh = true;
		// SDL variable that will be used to store input events
		SDL_Event events;
		// Initialize time variables. They will be used to limit the number of frames rendered per second.
		// Frame counter
		unsigned int riftc = 0, zedc = 1;
		// Chronometer
		unsigned int rifttime = 0, zedtime = 0, zedFPS = 0;
		int time1 = 0, timePerFrame = 0;
		int frameRate = (int)(1000 / MAX_FPS);

		// This boolean is used to test if the application is focused
		bool isVisible = true;

		// Enable the shader
		glUseProgram(shader.getProgramId());
		// Bind the Vertex Buffer Objects of the rectangle that displays ZED images
		// vertices
		glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
		glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
		glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
		// indices
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
		// texture coordinates
		glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
		glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
		glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);

		char label[2];
		char side;
		char *bufferI = NULL;
		bufferI = (char*)malloc(sizeof(char) * 10000000);
		char *bufferL = NULL;
		bufferL = (char*)malloc(sizeof(char) * 4000000);
		char *bufferD = NULL;
		bufferD = (char*)malloc(sizeof(char) * 10000000);
		char *bufferR = NULL;
		bufferR = (char*)malloc(sizeof(char) * 4000000);
		int c = 0;
		int bytesReaded = 0;
		int iSendResult = 0;
		int length_toread = 0;
		int size = 0;
		int numIteraciones = 0;
		bool firstPacket = false;
		bool moreData = false;
		int dataReaded = DEFAULT_BUFLEN;

		cv::Mat image0 = cv::Mat(resizeHeight, resizeWidth, CV_8UC4, 1);
		cv::Mat image1 = cv::Mat(zedHeight, zedWidth, CV_8UC4, 1);
		cv::Mat image2 = cv::Mat(zedHeight, zedWidth, CV_8UC4, 1);
		cv::Mat imageRight=cv::Mat(zedHeight, zedWidth, CV_8UC4, 1);
		cv::Mat imageLeft=cv::Mat(zedHeight, zedWidth, CV_8UC4, 1);

		//variables to measure the time we use to pass the images to oculus
		std::chrono::time_point<std::chrono::system_clock> startChrono, endChrono, startTest, endTest;
		startChrono = std::chrono::system_clock::now();
		endChrono = std::chrono::system_clock::now();
		startTest = std::chrono::system_clock::now();
		endTest = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds;
		std::chrono::duration<double> elapsed_seconds_test;
		double read[10000];
		double display[10000];
		double distribution[65][1000];
		double decoding[10000];
		int l = 0;
		int old_category = 8;
		int n = 0;
		int i = 0;
		std::ofstream fs("readLatency.txt");
		std::ofstream fs2("displayLatency.txt");
		std::ofstream fs3("distribution.txt");
		std::ofstream fs4("decoding.txt");


		//variables used in case of decoding the images
		std::vector<uchar> buff;
		std::vector<uchar> buff2;
		int capacityL = 0;
		int capacityR = 0;
		int sizeL = 0;
		int sizeR = 0;
		char resize;
		char encode = 'N';
		int capacityInt = 0;
		bool labels = false;
		// Main loop
		while (true) {
			// Compute the time used to render the previous frame
			 //SDL_GetTicks() Returns an unsigned 32-bit value representing the number of milliseconds since the SDL library initialized.
			timePerFrame = SDL_GetTicks() - time1;
			// If the previous frame has been rendered too fast
			if (timePerFrame < frameRate) {
				// Pause the loop to have a max FPS equal to MAX_FPS
				//SDL_Delay wait a specified number of milliseconds before returning.
				SDL_Delay(frameRate - timePerFrame);
				timePerFrame = frameRate;
			}
			// Increment the ZED chronometer
			zedtime += timePerFrame;
			// If ZED chronometer reached 1 second
			if (zedtime > 1000) {
				zedFPS = zedc;
				zedc = 0;
				zedtime = 0;
				if (n < 1000) {
					for (int h = 0; h < l; h++) {
						fs3 << distribution[h][n] << "  ";
					}
					fs3 << endl;
					n++;
					l = 0;
				}
			}
			// Increment the Rift chronometer and the Rift frame counter
			rifttime += timePerFrame;
			riftc++;
			// If Rift chronometer reached 200 milliseconds
			if (rifttime > 200) {
				// Display FPS
				std::cout << "\rRIFT FPS: " << 1000 / (rifttime / riftc) << " | ZED FPS: " << zedFPS;
				// Reset Rift chronometer
				rifttime = 0;
				// Reset Rift frame counter
				riftc = 0;
			}
			// Start frame chronometer
			time1 = SDL_GetTicks();

			// While there is an event catched and not tested
			while (SDL_PollEvent(&events)) {
				// If a key is released
				if (events.type == SDL_KEYUP) {
					// If Q quit the application
					if (events.key.keysym.scancode == SDL_SCANCODE_Q)
						end = true;
					// If R reset the hit value
					else if (events.key.keysym.scancode == SDL_SCANCODE_R)
						hit = 0.02f - hit;
					// If C pause/unpause rendering
					else if (events.key.keysym.scancode == SDL_SCANCODE_C)
						refresh = !refresh;
				}
				// If the mouse wheel is used
				if (events.type == SDL_MOUSEWHEEL)
				{
					// Increase or decrease hit value
					float s;
					events.wheel.y > 0 ? s = 1.0f : s = -1.0f;
					hit += 0.005f * s;
				}
			}

			// Get texture swap index where we must draw our frame
			GLuint curTexId;
			int curIndex;
			ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
			ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

			// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
			//Computes the distortion viewport, view adjust, and other rendering parameters for the specified eye.
			eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
			eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
			hmdToEyeOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
			hmdToEyeOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;
			// Get eye poses, feeding in correct IPD offset
			ovr_GetEyePoses(session, frameIndex, ovrTrue, hmdToEyeOffset, eyeRenderPose, &sensorSampleTime);

			bool leftEye = true;
			bool both = false;

			if (true) {
				// If the application is focused
				//if (isVisible) {
				//we need to receive the data from both cameras		 
				startChrono = std::chrono::system_clock::now();
				if (l == 0) {
					startTest = std::chrono::system_clock::now();
					endTest = startTest;
				}
			init:
				while (!both) {
					iResult = recv(ClientSocket, recvbuf, 2, 0);
					startChrono = std::chrono::system_clock::now();
					if (iResult>0) {
						label[0] = recvbuf[0];
						label[1] = recvbuf[1];						
						if ((label[0] != 'a') || (label[1] != 'a')) {
							bool next = false;
							sendbuf[0] = 'c';
							sendbuf[1] = 'c';
							memcpy(&sendbuf[2], &size, sizeof(unsigned int));
							memcpy(&sendbuf[6], &zedc, sizeof(unsigned int));
							iSendResult = send(ClientSocket, sendbuf, 10, 0);
							if (iSendResult == SOCKET_ERROR) {
								printf("send failed with error: %d\n", WSAGetLastError());
								closesocket(ClientSocket);
								WSACleanup();
								return 1;
							}
							while (!next) {
								iResult = recv(ClientSocket, recvbuf, 1, 0);
								if (iResult > 0) {
									label[0] = recvbuf[0];
									if (label[0] == 'a') {
										iResult = recv(ClientSocket, recvbuf, 1, 0);
										if (iResult > 0) {
											label[1] = recvbuf[0];
											if (label[1] == 'a') {
												iResult = recv(ClientSocket, recvbuf, 1, 0);
												if (iResult > 0) {
													side = recvbuf[0];
													if (side == 'L') 
													{
														next = true;
														labels = true;
													}
												}
											}
										}
									}
								}
							}
						}

						//now we read the the side of the image
						if (labels) {
							iResult = recv(ClientSocket, recvbuf, 5, 0);
						}
						else {
							iResult = recv(ClientSocket, recvbuf, 6, 0);
						}
						if (iResult>0) {
							if (labels) {
								encode = recvbuf[0];
								resize_factor = *(int *)&recvbuf[1];
								labels = false;
							}
							else {
								side = recvbuf[0];
								encode = recvbuf[1];
								resize_factor = *(int *)&recvbuf[2];
							}
							if ((encode != 'Y') && (encode != 'N') && (encode != 'R')) {
								goto init;
							}
							if ((resize_factor <= 0) || (resize_factor > 8)) {
								goto init;
							}
							if ((side != 'L') && (side != 'R')) {
								goto init;
							}
							/*char test = recvbuf[2];
							char test2 = recvbuf[3];
							char test3 = recvbuf[4];
							char test4 = recvbuf[5];*/
							if ((resize_factor <= 8 && resize_factor>0) && (resize_factor!=old_category)) {
								if (side == 'L') {
									if (encode == 'R') {
										imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC3, 1);
									}
									else if (encode == 'Y') {
										imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC3, 1);
									}
									else {
										imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									}
								}
								else if (side == 'R') {
									if (encode == 'R') {
										imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC3, 1);
									}
									else if (encode == 'Y') {
										imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC3, 1);
									}
									else {
										imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									}
								}
								else printf(" ");
							}
							if (side == 'R') {
								old_category = resize_factor;
							}
							//encode = recvbuf[2];
							assert((side == 'L') || (side == 'R'));

							//now we get the size of the image
							iResult = recv(ClientSocket, recvbuf, 4, 0);
							if (iResult>0) {
								size = *(int *)recvbuf;
								length_toread = size;
								if ((size <0) || (size>15000000)) {
									goto init;
								}
								/*if (resize == 'a') {
									resize_factor = 1;
									image0 = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);*/
									if (encode == 'Y') {
										iResult = recv(ClientSocket, recvbuf, 4, 0);
										capacityInt = *(int *)recvbuf;
										if ((capacityInt <0) || ( capacityInt>1500000)) {
											goto init;
										}
										if (side == 'L') {
											capacityL = capacityInt;
											sizeL = size;
										}
										else {
											capacityR = capacityInt;
											sizeR = size;
											if (sizeR > 3 * sizeL) {
												goto init;
											}
										}
									}
								
							/*
								else if (resize == 'b') {
									resize_factor = 1;
									image0 = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									if (encode == 'Y') {
										iResult = recv(ClientSocket, recvbuf, 4, 0);
										capacityInt = *(int *)recvbuf;
										if (side == 'L') {
											capacityL = capacityInt;
											sizeL = size;
										}
										else {
											capacityR = capacityInt;
											sizeR = size;
										}
									}
								}
								else if (resize == 'c') {
									resize_factor = 2;
									image0 = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									if (encode == 'Y') {
										iResult = recv(ClientSocket, recvbuf, 4, 0);
										capacityInt = *(int *)recvbuf;
										if (side == 'L') {
											capacityL = capacityInt;
											sizeL = size;
										}
										else {
											capacityR = capacityInt;
											sizeR = size;
										}
									}
								}
								else if (resize == 'd') {
									resize_factor = 4;
									image0 = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									if (encode == 'Y') {
										iResult = recv(ClientSocket, recvbuf, 4, 0);
										capacityInt = *(int *)recvbuf;
										if (side == 'L') {
											capacityL = capacityInt;
											sizeL = size;
										}
										else {
											capacityR = capacityInt;
											sizeR = size;
										}
									}
								}
								else if (resize == 'e') {
									resize_factor = 8;
									/*
									image0 = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
									iResult = recv(ClientSocket, recvbuf, 4, 0);
									for (int i = 0; i < 4; i++) {
									capacity[i] = recvbuf[i];
									}
									capacityInt = buffToInteger(capacity);
									if (side == 'L') {
									capacityL = capacityInt;
									sizeL = size;
									}
									else {
									capacityR = capacityInt;
									sizeR = size;
									}
									*/
								
								if (side == 'L') {
									bufferI = (char *)realloc(bufferI, size);
								}
								if (side == 'R') {
									bufferD = (char *)realloc(bufferD, size);
								}
								int c = 0;
								//finally we read the information of the image
								while (length_toread > 0) {
									iResult = recv(ClientSocket, recvbuf, min(4000000, length_toread), 0);
									if (iResult > 0) {
										if (side == 'L') {
											memcpy(&bufferI[c], recvbuf, iResult);
											length_toread -= iResult;
											c += iResult;
											
										}
										else if (side == 'R') {
											memcpy(&bufferD[c], recvbuf, iResult);
											c += iResult;
											length_toread -= iResult;
											if (c == size) {
												both = true;
												endChrono= std::chrono::system_clock::now();
												endTest = endChrono;
												elapsed_seconds = endChrono - startChrono;
												//printf("%lf\n", elapsed_seconds.count() * 1000);
												elapsed_seconds_test = endTest - startTest;
												if (i < 10000) {
													read[i] = (double)elapsed_seconds.count() * 1000;
													fs << read[i] << endl;
												}
												if (n < 1000) {
													distribution[l][n] = (double)elapsed_seconds_test.count() * 1000;
													l++;
												}
												zedc++;
												//we create the feedback we want to send to the intermidiate node
												
												sendbuf[0] = 'c';
												sendbuf[1] = 'c';
												memcpy(&sendbuf[2], &size, sizeof(unsigned int));
												memcpy(&sendbuf[6], &zedc, sizeof(unsigned int));
												iSendResult = send(ClientSocket, sendbuf, 10, 0);
												if (iSendResult == SOCKET_ERROR) {
													printf("send failed with error: %d\n", WSAGetLastError());
													closesocket(ClientSocket);
													WSACleanup();
													return 1;
												}
												
											}
										}
										else {
											printf("assert failed");
										}

									}
									else {
										printf("recv failed with error: %d\n", WSAGetLastError());
										closesocket(ClientSocket);
										WSACleanup();
										resetAll(session);
										SDL_DestroyWindow(window);
										goto whilestart;
										//return 1;
									}
								}

							}
							else {
								printf("recv failed with error: %d\n", WSAGetLastError());
								closesocket(ClientSocket);
								WSACleanup();
								resetAll(session);
								SDL_DestroyWindow(window);
								goto whilestart;
								//return 1;
							}
						}
						else {
							printf("recv failed with error: %d\n", WSAGetLastError());
							closesocket(ClientSocket);
							WSACleanup();
							resetAll(session);
							SDL_DestroyWindow(window);
							goto whilestart;
							//return 1;
						}

					}
					else {
						printf("recv failed with error: %d\n", WSAGetLastError());
						closesocket(ClientSocket);
						WSACleanup();
						resetAll(session);
						SDL_DestroyWindow(window);
						goto whilestart;
						//return 1;
					}
				}
				startChrono = std::chrono::system_clock::now();
				/*memcpy(image1.data, bufferI, size);
				cv::imwrite("zedLeft.jpg", image1);
				memcpy(image2.data, bufferD, size);
				cv::imwrite("zedRigth.jpg", image2);*/
				if (encode == 'Y') {
					buff.reserve(capacityL);
					buff.resize(sizeL);
					memcpy(buff.data(), bufferI, sizeL);
					startChrono = std::chrono::system_clock::now();
					imageLeft = cv::imdecode(buff, CV_LOAD_IMAGE_COLOR);
					endChrono = std::chrono::system_clock::now();
					buff.reserve(capacityR);					
					buff.resize(sizeR);

					memcpy(buff.data(), bufferD, sizeR);					
					imageRight = cv::imdecode(buff, CV_LOAD_IMAGE_COLOR);

					
					buff.clear();
				
					cv::Size dimensionsL = imageLeft.size();
					cv::Size dimensionsR = imageRight.size();
					//cv::imwrite("imageDecoded.jpg", imageLeft);
					int sizeLeft= dimensionsL.height * dimensionsL.width * 3;
					int sizeRigth = dimensionsR.height * dimensionsR.width * 3;
					if ((sizeLeft == (resize_factor *resize_factor * resizeWidth * resizeHeight * 3))) {
						if ((resize_factor != 8)) {
							cv::resize(imageLeft, image1, cv::Size(zedWidth, zedHeight));
							//cv::imwrite("zedResized.jpg", image1);
							bufferI = (char *)realloc(bufferI, image1.cols * image1.rows * 3);
							memcpy(bufferI, image1.data, image1.cols * image1.rows * 3);
						}
						else {
							bufferI = (char *)realloc(bufferI, sizeLeft);
							memcpy(bufferI, imageLeft.data, sizeLeft);
						}
					}
					
					if(sizeRigth == (resize_factor *resize_factor * resizeWidth * resizeHeight * 3)){
						if ((resize_factor != 8)) {
							cv::resize(imageRight, image1, cv::Size(zedWidth, zedHeight));
							//cv::imwrite("zedResizedRigth.jpg", image1);
							bufferD = (char *)realloc(bufferD, image1.cols * image1.rows * 3);
							memcpy(bufferD, image1.data, image1.cols * image1.rows * 3);
						}
						else {
							bufferD = (char *)realloc(bufferD, sizeRigth);
							memcpy(bufferD, imageRight.data, sizeRigth);
						}
					}
				}
				else if (encode == 'R') {
					if ((size == (resize_factor *resize_factor * resizeWidth * resizeHeight * 3)) && (resize_factor != 8)) {
						
						memcpy(imageLeft.data, bufferI, resize_factor *resize_factor * resizeHeight * resizeWidth * 3);
						//cv::imwrite("zedReceived.jpg", image0);
						startChrono = std::chrono::system_clock::now();
						cv::resize(imageLeft, image1, cv::Size(zedWidth, zedHeight));
						endChrono = std::chrono::system_clock::now();
						elapsed_seconds = endChrono - startChrono;
						if (i < 10000) {
							decoding[i] = (double)elapsed_seconds.count() * 1000;
							fs4 << decoding[i] << endl;
						}
						//cv::imwrite("zedResized.jpg", image1);
						bufferI = (char *)realloc(bufferI, zedHeight *zedWidth * 3);
						memcpy(bufferI, image1.data, zedHeight*zedWidth * 3);

						memcpy(imageRight.data, bufferD, resize_factor * resize_factor *  resizeHeight * resizeWidth * 3);
						//cv::imwrite("zedReceivedRigth.jpg", image0);
						cv::resize(imageRight, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResizedRigth.jpg", image1);
						bufferD = (char *)realloc(bufferD, zedHeight*zedWidth * 3);
						memcpy(bufferD, image1.data, zedHeight*zedWidth * 3);
					}
				}
				else{
					if ((size == (resize_factor *resize_factor * resizeWidth * resizeHeight * 4)) && (resize_factor != 8)) {
						
						memcpy(imageLeft.data, bufferI, resize_factor *resize_factor * resizeHeight * resizeWidth * 4);
						startChrono = std::chrono::system_clock::now();
						//cv::imwrite("zedReceived.jpg", image0);
						cv::resize(imageLeft, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResized.jpg", image1);
						endChrono = std::chrono::system_clock::now();
						elapsed_seconds = endChrono - startChrono;
						if (i < 10000) {
							decoding[i] = (double)elapsed_seconds.count() * 1000;
							fs4 << decoding[i] << endl;
						}
						bufferI = (char *)realloc(bufferI, zedHeight *zedWidth * 4);
						memcpy(bufferI, image1.data, zedHeight*zedWidth * 4);

						memcpy(imageRight.data, bufferD, resize_factor * resize_factor *  resizeHeight * resizeWidth * 4);
						//cv::imwrite("zedReceivedRigth.jpg", image0);
						cv::resize(imageRight, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResizedRigth.jpg", image1);
						bufferD = (char *)realloc(bufferD, zedHeight*zedWidth * 4);
						memcpy(bufferD, image1.data, zedHeight*zedWidth * 4);
					}
				}
				/*if (encode == 'Y') {
					buff.reserve(capacityL);
					buff2.reserve(capacityR);

					buff.resize(sizeL);
					buff2.resize(sizeR);

					memcpy(buff.data(), bufferI, sizeL);
					memcpy(buff2.data(), bufferD, sizeR);
					imageLeft = cv::imdecode(buff, CV_LOAD_IMAGE_COLOR);
					imageRight = cv::imdecode(buff2, CV_LOAD_IMAGE_COLOR);
					//cv::imwrite("zedResizedRigth.jpg", imageRight);
					cv::Size dimensions = imageLeft.size();
					//cv::imwrite("imageDecoded.jpg", imageLeft);
					size = dimensions.height * dimensions.width * 4;
					if (size == (resize_factor *resize_factor * resizeWidth * resizeHeight * 4)) {
						imageLeft = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
						cv::resize(imageLeft, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResized.jpg", image1);
						bufferI = (char *)realloc(bufferI, zedHeight*zedWidth * 3);
						memcpy(bufferI, image1.data, zedHeight*zedWidth * 3);

						imageRight = cv::Mat(resize_factor*resizeHeight, resize_factor*resizeWidth, CV_8UC4, 1);
						cv::resize(imageRight, image2, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResizedRigth.jpg", image2);
						bufferD = (char *)realloc(bufferD, zedHeight*zedWidth * 3);
						memcpy(bufferD, image2.data, zedHeight*zedWidth * 3);
						//std::memmove(bufferD, image2.data, zedHeight*zedWidth * 3);
					}
				}
				else {
					if ((size == (resize_factor *resize_factor * resizeWidth * resizeHeight * 4)) && (resize_factor != 8)) {
						memcpy(imageLeft.data, bufferI, resize_factor *resize_factor * resizeHeight * resizeWidth * 4);
						//cv::imwrite("zedReceived.jpg", image0);
						cv::resize(imageLeft, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResized.jpg", image1);
						bufferI = (char *)realloc(bufferI, zedHeight*zedWidth * 4);
						memcpy(bufferI, image1.data, zedHeight*zedWidth * 4);

						memcpy(imageRight.data, bufferD, resize_factor * resize_factor *  resizeHeight * resizeWidth * 4);
						//cv::imwrite("zedReceivedRigth.jpg", image0);
						cv::resize(imageRight, image1, cv::Size(zedWidth, zedHeight));
						//cv::imwrite("zedResizedRigth.jpg", image1);
						bufferD = (char *)realloc(bufferD, zedHeight*zedWidth * 4);
						memcpy(bufferD, image1.data, zedHeight*zedWidth * 4);
					}
				}*/
				/*memcpy(image1.data, bufferI,zedHeight * zedWidth * 4);
				cv::imwrite("LeftImage.jpg", image1);
				memcpy(image2.data, bufferD, zedHeight * zedWidth * 4);
				cv::imwrite("RigthImage.jpg", image2);*/
				if (refresh) {
					startChrono = std::chrono::system_clock::now();
#if OPENGL_GPU_INTEROP
					sl::zed::Mat m = zed->retrieveImage_gpu(sl::zed::SIDE::LEFT);
					cudaArray_t arrIm;
					cudaGraphicsMapResources(1, &cimg_L, 0);
					cudaGraphicsSubResourceGetMappedArray(&arrIm, cimg_L, 0, 0);
					cudaMemcpy2DToArray(arrIm, 0, 0, m.data, m.step, zedWidth * 4, zedHeight, cudaMemcpyDeviceToDevice);
					cudaGraphicsUnmapResources(1, &cimg_L, 0);

					m = zed->retrieveImage_gpu(sl::zed::SIDE::RIGHT);
					cudaGraphicsMapResources(1, &cimg_R, 0);
					cudaGraphicsSubResourceGetMappedArray(&arrIm, cimg_R, 0, 0);
					cudaMemcpy2DToArray(arrIm, 0, 0, m.data, m.step, zedWidth * 4, zedHeight, cudaMemcpyDeviceToDevice); // *4 = 4 channels * 1 bytes (uint)
					cudaGraphicsUnmapResources(1, &cimg_R, 0);
#endif

					// Bind the frame buffer we already created before
					glBindFramebuffer(GL_FRAMEBUFFER, fboID);
					// Set its color layer 0 as the current swap texture
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
					// Set its depth layer as our depth buffer
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
					// Clear the frame buffer
					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					glClearColor(0, 0, 0, 1);

					// Render for each Oculus eye the equivalent ZED image
					for (int eye = 0; eye < 2; eye++) {
						// Set the left or right vertical half of the buffer as the viewport
						glViewport(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
						// Bind the left or right ZED image
						glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? zedTextureID_L : zedTextureID_R);
#if !OPENGL_GPU_INTEROP
						if (encode == 'N') {
							glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, (eye == ovrEye_Left ? bufferI : bufferD));
						}
						else {
							if ((imageRight.cols * imageRight.rows) == 0) {
								glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, zedWidth, zedHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, (eye == ovrEye_Left ? bufferI : bufferI));
							}
							else {
								glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, zedWidth, zedHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, (eye == ovrEye_Left ? bufferI : bufferD));
							}
							
						}
						/*if (eye == ovrEye_Left)
						{
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, bufferI);
						bufferI = '\0';
						}
						else
						{
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, bufferD);
						bufferD = '\0';
						}*/
#endif
						// Bind the hit value
						//glGetUniformLocation returns the location of the variable called as the second argument
						//and glUniform specify the value of a uniform variable for the current program object
						glUniform1f(glGetUniformLocation(shader.getProgramId(), "hit"), eye == ovrEye_Left ? hit : -hit);
						// Bind the isLeft value
						//GLuint is a Unsigned binary integer
						glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
						// Draw the ZED image
						glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
					}
					// Avoids an error when calling SetAndClearRenderSurface during next iteration.
					// Without this, during the next while loop iteration SetAndClearRenderSurface
					// would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
					// associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
					glBindFramebuffer(GL_FRAMEBUFFER, fboID);
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
					// Commit changes to the textures so they get picked up frame
					ovr_CommitTextureSwapChain(session, textureChain);
				}

				// Do not forget to increment the frameIndex!
				frameIndex++;
			}
			/*
			Note: Even if we don't ask to refresh the framebuffer or if the Camera::grab()
			doesn't catch a new frame, we have to submit an image to the Rift; it
			needs 75Hz refresh. Else there will be jumbs, black frames and/or glitches
			in the headset.
			*/
			ovrLayerEyeFov ld;
			ld.Header.Type = ovrLayerType_EyeFov;
			// Tell to the Oculus compositor that our texture origin is at the bottom left
			ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft; // Because OpenGL | Disable head tracking
																	  // Set the Oculus layer eye field of view for each view
			for (int eye = 0; eye < 2; ++eye) {
				// Set the color texture as the current swap texture
				ld.ColorTexture[eye] = textureChain;
				// Set the viewport as the right or left vertical half part of the color texture
				ld.Viewport[eye] = OVR::Recti(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
				// Set the field of view
				ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
				// Set the pose matrix
				ld.RenderPose[eye] = eyeRenderPose[eye];
			}

			ld.SensorSampleTime = sensorSampleTime;

			ovrLayerHeader* layers = &ld.Header;
			// Submit the frame to the Oculus compositor
			// which will display the frame in the Oculus headset
			result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);

			if (!OVR_SUCCESS(result)) {
				std::cout << "ERROR: failed to submit frame" << std::endl;
				glDeleteBuffers(3, rectVBO);
				ovr_DestroyTextureSwapChain(session, textureChain);
				ovr_DestroyMirrorTexture(session, mirrorTexture);
				ovr_Destroy(session);
				ovr_Shutdown();
				SDL_GL_DeleteContext(glContext);
				SDL_DestroyWindow(window);
				SDL_Quit();
				continue;
				//return -1;
			}

			if (result == ovrSuccess && !isVisible) {
				std::cout << "\nThe application is now shown in the headset." << std::endl;
			}
			isVisible = (result == ovrSuccess);

			// This is not really needed for this application but it may be useful for an more advanced application
			ovrSessionStatus sessionStatus;
			ovr_GetSessionStatus(session, &sessionStatus);
			if (sessionStatus.ShouldRecenter) {
				std::cout << "Recenter Tracking asked by Session" << std::endl;
				ovr_RecenterTrackingOrigin(session);
			}
			// Copy the frame to the mirror buffer
			// which will be drawn in the SDL2 image
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fboID2);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			int GLwinWidth = winWidth;
			int GLwinHeight = winHeight;
			GLint w = GLwinWidth;
			GLint h = GLwinHeight;
			glBlitFramebuffer(0, h, w, 0,
				0, 0, w, h,
				GL_COLOR_BUFFER_BIT, GL_NEAREST);
			glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
			// Swap the SDL2 window
			// Bind the frame buffer we already created before
			glBindFramebuffer(GL_FRAMEBUFFER, fboID2);
			// Set its color layer 0 as the current swap texture
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, zedTextureID_L, 0);
			// Set its depth layer as our depth buffer
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID2, 0);
			// Clear the frame buffer
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glClearColor(0, 0, 0, 1);
			// Set the left or right vertical half of the buffer as the viewport
			glViewport(0, 0, bufferSize.w, bufferSize.h);
			// Bind the left or right ZED image
			glBindTexture(GL_TEXTURE_2D, zedTextureID_L);
#if !OPENGL_GPU_INTEROP
			if (encode == 'N') {
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, bufferD);
			}
			else {
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, zedWidth, zedHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, bufferD);
			}

			/*if (eye == ovrEye_Left)
			{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, bufferI);
			bufferI = '\0';
			}
			else
			{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, bufferD);
			bufferD = '\0';
			}*/
#endif
			// Bind the hit value
			//glGetUniformLocation returns the location of the variable called as the second argument
			//and glUniform specify the value of a uniform variable for the current program object
			//glUniform1f(glGetUniformLocation(shader.getProgramId(), "hit"),hit);
			// Bind the isLeft value
			//GLuint is a Unsigned binary integer
			//glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"),1U);
			// Draw the ZED image
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
			SDL_GL_SwapWindow(window);
			//SDL_RaiseWindow(window);

			endChrono = std::chrono::system_clock::now();
			elapsed_seconds = endChrono - startChrono;
			if (i < 10000) {
				display[i] = (double)elapsed_seconds.count() * 1000;
				fs2 << display[i] << endl;
			}
			i++;

		}

		// Disable all OpenGL buffer
		glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
		glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
		glUseProgram(0);
		glBindVertexArray(0);
		// Delete the Vertex Buffer Objects of the rectangle
		glDeleteBuffers(3, rectVBO);
		// Delete SDL, OpenGL, Oculus and PG context
		ovr_DestroyTextureSwapChain(session, textureChain);
		ovr_DestroyMirrorTexture(session, mirrorTexture);
		ovr_Destroy(session);
		ovr_Shutdown();
		SDL_GL_DeleteContext(glContext);
		SDL_DestroyWindow(window);
		SDL_Quit();

		// shutdown the connection since we're done
		iResult = shutdown(ClientSocket, SD_SEND);
		if (iResult == SOCKET_ERROR) {
			printf("shutdown failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			continue;
			//return 1;
		}

		// cleanup
		closesocket(ClientSocket);
		WSACleanup();
	}
	return 0;
}