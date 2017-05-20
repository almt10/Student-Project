#include "stdafx.h"
#include <iostream>
#include <Windows.h>

#include <GL/glew.h>

#include <stddef.h>

#include <SDL.h>
#include <SDL_syswm.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include "FlyCapture2.h"

#include "Shader.hpp"

#define MAX_FPS 75

GLchar* OVR_PG_VS =
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

GLchar* OVR_PG_FS =
"#version 330 core\n \
			uniform sampler2D u_texturePG; \n \
			in vec2 b_coordTexture;\n \
			out vec4 out_color; \n \
			void main()\n \
			{\n \
				out_color = vec4(texture(u_texturePG, b_coordTexture).rgb,1); \n \
			}";

#include <sstream>
using namespace FlyCapture2;
using namespace std;

void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion(&fc2Version);

	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() << endl;

	ostringstream timeStamp;
	timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
	cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo(CameraInfo* pCamInfo)
{
	cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << pCamInfo->serialNumber << endl;
	cout << "Camera model - " << pCamInfo->modelName << endl;
	cout << "Camera vendor - " << pCamInfo->vendorName << endl;
	cout << "Sensor - " << pCamInfo->sensorInfo << endl;
	cout << "Resolution - " << pCamInfo->sensorResolution << endl;
	cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
	cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;

}

void PrintError(Error error)
{
	error.PrintErrorTrace();
}

int checkError(Error error)
{
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}
	return 0;
}

int initializePGCameras(Camera* cam1, Camera* cam2)
{
	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid1, guid2;
	CameraInfo camInfo1, camInfo2;
	FC2Config config;
	if (checkError(busMgr.GetNumOfCameras(&numCameras))) return -1;
	cout << "Number of cameras detected: " << numCameras << endl;
	if (numCameras != 2) return -1;
	if (checkError(busMgr.GetCameraFromIndex(0, &guid1))) return -1;
	if (checkError(busMgr.GetCameraFromIndex(1, &guid2))) return -1;
	// Connect to a camera
	if (checkError(cam1->Connect(&guid1)) || checkError(cam2->Connect(&guid2))) return -1;
	// Get the camera information and print it
	if (checkError(cam1->GetCameraInfo(&camInfo1)) || checkError(cam2->GetCameraInfo(&camInfo2))) return -1;
	PrintCameraInfo(&camInfo1);
	PrintCameraInfo(&camInfo2);
	// Get the camera configuration
	if (checkError(cam1->GetConfiguration(&config)) || checkError(cam2->GetConfiguration(&config))) return -1;
	// Set the number of driver buffers used to 10.
	config.numBuffers = 10;
	// Set the camera configuration
	if (checkError(cam1->SetConfiguration(&config)) || checkError(cam2->SetConfiguration(&config))) return -1;
	return 0;
}
int main(int argc, char **argv) {
	// Initialize SDL2's context 
	//Use this function to initialize the SDL library. This must be called before using any other SDL function.
	SDL_Init(SDL_INIT_VIDEO); 
	// Initialize Oculus' context
	ovrResult result = ovr_Initialize(nullptr);
	if (OVR_FAILURE(result)) {
		std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
		SDL_Quit();
		return -1;
	}

	ovrSession session;
	ovrGraphicsLuid luid;
	// Connect to the Oculus headset
	result = ovr_Create(&session, &luid);
	if (OVR_FAILURE(result)) {
		std::cout << "ERROR: Oculus Rift not detected" << std::endl;
		ovr_Shutdown();
		SDL_Quit();
		return -1;
	}

	int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
	//we stablish the width and height of the window
	//int winWidth = 1280;
	//int winHeight = 480;
	int winWidth = 1600;
	int winHeight = 600;
	//int winWidth = 2160;
	//int winHeight = 1200;
	Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
	// Create SDL2 Window
	SDL_Window* window = SDL_CreateWindow("OVR POINT GREY App", x, y, winWidth, winHeight, flags);
	// Create OpenGL context
	SDL_GLContext glContext = SDL_GL_CreateContext(window);
	// Initialize GLEW
	glewInit();
	// Turn off vsync to let the compositor do its magic
	SDL_GL_SetSwapInterval(0);

	// Initialize the PG Camera
	Camera cam1, cam2;
	Image rawImage1, rawImage2, convertedImage1, convertedImage2;
	PrintBuildInfo();


	// Since this application saves images in the current folder
	// we must ensure that we have permission to write to this folder.
	// If we do not have permission, fail right away.
	FILE* tempFile = fopen("test.txt", "w+");
	if (tempFile == NULL)
	{
		cout << "Failed to create file in current folder.  Please check permissions." << endl;
		return -1;
	}

	fclose(tempFile);
	remove("test.txt");

	if (initializePGCameras(&cam1, &cam2)) return -1;

	// Start capturing images
	if (checkError(cam1.StartCapture()) || checkError(cam2.StartCapture())) return -1;
	
	GLuint pgTextureID_L, pgTextureID_R;
	int pgWidth = 1280;
	int pgHeight = 960;
	//int pgWidth = 800;
	//int pgHeight = 600;
	// Generate OpenGL texture for left images of the PG camera
	glGenTextures(1, &pgTextureID_L); //generate a texture name
	glBindTexture(GL_TEXTURE_2D, pgTextureID_L); 
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pgWidth, pgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// Generate OpenGL texture for right images of the PG camera
	glGenTextures(1, &pgTextureID_R);
	glBindTexture(GL_TEXTURE_2D, pgTextureID_R);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pgWidth, pgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);

	ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
	// Get the texture sizes of Oculus eyes
	ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], 1.0f);
	ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], 1.0f);
	// Compute the final size of the render buffer
	ovrSizei bufferSize;
	bufferSize.w = textureSize0.w + textureSize1.w;
	bufferSize.h = max(textureSize0.h, textureSize1.h);
	// Initialize OpenGL swap textures to render
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
		return -1;
	}
	// Generate frame buffer to render
	GLuint fboID;
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

	// Create a mirror texture to display the render result in the SDL2 window
	ovrMirrorTextureDesc descMirrorTexture;
	memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
	descMirrorTexture.Width = winWidth;
	descMirrorTexture.Height = winHeight;
	descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

	ovrMirrorTexture mirrorTexture = nullptr;
	result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
	if (!OVR_SUCCESS(result)) {
		std::cout << "ERROR: Failed to create mirror texture" << std::endl;
	}
	GLuint mirrorTextureId;
	ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

	GLuint mirrorFBOID;
	glGenFramebuffers(1, &mirrorFBOID);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
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
	Shader shader(OVR_PG_VS, OVR_PG_FS);


	// Compute the PG image field of view with the PG parameters
	float pgFovH = atanf(pgWidth / (4.5f*2.f)) * 1.f;
	// Compute the Horizontal Oculus' field of view with its parameters
	float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
	// Compute the useful part of the PG image
	unsigned int usefulWidth = pgWidth * ovrFovH / pgFovH;
	// Compute the size of the final image displayed in the headset with the PG image's aspect-ratio kept
	unsigned int widthFinal = bufferSize.w / 2;
	float heightGL = 1.f;
	float widthGL = 1.f;
	if (usefulWidth > 0.f) {
		unsigned int heightFinal = pgHeight * widthFinal / usefulWidth;
		// Convert this size to OpenGL viewport's frame's coordinates
		heightGL = (heightFinal) / (float)(bufferSize.h);
		widthGL = ((pgWidth * (heightFinal / (float)pgHeight)) / (float)widthFinal);
	}
	else {
		std::cout << "WARNING: PG parameters got wrong values."
			"Default vertical and horizontal FOV are used.\n"
			"Check your calibration file or check if your PG is not too close to a surface or an object."
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
	// Initialize a boolean that will be used to stop the applications loop and another one to pause/unpause rendering
	bool end = false;
	bool refresh = true;
	// SDL variable that will be used to store input events
	SDL_Event events;
	// Initialize time variables. They will be used to limit the number of frames rendered per second.
	// Frame counter
	unsigned int riftc = 0, pgc = 1;
	// Chronometer
	unsigned int rifttime = 0, pgtime = 0, pgFPS = 0;
	int time1 = 0, timePerFrame = 0;
	int frameRate = (int)(1000 / MAX_FPS);

	// This boolean is used to test if the application is focused
	bool isVisible = true;

	// Enable the shader
	glUseProgram(shader.getProgramId());
	// Bind the Vertex Buffer Objects of the rectangle that displays PG images
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

	// Main loop
	while (!end) {
		// Compute the time used to render the previous frame
		timePerFrame = SDL_GetTicks() - time1;
		// If the previous frame has been rendered too fast
		if (timePerFrame < frameRate) {
			// Pause the loop to have a max FPS equal to MAX_FPS
			SDL_Delay(frameRate - timePerFrame);
			timePerFrame = frameRate;
		}
		// Increment the PG chronometer
		pgtime += timePerFrame;
		// If PG chronometer reached 1 second
		if (pgtime > 1000) {
			pgFPS = pgc;
			pgc = 0;
			pgtime = 0;
		}
		// Increment the Rift chronometer and the Rift frame counter
		rifttime += timePerFrame;
		riftc++;
		// If Rift chronometer reached 200 milliseconds
		if (rifttime > 200) {
			// Display FPS
			std::cout << "\rRIFT FPS: " << 1000 / (rifttime / riftc) << " | PG FPS: " << pgFPS;
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
			}
		}

		// Get texture swap index where we must draw our frame
		GLuint curTexId;
		int curIndex;
		ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
		ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

		// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
		eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
		eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
		hmdToEyeOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
		hmdToEyeOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;
		// Get eye poses, feeding in correct IPD offset
		ovr_GetEyePoses(session, frameIndex, ovrTrue, hmdToEyeOffset, eyeRenderPose, &sensorSampleTime);


		// If the application is focused
		if (isVisible) {
			// If successful grab a new PG image
				ostringstream filename1, filename2;
				// Retrieve an image
				if (checkError(cam1.RetrieveBuffer(&rawImage1)) || checkError(cam2.RetrieveBuffer(&rawImage2))) continue;

				// Convert the raw image
				if (checkError(rawImage1.Convert(PIXEL_FORMAT_RGB, &convertedImage1)) || checkError(rawImage2.Convert(PIXEL_FORMAT_RGB, &convertedImage2))) return -1;
				pgc++;
				int a = convertedImage1.GetDataSize();
				if (refresh) {
					// Bind the frame buffer
					glBindFramebuffer(GL_FRAMEBUFFER, fboID);
					// Set its color layer 0 as the current swap texture
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
					// Set its depth layer as our depth buffer
					glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
					// Clear the frame buffer
					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					glClearColor(0, 0, 0, 1);

					// Render for each Oculus eye the equivalent PG image
					for (int eye = 0; eye < 2; eye++) {
						// Set the left or right vertical half of the buffer as the viewport
						glViewport(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
						//glViewport(0, 0, bufferSize.w, bufferSize.h);
						// Bind the left or right PG image
						glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? pgTextureID_L : pgTextureID_R);
#if !OPENGL_GPU_INTEROP
						Image imagepicture;
						if (eye == ovrEye_Left) 
						{
							glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pgWidth, pgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, convertedImage1.GetData());
						}
						else
						{
							glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pgWidth, pgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, convertedImage2.GetData());
						}
#endif
						// Bind the hit value
						glUniform1f(glGetUniformLocation(shader.getProgramId(), "hit"), eye == ovrEye_Left ? hit : -hit);
						// Bind the isLeft value
						glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
						// Draw the PG image
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
			return -1;
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
		glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
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
		SDL_GL_SwapWindow(window);
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
	// Stop capturing images
	if (checkError(cam1.StopCapture()) || checkError(cam2.StopCapture())) return -1;
	// Disconnect the camera
	if (checkError(cam1.Disconnect()) || checkError(cam2.Disconnect())) return -1;
	// Quit
	return 0;
}
