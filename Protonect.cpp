#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include "viewer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ctime>
#include <windows.h>

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s) {
  protonect_shutdown = true;
}


// *****************************************************************************************************
// ************************** SUPPORT FUNCTIONS (THUY NGUYEN, APR 12, 2017) ****************************
// *****************************************************************************************************

#define DEV01_SERIAL	"010003364147"
#define DEV02_SERIAL	"003463564047"

#define PNG_EXTENTION	".png"
#define BMP_EXTENTION	".bmp"


std::string getCurrentTimeAsString() {
	time_t currentTime;
	time(&currentTime); // get the current time

	struct tm *localTime = localtime(&currentTime);  // Convert the current time to the local time
	int year = localTime->tm_year + 1900;	// 4 digits
	int month = localTime->tm_mon + 1;		// 1 to 2 digits
	int day = localTime->tm_mday;			// 1 to 2
	int hour = localTime->tm_hour;			// 1 to 2
	int minute = localTime->tm_min;			// 1 to 2
	int second = localTime->tm_sec;			// 1 to 2

	// format the current time as a char array
	char charStr[100];
	sprintf(charStr, "%d-%02d-%02d_%02d-%02d-%02d_%d", year, month, day, hour, minute, second, GetTickCount64());

	return std::string(charStr);
}


void saveDepthImage(libfreenect2::Frame *depth, cv::String filename) {

	// copy libfreenect2's depthImg (float) to OpenCV's depthImg (CV_32FC1)
	// ref: https://github.com/giacomodabisias/libfreenect2pclgrabber/blob/master/include/k2g.h
	cv::Mat depth32FC1(depth->height, depth->width, CV_32FC1, depth->data);

	// convert OpenCV's depthImg (CV_32FC1) to OpenCV's depthImg (CV_16UC1)
	// ref: https://github.com/OpenKinect/libfreenect2/issues/438
	cv::Mat depth16UC1;
	depth32FC1.convertTo(depth16UC1, CV_16UC1);

	// convert OpenCV's depthImg (CV_16UC1) (THIS IS ALSO THE FORMAT OF MICROSOFT KINECT SDK'S IDEPTHFRAME) to Kinect2Toolbox's depthImg (CV_16UC1) to save to PNG files
	// ref: https://github.com/xiaozhuchacha/Kinect2Toolbox/blob/master/DepthRecorder/DepthBasics.cpp
	cv::imwrite(filename, depth16UC1 / 8); 

	// if wanted to display using OpenCV
	//cv::imshow("test", depthImg16UC1 / 8); // this shows black
	//cv::imshow("test", depthImg32FC1 / 4096.0f); // ref: https://github.com/chihyaoma/KinectOneStream/blob/master/KinectOneStream.cpp

	// release
	depth32FC1.release();
	depth16UC1.release();
}


void saveRGBImage(libfreenect2::Frame *alignedRGB, cv::String filename) { // registed image of depth and color image OR original-high resolution RGB image
	cv::Mat aligned8UC4(alignedRGB->height, alignedRGB->width, CV_8UC4, alignedRGB->data); // ref: https://github.com/chihyaoma/KinectOneStream/blob/master/KinectOneStream.cpp
	cv::imwrite(filename, aligned8UC4);
	aligned8UC4.release();
}


void saveFrames(libfreenect2::Frame *depth01, libfreenect2::Frame *alignedRGB01, libfreenect2::Frame *rgb01) {
	std::string currTimeStr = getCurrentTimeAsString();

	std::stringstream ss01depth;	ss01depth	<< DEV01_SERIAL << "_depth_"	<< currTimeStr << PNG_EXTENTION;		saveDepthImage(depth01, ss01depth.str());
	std::stringstream ss01aligned;	ss01aligned << DEV01_SERIAL << "_aligned_"	<< currTimeStr << PNG_EXTENTION;		saveRGBImage(alignedRGB01, ss01aligned.str());
	std::stringstream ss01rgb;		ss01rgb		<< DEV01_SERIAL << "_rgb_"		<< currTimeStr << BMP_EXTENTION;		saveRGBImage(rgb01, ss01rgb.str());	// save to BMP is much faster than PNG

	// release
	ss01depth.clear();		ss01depth.str(std::string());  // release stringstream
	ss01aligned.clear();	ss01aligned.str(std::string());
	ss01rgb.clear();		ss01rgb.str(std::string());
}


void saveFrames(libfreenect2::Frame *depth01, libfreenect2::Frame *alignedRGB01, libfreenect2::Frame *rgb01,
				libfreenect2::Frame *depth02, libfreenect2::Frame *alignedRGB02, libfreenect2::Frame *rgb02) {
	std::string currTimeStr = getCurrentTimeAsString();

	std::stringstream ss01depth;	ss01depth	<< DEV01_SERIAL << "_depth_"	<< currTimeStr << PNG_EXTENTION;		saveDepthImage(depth01, ss01depth.str());
	std::stringstream ss01aligned;	ss01aligned << DEV01_SERIAL << "_aligned_"	<< currTimeStr << PNG_EXTENTION;		saveRGBImage(alignedRGB01, ss01aligned.str());
	std::stringstream ss01rgb;		ss01rgb		<< DEV01_SERIAL << "_rgb_"		<< currTimeStr << BMP_EXTENTION;		saveRGBImage(rgb01, ss01rgb.str());

	std::stringstream ss02depth;	ss02depth	<< DEV02_SERIAL << "_depth_"	<< currTimeStr << PNG_EXTENTION;		saveDepthImage(depth02, ss02depth.str());
	std::stringstream ss02aligned;	ss02aligned << DEV02_SERIAL << "_aligned_"	<< currTimeStr << PNG_EXTENTION;		saveRGBImage(alignedRGB02, ss02aligned.str());
	std::stringstream ss02rgb;		ss02rgb		<< DEV02_SERIAL << "_rgb_"		<< currTimeStr << BMP_EXTENTION;		saveRGBImage(rgb02, ss02rgb.str());

	// release
	ss01depth.clear();		ss01depth.str(std::string());  // release stringstream
	ss01aligned.clear();	ss01aligned.str(std::string());
	ss01rgb.clear();		ss01rgb.str(std::string());

	ss02depth.clear();		ss02depth.str(std::string());  // release stringstream
	ss02aligned.clear();	ss02aligned.str(std::string());
	ss02rgb.clear();		ss02rgb.str(std::string());
}


int GetDeviceListener(libfreenect2::Freenect2 *freenect2, std::string serial, libfreenect2::Freenect2Device **dev,
					  libfreenect2::SyncMultiFrameListener *listener, libfreenect2::Registration **registration) {
	*dev = freenect2->openDevice(serial);
	if (*dev == 0) {
		std::cout << "Failure opening device " << serial << std::endl;
		return -1;
	}

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	(*dev)->setColorFrameListener(listener);
	(*dev)->setIrAndDepthFrameListener(listener);

	if (!(*dev)->start())
		return -1;
	std::cout << "Device " << (*dev)->getSerialNumber() << " (firmware " << (*dev)->getFirmwareVersion() << ")" << std::endl;

	*registration = new libfreenect2::Registration((*dev)->getIrCameraParams(), (*dev)->getColorCameraParams());

	return 1;
}


// *****************************************************************************************************
// ********************************* END - SUPPORT FUNCTIONS *******************************************
// *****************************************************************************************************


int main(int argc, char *argv[]) {
  std::string program_path(argv[0]);
  std::cerr << "Usage: " << program_path << " [-noviewer] [-nosaver] [-use1dev] [-help]" << std::endl;
  std::cerr << "        [-frames <number of frames to process>]" << std::endl;


  // ------------ [context]
  bool viewer_enabled = true;
  bool saver_enabled = true;
  bool use2devices = true;
  size_t framemax = -1;

  for(int argI = 1; argI < argc; ++argI) {
    const std::string arg(argv[argI]);

    if(arg == "-help" || arg == "--help" || arg == "-h") {      
		return 0; // Just let the initial lines display at the beginning of main
    } else if(arg == "-noviewer" || arg == "--noviewer") {
		viewer_enabled = false;
	} else if (arg == "-nosaver" || arg == "--nosaver") {
		saver_enabled = false;
	} else if (arg == "-use1dev" || arg == "--use1dev") {
		use2devices = false;
	} else if (arg == "-frames") {
		++argI;
		framemax = strtol(argv[argI], NULL, 0);
		if (framemax == 0) {
			std::cerr << "invalid frame count '" << argv[argI] << "'" << std::endl;
			return -1;
		}
    } else {
		std::cout << "Unknown argument: " << arg << std::endl;
    }
  }


  // ------------ [discovery]
  libfreenect2::Freenect2 freenect2;
  if(freenect2.enumerateDevices() == 0) {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  

  // ------------ [dev] [listeners] [registration setup] 
  libfreenect2::Freenect2Device *dev01 = 0;
  libfreenect2::Freenect2Device *dev02 = 0;
  
  //int types = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener01(types); 
  libfreenect2::SyncMultiFrameListener listener02(types);

  libfreenect2::Registration* registration01 = 0;
  libfreenect2::Registration* registration02 = 0;

  GetDeviceListener(&freenect2, DEV01_SERIAL, &dev01, &listener01, &registration01);
  if (use2devices) GetDeviceListener(&freenect2, DEV02_SERIAL, &dev02, &listener02, &registration02);


  // ------------ [frame setup]
  libfreenect2::FrameMap frames01;
  libfreenect2::FrameMap frames02;

  libfreenect2::Frame *rgb01, *ir01, *depth01;
  libfreenect2::Frame *rgb02, *ir02, *depth02;

  int depthW = 512, depthH = 424, nChannels = 4;
  libfreenect2::Frame undistorted01(depthW, depthH, nChannels), registered01(depthW, depthH, nChannels);
  libfreenect2::Frame undistorted02(depthW, depthH, nChannels), registered02(depthW, depthH, nChannels);
  

  // ------------ [loop start]
  Viewer viewer;
  if (viewer_enabled)
	  viewer.initialize();

  size_t framecount = 0;
  int timeout10s = 10000; // 10*1000 (10 seconds)
  

  while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax)) {

	// capture frames from Device 1 
    if (!listener01.waitForNewFrame(frames01, timeout10s)) { // 10 seconds
      std::cout << "Device " << DEV01_SERIAL << " " << "timeout!" << std::endl;
      return -1;
    }
	rgb01 = frames01[libfreenect2::Frame::Color];
	//ir01 = frames01[libfreenect2::Frame::Ir];
	depth01 = frames01[libfreenect2::Frame::Depth];
	registration01->apply(rgb01, depth01, &undistorted01, &registered01);

	// capture frames from Device 2
	if (use2devices) {
		if (!listener02.waitForNewFrame(frames02, timeout10s)) {
			std::cout << "Device " << DEV02_SERIAL << " " << "timeout!" << std::endl;
			return -1;
		}
		rgb02 = frames02[libfreenect2::Frame::Color];
		//ir02 = frames02[libfreenect2::Frame::Ir];
		depth02 = frames02[libfreenect2::Frame::Depth];
		registration02->apply(rgb02, depth02, &undistorted02, &registered02);
	}

	// save frames
	if (saver_enabled) {
		if (use2devices)
			saveFrames(depth01, &registered01, rgb01, depth02, &registered02, rgb02);
		else
			saveFrames(depth01, &registered01, rgb01);
	}

	// check even when viewer is not running
    framecount++;
    if (!viewer_enabled) {
      if (framecount % 100 == 0)
		  std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener01.release(frames01);
	  listener02.release(frames02);
      continue;
    }

	// show frames, when viewer is enabled
    /*viewer.addFrame("RGB", rgb01);		// correct naming is required:  "ir" is top-left,		"registered" is top-right
    viewer.addFrame("ir", ir02);			//								"RGB" is bottom-left,	"depth" is bottom-right,
	viewer.addFrame("depth", depth01);
    viewer.addFrame("registered", &registered01);
	*/
	viewer.addFrame("RGB", rgb01);
	viewer.addFrame("depth", depth01);
	if (use2devices) {
		viewer.addFrame("ir", depth02);
		viewer.addFrame("registered", rgb02);
	}

    protonect_shutdown = protonect_shutdown || viewer.render();


	// loop end
    listener01.release(frames01);
	listener02.release(frames02);
    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(

  // ------------ [stop]
  dev01->stop();
  dev01->close();
  
  if (use2devices) {
	  dev02->stop();
	  dev02->close();
  }
  delete registration01;
  delete registration02;
  return 0;
}
