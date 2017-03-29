#include <ros/ros.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Camera_Node.h"

class Drone_Recorder{

	cv::VideoWriter * writer;
	static void processImageCallback(const cv_bridge::CvImageConstPtr & imagePtr, void * _this);

public:
	Drone_Recorder(){
		const int width = 1280;
		const int height = 720;
		const int fps = 30;
		const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
		writer = new cv::VideoWriter("sample.avi", fourcc, fps, cv::Size(width, height));
	}
	void processImage(const cv_bridge::CvImageConstPtr image);

	~Drone_Recorder(){
		writer->release();
		delete writer;
	}
};
