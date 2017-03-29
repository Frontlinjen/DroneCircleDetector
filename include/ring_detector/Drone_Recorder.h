#include <ros/ros.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

class Drone_Recorder{

	cv::VideoWriter * writer;

	Drone_Recorder(){
		const int width = 1280;
		const int height = 720;
		const int fps = 30;
		const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
		writer = new cv::VideoWriter("sample.avi", fourcc, fps, cv::Size(width, height));
	}

	~Drone_Recorder(){
		writer->flush();
		writer->release();
	}
};
