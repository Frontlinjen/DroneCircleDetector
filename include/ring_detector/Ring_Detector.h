#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <semaphore.h>
#include <chrono>
#include <utility>
#include <math.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include "Camera_Node.h"


template<typename T, int MIN=0, int MAX=50, int RATIO=1>
void TrackbarCallback(int pos, void * ptr){
    if(pos < MIN || pos > MAX)
        return;
    double ratio = 1/RATIO;
    T * val = static_cast<T*>(ptr);
    *val = pos*RATIO;
}

class Ring_Detector{
private:
	const float UPDATE_RATE;
	int minDist, minRadius, maxRadius, param1, param2;

public:
	void processImage( cv_bridge::CvImageConstPtr image);

	Ring_Detector(Camera_Node & node) : UPDATE_RATE(120){
		minDist = 1;
		minRadius = 100;
		maxRadius = 100;
		param1 = 100;
		param2 = 100;
		cv::namedWindow("Drone feed", 1);
		cv::createTrackbar("minDist", "Drone feed", &minDist, 300, TrackbarCallback<int>, &minDist);
		cv::createTrackbar("param1", "Drone feed", &param1, 300, TrackbarCallback<int>, &param1);
		cv::createTrackbar("param2", "Drone feed", &param2, 300, TrackbarCallback<int>, &param2);
		cv::createTrackbar("minRadius", "Drone feed", &minRadius, 300, TrackbarCallback<int>, &minRadius);
		cv::createTrackbar("maxRadius", "Drone feed", &maxRadius, 300, TrackbarCallback<int>, &maxRadius);
		node.registerCallback(imageCallBack, this);
	}
    static void imageCallBack(const cv_bridge::CvImageConstPtr & imagePtr, void * _this);
};
