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
#include <math.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include "ring_detector/Ring_Detector.h"
#include "ring_detector/MessageFormats.h"

void Ring_Detector:: ProcessImage(const cv_bridge::CvImageConstPtr resource)
{
	if(resource.get() == NULL)
		return;
	if(!initialized)
	{

		cv::namedWindow("Drone feed", CV_WINDOW_NORMAL);
		cv::namedWindow("Input feed", CV_WINDOW_NORMAL);
		cv::createTrackbar("minDist", "Input feed", dummy, 300, TrackbarCallback<int, 1, 300>, &minDist);
		cv::createTrackbar("param1", "Input feed", dummy, 300, TrackbarCallback<int, 1, 300>, &param1);
		cv::createTrackbar("param2", "Input feed", dummy, 300, TrackbarCallback<int, 1, 300>, &param2);
		cv::createTrackbar("minRadius", "Input feed", dummy, 300, TrackbarCallback<int, 1, 300>, &minRadius);
		cv::createTrackbar("maxRadius", "Input feed", dummy, 300, TrackbarCallback<int, 1, 300>, &maxRadius);
		cv::createTrackbar("minSaturation", "Input feed", dummy, 300, TrackbarCallback<int, 100, 300>, &minSaturation);
		cv::createTrackbar("minValue", "Input feed", dummy, 300, TrackbarCallback<int, 50, 300>, &minValue);
		cv::createTrackbar("hueValue", "Input feed", dummy, 300, TrackbarCallback<int, 0, 300>, &hueValue);
		cv::createTrackbar("hueRange", "Input feed", dummy, 300, TrackbarCallback<int, 15, 300>, &hueRange);
		initialized = true;
	}
	cv::Mat grad;
	std::vector<int> v;
	double delta = 0.0;
	double scale = 1.0;
	int ddepth = CV_16S;
	cv::Mat droneFeed;
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	//Red scalling i HSV
	cv::cvtColor(resource->image, droneFeed, CV_BGR2HSV);
	std::vector<cv::Mat> hsvChannels;
	cv::split(droneFeed, hsvChannels);
    cv::Mat hueImage = hsvChannels[0];
    cv::Mat hueMask;
//  int hueValue = 0; // rød
//  int hueRange = 15;
    // min sat = 50
    // min val = 50
    // hue val = 120
    // range =10/15
    cv::inRange(hueImage, hueValue - hueRange, hueValue + hueRange, hueMask);
    //Tjek om farven er indenfor vores Huerange*
    if (hueValue - hueRange < 0 || hueValue + hueRange > 180)
    {
    	cv::Mat hueMaskUpper;
        int upperHueValue = hueValue + 180;
        cv::inRange(hueImage, upperHueValue - hueRange, upperHueValue + hueRange, hueMaskUpper);
        hueMask = hueMask | hueMaskUpper;
    }
    //Vi sortere resten fra
    cv::Mat saturationMask = hsvChannels[1] > minSaturation;
    cv::Mat valueMask = hsvChannels[2] > minValue;
    hueMask = (hueMask & saturationMask) & valueMask;
    cv::imshow("red", hueMask);

    //Circle detection
    cv::cvtColor(resource->image, droneFeed, CV_BGR2GRAY);
	cv::GaussianBlur(droneFeed, droneFeed, cv::Size(9, 9), 1, 1, cv::BORDER_DEFAULT );
	cv::Sobel(droneFeed, grad_x, CV_32FC1, 1, 0, 3);
	convertScaleAbs( grad_x, abs_grad_x );
	convertScaleAbs( grad_y, abs_grad_y );
	cv::Sobel(droneFeed, grad_y, CV_32FC1, 0, 1, 3);
	std::vector<cv::Vec3f> circles;
	double dminDist = minDist / 1.0;
	double dparam1 = param1 / 1.0;
	double dparam2 = param2 / 1.0;
	double dminRadius = minRadius / 1.0;
	double dmaxRadius = maxRadius / 1.0;
	cv::HoughCircles(droneFeed, circles, CV_HOUGH_GRADIENT, 1, minDist, dparam1, dparam2, dminRadius, dmaxRadius);
	CircleScanResult * circleResult = new CircleScanResult;
	for(std::vector<cv::Vec3f>::iterator itr = circles.begin(); itr != circles.end(); ++itr )
	{
		circleResult->objects.emplace_back();
		CircleData * data = &circleResult->objects.back();
		cv::Point center(cvRound((*itr)[0]), cvRound((*itr)[1]));
		int radius = cvRound((*itr)[2]);
		// draw the circle center
		cv::circle(droneFeed, center, 3, cv::Scalar(255,255,255), -1, 8, 0);
		// draw the circle outline
		cv::circle(droneFeed, center, radius, cv::Scalar(255,0,255), 3, 8, 0);
		data->radius = radius;
		data->angle = 0;
		data->x = center.x;
		data->y = center.y;
	}
	imshow("Drone Feed", droneFeed);
	cv::waitKey(1);
	
}


