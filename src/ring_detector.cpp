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

void Ring_Detector:: ProcessImage(const cv_bridge::CvImageConstPtr resource)
{
  if(resource.get() == NULL)
    return;
  cv::Mat grad;
  std::vector<int> v;
	double delta = 0.0;
	double scale = 1.0;
	int ddepth = CV_16S;
	cv::Mat droneFeed;
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
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
    	cv::HoughCircles(droneFeed, circles, CV_HOUGH_GRADIENT, 1, minDist, dparam1, dparam2, dminRadius, dmaxRadius);
    	for( size_t i = 0; i < circles.size(); i++ )
    	{
    		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        	int radius = cvRound(circles[i][2]);
        	// draw the circle center
         	cv::circle(droneFeed, center, 3, cv::Scalar(255,255,255), -1, 8, 0);
         	// draw the circle outline
         	cv::circle(droneFeed, center, radius, cv::Scalar(255,0,255), 3, 8, 0);
    	}
    	imshow("Drone Feed", droneFeed);
        cv::waitKey(1);
}
 
 
