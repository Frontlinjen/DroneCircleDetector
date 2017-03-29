#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include <ring_detector/Drone_Recorder.h>

void Drone_Recorder::processImage(const cv_bridge::CvImageConstPtr image)
{
}

void Drone_Recorder::processImageCallback(const cv_bridge::CvImageConstPtr & imagePtr, void * _this)
{
	Drone_Recorder * recorder = static_cast<Drone_Recorder*>(_this);
	recorder->processImage(imagePtr);
}
