#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <semaphore.h>
#include "ring_detector/Camera_Node.h"
#include "ring_detector/Ring_Detector.h"
#include <opencv2/highgui.hpp>
#include "ring_detector/QR_Detector.h"
#include "ring_detector/RingEstimation.h"
#include <iostream>
#include <opencv2/core/ocl.hpp>
#include <cmath>
#include <thread>
//Not thread safe
void Camera_Node::RegisterCallback(ProcessImageCallback c){
  m_Jobs.emplace_back(currentImage, c);
  m_Workers.emplace_back(&ProcessingThread::Run, m_Jobs.back());
}
void chatterCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}


//Thread safe
void Camera_Node::ImageCallback(const sensor_msgs::ImageConstPtr & msg)
{
	boost::shared_ptr<cv_bridge::CvImage> bridge = cv_bridge::toCvCopy(msg, "bgr8");
	//FISH EYE CORRECTION
		cv::Mat *img = &(bridge->image);
		cv::Mat distImage(img->rows, img->cols, CV_32F);
		float data[9] = {
				561.999146, 0, 307.433982,
				0, 561.782697, 190.144373,
				0, 0, 1};

		std::vector<float> d {-0.50758, 0.24911, 0.000579, 0.000996, 0};
		cv::Mat k = cv::Mat(3, 3, CV_32FC1, &data);
		cv::undistort(*img, distImage, k, d);
		*img = distImage;
	//Vi deler billedet med alle andre
	currentImage.Set(bridge);
}



int main(int argc, char ** argv){
  ros::init(argc, argv, "RingDetector");
  std::cout << "Using OpenCV: " << CV_VERSION << '\n';
  std::cout << "Supports OpenCL: " << cv::ocl::haveOpenCL() << '\n';
  ros::NodeHandle n;
  RingEstimation estimator(n);
  Camera_Node cNode;
  Ring_Detector detector(&estimator);
  QR_Detector qrdetector(&estimator);
  std::thread t(&RingEstimation::Run, std::ref(estimator));
  cNode.RegisterCallback(static_cast<ImageProcessor*>(&detector));
  cNode.RegisterCallback(static_cast<ImageProcessor*>(&qrdetector));
  cNode.Start();
  t.join();
}


void Camera_Node::Start(){
  while(ros::ok()){
    ros::spinOnce();
    
  }
  //Tell workers to finish whatever they are doing
  JobList::iterator itr = m_Jobs.begin();
  while(itr != m_Jobs.end()){
    itr->Stop();
    ++itr;
  }
  WorkerList::iterator workers = m_Workers.begin();
  while(workers != m_Workers.end()){
    workers->join();
    ++workers;
  }
  m_Jobs.clear();
  m_Workers.clear();
  cv::destroyAllWindows();
}


ProcessingThread::ProcessingThread(SharedResource<cv_bridge::CvImageConstPtr> & imageStore, ProcessImageCallback entry) :
  m_CurrentImage(imageStore),
  func(entry),
  running(true)
  {}
void ProcessingThread::Run(){
  while(running){
   m_StartTime = time(0); //Logs the start time, so that we know when we started
   func->ProcessImage(m_CurrentImage.Get());
  }
}
