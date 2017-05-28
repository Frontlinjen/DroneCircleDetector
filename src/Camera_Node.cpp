#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <semaphore.h>
#include "ring_detector/Camera_Node.h"
#include "ring_detector/Ring_Detector.h"
#include <opencv2/highgui.hpp>
#include "ring_detector/QR_Detector.h"
//Not thread safe
void Camera_Node::RegisterCallback(ProcessImageCallback c){
  m_Jobs.emplace_back(currentImage, c);
  m_Workers.emplace_back(&ProcessingThread::Run, m_Jobs.back());
}


//Thread safe
void Camera_Node::ImageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  currentImage.Set(cv_bridge::toCvShare(msg));
}



int main(int argc, char ** argv){
  ros::init(argc, argv, "RingDetector");
  Camera_Node cNode;
  Ring_Detector detector;
  QR_Detector qrdetector;
  cNode.RegisterCallback(static_cast<ImageProcessor*>(&detector));
  cNode.RegisterCallback(static_cast<ImageProcessor*>(&qrdetector));
  cNode.Start();
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
