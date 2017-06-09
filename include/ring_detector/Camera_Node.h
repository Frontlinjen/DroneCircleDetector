#pragma once
#include <ros/ros.h>
#include <stdio.h>
#include <atomic>
#include <thread>
#include <vector>
#include <mutex>
#include <time.h>
#include "AtomicQueue.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ImageProcessor.h"
#include "SharedResource.h"
class ProcessingThread;

typedef ImageProcessor* ProcessImageCallback;
typedef std::vector<std::thread> WorkerList;
typedef std::vector<ProcessingThread> JobList;

class Camera_Node{
 public:
  void Start();
  void ImageCallback(const sensor_msgs::ImageConstPtr & img);
  void RegisterCallback(ProcessImageCallback c);
  void WhiteBalance(const cv::Mat& in, float percentage, cv::Mat& out);
  void equalize(const cv::Mat& in);
  Camera_Node() : it(nodeHandle)
     {
        sub = it.subscribe<Camera_Node>("ardrone/front/image_raw", 60, &Camera_Node::ImageCallback, this);
  };
  ~Camera_Node(){
  };

 private:
  ros::NodeHandle nodeHandle;
  WorkerList  m_Workers;
  JobList m_Jobs;
  image_transport::Subscriber sub;
  image_transport::ImageTransport it;
  SharedResource<cv_bridge::CvImageConstPtr> currentImage;
};

class ProcessingThread{
  time_t m_StartTime;
  SharedResource<cv_bridge::CvImageConstPtr>& m_CurrentImage;
  ProcessImageCallback func;
  bool running;
public:
  ProcessingThread(SharedResource<cv_bridge::CvImageConstPtr> & imageStore, ProcessImageCallback entry);
  void Run();
  void Stop() { running = false; }
};
