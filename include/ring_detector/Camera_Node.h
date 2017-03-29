#pragma once
#include <ros/ros.h>
#include <semaphore.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
typedef void (*processImageCallback)(const cv_bridge::CvImageConstPtr & imagePtr, void * _this);

class Camera_Node{
  struct callbackEntry{
    callbackEntry(processImageCallback callback, void * data)
    {
      func = callback;
      userdata = data;
    }
    processImageCallback func;
    void * userdata;
  };
  typedef std::vector<Camera_Node::callbackEntry> CallbackContainer;
  CallbackContainer callbacks;
 public:
  void imageCallback(const sensor_msgs::ImageConstPtr & img);
  void registerCallback(processImageCallback, void * data);
 Camera_Node() : it(nodeHandle){
    sem_init(&semaphore, 0,1);
    sub = it.subscribe<Camera_Node>("ardrone/front/image_raw", 60, &Camera_Node::imageCallback, this);
  };
  ~Camera_Node(){
    sem_destroy(&semaphore);
  };

 private:
  void processImage(const sensor_msgs::ImageConstPtr & msg);
  sem_t semaphore;
  ros::NodeHandle nodeHandle;
  image_transport::Subscriber sub;
  image_transport::ImageTransport it;
};
