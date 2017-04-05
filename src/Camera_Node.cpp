#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <semaphore.h>
#include "ring_detector/Camera_Node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "ring_detector/AutoTimeMeasure.h"
#include "ring_detector/Drone_Recorder.h"
#include "ring_detector/Ring_Detector.h"



void Camera_Node::registerCallback(processImageCallback callback, void * data){
  callbacks.emplace_back(callback, data);
}



void Camera_Node::imageCallback(const sensor_msgs::ImageConstPtr & msg)
   {
     processImage(msg);
   }

void Camera_Node::processImage(const sensor_msgs::ImageConstPtr & msg)
  {
    if(sem_trywait(&semaphore)!=0)
      return;
    BEGIN_SCOPE_MEASURE("ImageProcessing");
    cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(msg);
    for(CallbackContainer::iterator itr = callbacks.begin(); itr != callbacks.end(); ++itr){
      itr->func(imagePtr, itr->userdata);
    }
    cv::waitKey(1);
    sem_post(&semaphore);
  }

int main(int argc, char ** argv){
  ros::init(argc, argv, "RingDetector");
  Camera_Node cNode;
  Ring_Detector detector(cNode);
  while(ros::ok()){
    ros::spinOnce();
    cv::waitKey(1);
  }

}
