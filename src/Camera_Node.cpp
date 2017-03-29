#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <semaphore.h>
#include "ring_detector/Camera_Node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "ring_detector/AutoTimeMeasure.h"

Camera_Node * Camera_Node::getInstance()
 {
    if(instance == nullptr){
      instance = new Camera_Node();
    }
 }

Camera_Node * Camera_Node::instance = nullptr;

void Camera_Node::imageCallback(const sensor_msgs::ImageConstPtr & msg)
   {
     Camera_Node * _this = Camera_Node::getInstance();
     _this->processImage(msg);
   }

void Camera_Node::processImage(const sensor_msgs::ImageConstPtr & msg)
  {
    if(sem_trywait(&semaphore)!=0)
      return;
    BEGIN_SCOPE_MEASURE("ImageProcessing");
    cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(msg);
    cv::waitKey(1);
    sem_post(&semaphore);
  }

int main(int argc, char ** argv)
{
  cv::namedWindow("Drone feed", 1);
  cv::createTrackbar("minDist", "Drone feed", &minDist, 300);
  cv::createTrackbar("param1", "Drone feed", &param1, 300);
  cv::createTrackbar("param2", "Drone feed", &param2, 300);
  cv::createTrackbar("minRadius", "Drone feed", &minRadius, 300);
  cv::createTrackbar("maxRadius", "Drone feed", &maxRadius, 300);

  using namespace image_transport;
  ros::init(argc, argv, "RingDetector");

  ros::NodeHandle nodeHandle;
  ImageTransport it(nodeHandle);
  Subscriber sub = it.subscribe("ardrone/front/image_raw", 60, Camera_Node::imageCallback);
  while(ros::ok()){
    ros::spinOnce();
    cv::waitKey(1);
  }
  return 0;
}
