#include <ros/ros.h>
#include <semaphore.h>
#include <image_transport/image_transport.h>


class Camera_Node{
 public:
  static Camera_Node * getInstance();
  static void imageCallback(const sensor_msgs::ImageConstPtr & img);

 private:
  void processImage(const sensor_msgs::ImageConstPtr & msg);
  Camera_Node() : it(nodeHandle){
    sem_init(&semaphore, 0,1);
    sub = it.subscribe("ardrone/front/image_raw", 60, imageCallback);
    setupTrackbars();
  }
  void setupTrackbars(){

  }

  ~Camera_Node(){
    sem_destroy(&semaphore);
  }
  static Camera_Node * instance;
  sem_t semaphore;
  ros::NodeHandle nodeHandle;
  image_transport::Subscriber sub;
  image_transport::ImageTransport it;
};
