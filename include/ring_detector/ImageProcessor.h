#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
struct ImageProcessor{
  virtual void ProcessImage(const sensor_msgs::ImageConstPtr & resource) = 0;
  virtual ~ImageProcessor() = 0;
};






