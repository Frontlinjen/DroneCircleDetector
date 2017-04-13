#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
struct ImageProcessor{
  virtual void ProcessImage(const cv_bridge::CvImageConstPtr resource) = 0;
  virtual ~ImageProcessor() = 0;
};

inline ImageProcessor::~ImageProcessor(){

}






