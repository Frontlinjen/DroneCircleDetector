#pragma once
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "SharedResource.h"
struct ImageProcessor{
  virtual void ProcessImage(const Resource<cv_bridge::CvImageConstPtr> resource) = 0;
  virtual ~ImageProcessor() = 0;
};

inline ImageProcessor::~ImageProcessor(){

}






