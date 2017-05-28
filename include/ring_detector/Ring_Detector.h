#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <semaphore.h>
#include <chrono>
#include <utility>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include "Camera_Node.h"
#include "ImageProcessor.h"



template<typename T, int MIN=0, int MAX=50, int RATIO=1>
void TrackbarCallback(int pos, void * ptr){
    if(pos < MIN || pos > MAX)
        return;
    double ratio = 1/RATIO;
    T * val = static_cast<T*>(ptr);
    *val = pos*RATIO;
}

  class Ring_Detector : public ImageProcessor {
private:
	const float UPDATE_RATE;
	int minDist, minRadius, maxRadius, param1, param2;
	bool initialized;

public:
	void ProcessImage(const cv_bridge::CvImageConstPtr resource) override;
    virtual ~Ring_Detector() override {
      cv::destroyWindow("Drone feed");
    }
	Ring_Detector() : UPDATE_RATE(120){
       	minDist = 1;
		minRadius = 100;
		maxRadius = 100;
		initialized = false;
		param1 = 100;
		param2 = 100;
	}
    
};
