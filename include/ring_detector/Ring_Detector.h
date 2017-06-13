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
#include "RingEstimation.h"



template<typename T, int MIN=0, int MAX=50, int RATIO=1>
void TrackbarCallback(int pos, void * ptr){
	if(pos < MIN || pos > MAX)
		return;
	double ratio = 1/RATIO;
	T * val = static_cast<T*>(ptr);
	*val = pos*RATIO;
}

class Ring_Detector : public ImageProcessor {
	const float UPDATE_RATE;
	int param1, param2, minSaturation, minValue, hueValue, hueRange, treshold, minLength, maxGap, lineParam;
	bool initialized;
	int * m_default; //The trackbar needs a pointer even though the documentation marks it as "optional"
	RingEstimation * m_callOnFinish; //Object which gathers information from QR and Image
public:
	Ring_Detector(RingEstimation* est) : UPDATE_RATE(120){
		m_callOnFinish = est;
		m_default = new int(1);
		initialized = false;
		param1 = 100;
		param2 = 100;
		minSaturation = 100;
		minValue = 50;
		hueValue = 0;
		hueRange = 15;
		treshold = 40;
		minLength = 10;
		maxGap = 10;
		lineParam = 1;
	}
	
	void ProcessImage(const Resource<cv_bridge::CvImageConstPtr> resource) override;
    virtual ~Ring_Detector() override {
      cv::destroyWindow("Drone feed");
      delete m_default;
    }	
    
};
