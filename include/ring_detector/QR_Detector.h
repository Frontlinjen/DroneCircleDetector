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
#include <zbar.h>
#include "RingEstimation.h"

constexpr int widthWall = 214;
constexpr int widthRing = 194;
constexpr float constantWall = 514.0186915888;
constexpr float constantRing = 561.2244897959;

class QR_Detector : public ImageProcessor{
	zbar::ImageScanner scanner;
	float deltaDistance = 0;
	RingEstimation* m_callOnFinish;
public:
	QR_Detector(RingEstimation* callback){
		m_callOnFinish = callback;
	}
	float Scalarproduct(cv::Vec2f a, cv::Vec2f b){
		float i = (a[0] * b[0]) + (a[1] * b[1]);
		return i;
	}
	float Pythagoras(cv::Vec2f a, cv::Vec2f b){
		float i = sqrt((pow(a[0], 2) + pow(a[1], 2))) * sqrt((pow(b[0], 2) + pow(b[1], 2)));
		return i;
	}
	void ProcessImage(const Resource<cv_bridge::CvImageConstPtr> resource) override;
	virtual ~QR_Detector() override {
		cv::destroyWindow("MyVideo");
	}
	QR_Detector(){
		scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
	}
};
