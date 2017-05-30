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

class QR_Detector : public ImageProcessor{
	zbar::ImageScanner scanner;
	public:
		void ProcessImage(const cv_bridge::CvImageConstPtr resource) override;
		virtual ~QR_Detector() override {
		      cv::destroyWindow("MyVideo");
		    }
		QR_Detector(){
			scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
		}
};
