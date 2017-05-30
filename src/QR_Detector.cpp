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
#include <math.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include "ring_detector/QR_Detector.h"
#include <zbar.h>


void QR_Detector::ProcessImage(const cv_bridge::CvImageConstPtr resource)
{
	if(resource.get() == NULL)
			return;
	cv::Mat img;
	cv::cvtColor(resource->image,img,CV_BGR2GRAY);
	zbar::Image imageToScan(img.cols, img.rows, "Y800", img.data, img.cols*img.rows);
	int result = scanner.scan(imageToScan);
	for(zbar::Image::SymbolIterator itr = imageToScan.symbol_begin(); itr != imageToScan.symbol_end(); ++itr){
		std::cout << "decoded" << itr->get_type_name() << "symbol" << itr->get_data() << "" << "" << std::endl;
	}
	imageToScan.set_data(NULL, 0);
}

