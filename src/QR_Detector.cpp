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
#include "ring_detector/MessageFormats.h"


void QR_Detector::ProcessImage(const cv_bridge::CvImageConstPtr resource)
{
	if(resource.get() == NULL)
			return;
	cv::Mat img;
	cv::cvtColor(resource->image,img,CV_BGR2GRAY);
	img = img > 128;
	zbar::Image imageToScan(img.cols, img.rows, "Y800", img.data, img.cols*img.rows);
	int result = scanner.scan(imageToScan);
	QRScanResult * qrResult = new QRScanResult;
	for(zbar::Image::SymbolIterator itr = imageToScan.symbol_begin(); itr != imageToScan.symbol_end(); ++itr){
		qrResult->objects.emplace_back();
		QRData * data = &qrResult->objects.back();
		std::cout << "decoded" << itr->get_type_name() << "symbol" << itr->get_data() << "" << "" << std::endl;
		std::vector<cv::Point> vp;
		int n = itr->get_location_size();
		for(int i = 0; i < n; i++){
			vp.push_back(cv::Point(itr->get_location_x(i), itr->get_location_y(i)));
		}
		cv::RotatedRect r = minAreaRect(vp);
		cv::Point2f pts[4];
		r.points(pts);
		for(int i = 0; i < 4; i++){
			cv::line(img, pts[i], pts[(i+1)%4],cv::Scalar(255,0,0),3);
		}
		int pixels = r.size.width;
		deltaDistance = (width * constant) / pixels;
		std::cout << "Angle: " << r.angle << std::endl;
		std::cout << "Distance: " << deltaDistance << std::endl;
		//data->ring_number = itr->get_data();
		data->angle = r.angle;
		data->distance = deltaDistance;
		data->x = r.center.x;
		data->y = r.center.y;


	}
	imageToScan.set_data(NULL, 0);
}

