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
#define PI 3.14159265


void QR_Detector::ProcessImage(const Resource<cv_bridge::CvImageConstPtr> resource)
{
	if(resource.resource.get() == NULL)
		return;
	cv::Mat img;
	cv::cvtColor(resource.resource->image,img,CV_BGR2GRAY);
	img = img > 128;
	zbar::Image imageToScan(img.cols, img.rows, "Y800", img.data, img.cols*img.rows);
	int result = scanner.scan(imageToScan);
	QRScanResult * qrResult = new QRScanResult();
	qrResult->frameID = resource.id;
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

		if(itr->get_data().at(0) == 'p' || itr->get_data().at(0) == 'P')
		{
			data->ring_number = std::atoi(&itr->get_data().at(3));
			std::cout << "Ring number: " << data->ring_number << std::endl;
			distance = (widthRing * constantRing) / pixels;
		}

		if(itr->get_data().at(0) == 'w' || itr->get_data().at(0) == 'W'){
			std::cout << "The QR code is a wall! Which belongs to the ";
			distance = (widthWall * constantWall) / pixels;
			switch(itr->get_data().at(2)){
			case 0 : std::cout << "north wall" << std::endl;
			break;
			case 1 : std::cout << "east wall" << std::endl;
			break;
			case 2 : std::cout << "south wall" << std::endl;
			break;
			case 3 : std::cout << "west wall" << std::endl;;
			break;
			}
		}

		float pixelLength = distance * 0.22;
		float abs_centerX = r.center.x;
		float abs_distance = distance;
		if(r.center.x > 320.0){
			abs_centerX = ((r.center.x - 320.0) * pixelLength) / 100;

		}
		if(r.center.x < 320.0){
			abs_centerX = ((320.0 - r.center.x) * pixelLength) / 100;
		}
		abs_distance = sqrt((pow(distance, 2) + pow(abs_centerX, 2)));

		std::cout << "QR-Distance: " << distance << std::endl;
		data->angle = r.angle;
		data->distance = abs_distance;
		data->x = r.center.x;
		data->y = r.center.y;


	}
	imageToScan.set_data(NULL, 0);
	m_callOnFinish->Recieve(qrResult);
}

