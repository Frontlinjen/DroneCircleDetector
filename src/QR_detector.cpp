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
#include "ring_detector/Ring_Detector.h"
#include <zbar.h>
using namespace cv;
using namespace std;
using namespace zbar;

void QR_Detector:: ProcessImage(const cv_bridge::CvImageConstPtr resource)
{
	if(resource.get() == NULL)
		return;

	  ImageScanner scanner;
	    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	   double dWidth = resource(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	   double dHeight = resource(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	   cout << "Frame size : " << dWidth << " x " << dHeight << endl;
	   namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	   while (1)
	   {
	     Mat frame;
	     bool bSuccess = resource(frame); // read a new frame from video
	      if (!bSuccess) //if not success, break loop
	     {
	        cout << "Cannot read a frame from video stream" << endl;
	        break;
	     }
	     Mat grey;
	     cvtColor(frame,grey,CV_BGR2GRAY);
	     int width = frame.cols;
	     int height = frame.rows;
	     uchar *raw = (uchar *)grey.data;
	     // wrap image data
	     Image image(width, height, "Y800", raw, width * height);
	     // scan the image for barcodes
	     int n = scanner.scan(image);
	     // extract results
	     for(Image::SymbolIterator symbol = image.symbol_begin();
	     symbol != image.symbol_end();
	     ++symbol) {
	         vector<Point> vp;
	     // do something useful with results
	     cout << "decoded " << symbol->get_type_name() << " symbol "" << symbol->get_data() << '"' <<" "<< endl;
	       int n = symbol->get_location_size();
	       for(int i=0;i<n;i++){
	         vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
	       }
	       RotatedRect r = minAreaRect(vp);
	       Point2f pts[4];
	       r.points(pts);
	       for(int i=0;i<4;i++){
	         line(frame,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
	       }
	       //cout<<"Angle: "<<r.angle<<endl;
	     }
	     imshow("MyVideo", frame); //show the frame in "MyVideo" window
	     if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	     {
	       cout << "esc key is pressed by user" << endl;
	       break;
	     }
	   }
	   return;
	 }
}

