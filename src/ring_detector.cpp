
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/utility.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <image_transport/image_transport.h>
void imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(msg);
    cv::imshow("Drone feed", image->image);
    cv::waitKey(1);
}

int main(int argc, char ** argv)
{
	cv::namedWindow("Drone feed", 1);
	using namespace image_transport;
    ros::init(argc, argv, "RingDetector");
    ros::NodeHandle nodeHandle;
	ImageTransport it(nodeHandle);
    Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}
