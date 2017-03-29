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
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>

template<typename T, int MIN=0, int MAX=100, int RATIO=1>
void TrackbarCallback(int pos, void * ptr){
    if(pos < MIN || pos > MAX)
        return;
    double ratio = 1/RATIO;
    T * val = static_cast<T*>(ptr);
    *val = pos*RATIO;
}

#define MEASURE_TIME 1

#ifdef MEASURE_TIME
#define BEGIN_SCOPE_MEASURE(name)  AutoTimeMeasure mTime(name)
#else
#define BEGIN_SCOPE_MEASURE(name) do {} while(0)
#endif

class AutoTimeMeasure{
    const char * name;
    std::chrono::high_resolution_clock::time_point start;
public:
    AutoTimeMeasure(const char * c)
    {
        name = c;
        start = std::chrono::high_resolution_clock::now();
    }
    ~AutoTimeMeasure(){
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        printf("%s finished in %fs\n", name, std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()/1000000000.0);
    }
};

const float UPDATE_RATE = 120.0;
sem_t * semaphore;
cv::VideoWriter * writer;

int minDist = 10;
int minRadius = 10;
int maxRadius = 10;
int param1 = 30;
int param2 = 150;

void processImage( cv_bridge::CvImageConstPtr image);

void imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(sem_trywait(semaphore)!=0)
		return;

	BEGIN_SCOPE_MEASURE("ImageProcessing");
	cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(msg);
	processImage(imagePtr);
	cv::waitKey(1);
	sem_post(semaphore);
}
void SetupRecorder()
{
//#if RECORD_STREAM
    const int width = 1280;
    const int height = 720;
    const int fps = 30;
    const int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G'); //cap.get(CAP_PROP_FOURCC);
    writer = new cv::VideoWriter("sample.avi", fourcc, fps, Size(width, height));
//    #endif
}

//min, max, step?


int main(int argc, char ** argv)
{

	semaphore = new sem_t();
	sem_init(semaphore, 0,1);
	SetupRecorder();
	cv::namedWindow("Drone feed", 1);
    cv::createTrackbar("minDist", "Drone feed", &minDist, 300);
    cv::createTrackbar("param1", "Drone feed", &param1, 300);
    cv::createTrackbar("param2", "Drone feed", &param2, 300);
    cv::createTrackbar("minRadius", "Drone feed", &minRadius, 300);
    cv::createTrackbar("maxRadius", "Drone feed", &maxRadius, 300);

    using namespace image_transport;
	ros::init(argc, argv, "RingDetector");

    ros::NodeHandle nodeHandle;
	ImageTransport it(nodeHandle);
    Subscriber sub = it.subscribe("ardrone/front/image_raw", 60, imageCallback);
    while(ros::ok()){
		ros::spinOnce();
		cv::waitKey(1);
    }
	sem_destroy(semaphore);
    writer->release();
    delete writer;
	delete semaphore;

    return 0;
}

void processImage( cv_bridge::CvImageConstPtr image)
{
    cv::Mat grad;
	double delta = 0.0;
	double scale = 1.0;
	int ddepth = CV_16S;
	cv::Mat droneFeed;
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::cvtColor(image->image, droneFeed, CV_BGR2GRAY);
	cv::GaussianBlur(droneFeed, droneFeed, cv::Size(9, 9), 1, 1, cv::BORDER_DEFAULT );
	cv::Sobel(droneFeed, grad_x, CV_32FC1, 1, 0, 3);
	convertScaleAbs( grad_x, abs_grad_x );
	convertScaleAbs( grad_y, abs_grad_y );
	cv::Sobel(droneFeed, grad_y, CV_32FC1, 0, 1, 3);
	std::vector<cv::Vec3f> circles;
    	double dminDist = minDist / 1.0;
    	double dparam1 = param1 / 1.0;
    	double dparam2 = param2 / 1.0;
    	double dminRadius = minRadius / 1.0;
    	double dmaxRadius = maxRadius / 1.0;
    	cv::HoughCircles(droneFeed, circles, CV_HOUGH_GRADIENT, 1, minDist, dparam1, dparam2, dminRadius, dmaxRadius);
    	cv::HoughCircles(droneFeed, circles, CV_HOUGH_GRADIENT, 1, minDist, dparam1, dparam2, dminRadius, dmaxRadius);
    	for( size_t i = 0; i < circles.size(); i++ )
    	{
    		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        	int radius = cvRound(circles[i][2]);
        	// draw the circle center
         	cv::circle(droneFeed, center, 3, cv::Scalar(255,255,255), -1, 8, 0);
         	// draw the circle outline
         	cv::circle(droneFeed, center, radius, cv::Scalar(255,0,255), 3, 8, 0);
    	}
    	imshow("Drone Feed", droneFeed);
    	return;
}
