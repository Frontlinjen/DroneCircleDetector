#pragma once
#include "MessageFormats.h"
#include "RingBucket.h"
#include <deque>
#include <condition_variable>
#include <mutex>
#include "ros/ros.h"
#include <tum_ardrone/filter_state.h>


class RingEstimation{
	std::deque<CircleScanResult*> m_CircleResults;
	std::deque<QRScanResult*> m_ScanResults;
	std::mutex m_lock;
	std::condition_variable m_cond;
	ros::Publisher publisher;
	ros::Subscriber subscriber;
	RingBucket m_Bucket;
	bool m_Running;
	float drone_x, drone_y, drone_yaw;
public:

	//Called from thread 1
	void Recieve(CircleScanResult* result);
	//Called from thread 2
	void Recieve(QRScanResult* result);
	RingEstimation(ros::NodeHandle n);
	void Run();
	void UpdatePosition(tum_ardrone::filter_state msg);
	void inline ProcessImage(CircleScanResult* circles, QRScanResult* QR);
};
