#include "ring_detector/RingEstimation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ring_detector/MessageFormats.h"
#include "ring_detector/RingData.h"

RingEstimation::RingEstimation(ros::NodeHandle n){
	subscriber = n.subscribe<tum_ardrone::filter_state>("/ardrone/predictedPose", 1000, &RingEstimation::UpdatePosition, this);
	publisher = n.advertise<ring_detector::RingData>("circleData", 1000);
	ros::Rate loop_rate(10);
}

void RingEstimation::UpdatePosition(tum_ardrone::filter_state msg){
	drone_x = msg.x;
	drone_y = msg.y;
	drone_yaw = msg.yaw;
	printf("%s %f \n%s %f","x: ", drone_x,"y: ", drone_y);
}

void RingEstimation::Recieve(CircleScanResult* result){
	{
		std::lock_guard<std::mutex> lock(m_lock);
		m_CircleResults.push_back(result); //Not thread safe
	}
	m_cond.notify_one();
}

void RingEstimation::Recieve(QRScanResult* result){
	{
		std::lock_guard<std::mutex> lock(m_lock);
		m_ScanResults.push_back(result);
	}
	m_cond.notify_one();
}

void RingEstimation::Run(){
	m_Running = true;
	while(m_Running){
		CircleScanResult* circle = NULL;
		QRScanResult* QR = NULL;
		std::unique_lock<std::mutex> lock(m_lock);
		m_cond.wait(lock, [this]()->bool{return m_CircleResults.size() > 1 && m_ScanResults.size() > 1;});

		circle = m_CircleResults.front();
		QR = m_ScanResults.front();
		if(circle->frameID != QR->frameID){
			//We only process the most recent one
			if(circle->frameID > QR->frameID){
				circle = NULL;
				m_ScanResults.pop_front();
			}
			else
			{
				QR = NULL;
				m_CircleResults.pop_front();
			}
		}
		else{
			m_CircleResults.pop_front();
			m_ScanResults.pop_front();
		}
		m_lock.unlock(); //No reason for us to hold onto the lock anymore.
		ProcessImage(circle, QR);
	}
}

void RingEstimation::ProcessImage(CircleScanResult* circles, QRScanResult* QR){

	//CircleData *Circledata = new CircleData();
	QRData *QRdata = new QRData();
	RingDataInternal *ringData = new RingDataInternal();
	//time_t timev;
	ring_detector::RingData data;
	if(circles == NULL){
		for(std::vector<QRData>::iterator itr = QR->objects.begin(); itr != QR->objects.end(); ++itr){
			data.delta_x = 0;
			data.delta_y = itr->distance;
			data.delta_z = 0.60; //Skal reestimeres


			//calulating absoulte x and y for the ring
			float cathetusA = sin(drone_yaw) * itr->distance;
			float cathetusB = cos(drone_yaw) * itr->distance;
			ringData->abs_x = (drone_x - data.delta_x) + cathetusA;
			ringData->abs_y = (drone_y - data.delta_y) + cathetusB;

			if(m_Bucket.Get(ringData->abs_x, ringData->abs_y)->empty()){
				m_Bucket.Insert(ringData);
				ringData->viewcount = 1;
				ringData->accuracy = 0.0;
				ringData->ring_number = itr->ring_number;
			}
			//Example on our accuracy estimation
			else
				ringData->viewcount = ringData->viewcount + 1;
			ringData->accuracy = ringData->accuracy + 15;
			if(ringData->accuracy >=65)
				publisher.publish(data);

		}
	}
	else
	{
		for(std::vector<CircleData>::iterator itr = circles->objects.begin(); itr != circles->objects.end(); ++itr){
			data.delta_x = 0; //bredde
			data.delta_y = itr->distance; //Fremad
			data.delta_z = 0; //Højde

			//calulating absoulte x and y for the ring
			float cathetusA = sin(drone_yaw) * itr->distance;
			float cathetusB = cos(drone_yaw) * itr->distance;

			ringData->abs_x = (drone_x - ringData->delta_x) + cathetusA;
			ringData->abs_y = (drone_y - ringData->delta_y) + cathetusB;

			//calculating if the ring/QR code are within same area
			float i_x = ringData->abs_x - QRdata->x;
			float i_y = ringData->abs_y - QRdata->y;
			if(i_x <=1 && i_x >= -1 && i_y <=1 && i_y >= -1){
				ringData->ring_number = QRdata->ring_number;
				ringData->distance = QRdata->distance;
			}
			//ringData->timestamp = std::time(&timev);
			if(m_Bucket.Get(ringData->abs_x, ringData->abs_y)->empty()){
				m_Bucket.Insert(ringData);
				ringData->viewcount = 1;
				ringData->accuracy = 0.0;
			}
			//Example on our accuracy estimation
			else
				ringData->viewcount = ringData->viewcount + 1;
			ringData->accuracy = ringData->accuracy + 1;
			if(ringData->accuracy >=65)
				publisher.publish(data);
			//publisher.publish(data);
		}
	}
	delete circles;
	delete QR;
}
