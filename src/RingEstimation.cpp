#include "ring_detector/RingEstimation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ring_detector/MessageFormats.h"
#include <ring_detector/RingData.h>


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
	if(circles != NULL)
		{
			for(std::vector<CircleData>::iterator itr = circles->objects.begin(); itr != circles->objects.end(); ++itr){
				RingDataInternal *ringData = new RingDataInternal();

				//calulating absoulte x and y for the ring
				float cathetusA = sin(drone_yaw) * itr->distance;
				float cathetusB = cos(drone_yaw) * itr->distance;
				ringData->abs_x = (drone_x - itr->x) + cathetusA;
				ringData->abs_y = (drone_y - itr->y) + cathetusB;

				if(ringData->abs_x >= grid_height || ringData->abs_y >= grid_width){
							delete ringData;
							continue;
				}

				RingBucketContainer * bucket = m_Bucket.Get(ringData->abs_x, ringData->abs_y);
				if(bucket->empty()){
					m_Bucket.Insert(ringData);
					ringData->ringViewCount = 1;
				}
				else{
					bucket->front()->ringViewCount += 1;
					delete ringData;
				}
			}
		}
	if(QR != NULL){
		for(std::vector<QRData>::iterator itr = QR->objects.begin(); itr != QR->objects.end(); ++itr){
			RingDataInternal *ringData = new RingDataInternal();
			//calulating absoulte x and y for the ring
			float cathetusA = sin(drone_yaw) * itr->distance;
			float cathetusB = cos(drone_yaw) * itr->distance;
			ringData->abs_x = (drone_x - itr->x) + cathetusA;
			ringData->abs_y = (drone_y - itr->y) + cathetusB;

			if(ringData->abs_x >= grid_height || ringData->abs_y >= grid_width){
				delete ringData;
				continue;
			}
			RingBucketContainer * bucket = m_Bucket.Get(ringData->abs_x, ringData->abs_y);
			if(bucket->empty()){
				m_Bucket.Insert(ringData);
				ringData->QRViewCount = 1;
				ringData->ring_number = itr->ring_number;
			}
			else{
				bucket->front()->QRViewCount++;
				delete ringData;
			}
		}
	}
	for(size_t x = 0; x < grid_height ; ++x){
		for(size_t y = 0; y < grid_height ; ++y){
			RingBucketContainer* bucket = m_Bucket.Get(x,  y);
			for(RingBucketContainer::iterator itr = bucket->begin(); itr != bucket->end(); ++itr){
				float accuracy = (*itr)->GetAccuracy();
				if((*itr)->lastBroadcastAccuracy < accuracy){

					ring_detector::RingData d;
					d.ring_number = (*itr)->ring_number;
					d.delta_x = (*itr)->delta_x;
					d.delta_y = (*itr)->delta_y;
					d.delta_z = (*itr)->delta_z;
					d.abs_x = (*itr)->abs_x;
					d.abs_y = (*itr)->abs_y;
					d.abs_z = (*itr)->abs_z;
					d.norm_x = (*itr)->norm_x;
					d.norm_y = (*itr)->norm_y;
					d.possibility = accuracy;

					publisher.publish(d);
					(*itr)->lastBroadcastAccuracy = accuracy;
				}
			}
		}
	}
}
