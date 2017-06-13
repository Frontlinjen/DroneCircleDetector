#include "ring_detector/RingEstimation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


RingEstimation::RingEstimation(ros::NodeHandle n){
	ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);

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

void RingEstimation::ProcessImage(CircleScanResult* circles, QRScanResult* QR) {
	CircleData *Circledata = new CircleData();
	//QRData *QRdata = new QRData();
	RingData *ringData = new RingData();
	//time_t timev;
	/*for(std::vector<QRData>::iterator itr = QR->objects.begin(); itr != QR->objects.end(); ++itr){
		ringData->delta_x = QRdata->x;
		ringData->delta_y = QRdata->y;
	}*/

	for(std::vector<CircleData>::iterator itr = circles->objects.begin(); itr != circles->objects.end(); ++itr){
		//RingData *ringData = new RingData();
		ringData->abs_x = 0;
		ringData->abs_z = 0;
		ringData->abs_y = Circledata->distance;
		//ringData->abs_x = drone->x - ringData->delta_x;
		//ringData->abs_y = drone->y - ringData->delta_y;
		/*float i_x = ringData->abs_x - QRdata->x;
		float i_y = ringData->abs_y - QRdata->y;
		if(i_x <=1 && i_x >= -1 && i_y <=1 && i_y >= -1){
			ringData->ring_number = QRdata->ring_number;
			ringData->distance = QRdata->distance;
		}*/
		//ringData->timestamp = std::time(&timev);
		/*if(RingData(ringData->abs_x, ringData->abs_y) != null){
			ringData->viewcount = ringData->viewcount + 1;
		}else
			ringData->viewcount = 1;*/
		if(m_Bucket.Get(ringData->abs_x, ringData->abs_y)->empty()){
			m_Bucket.Insert(ringData);
		}
	}

	delete circles;
	delete QR;
}
