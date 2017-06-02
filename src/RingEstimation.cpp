#include "ring_detector/RingEstimation.h"
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
    m_cond.wait(lock, [this]()->bool{return m_CircleResults.size() > 0 && m_ScanResults.size() > 0 && (m_CircleResults.front()->frameID == m_ScanResults.front()->frameID);});
circle = m_CircleResults.front();
QR = m_ScanResults.front();
m_CircleResults.pop_front();
m_ScanResults.pop_front();
m_lock.unlock(); //No reason for us to hold onto the lock anymore.
ProcessImage(circle, QR);
  }      
}

void RingEstimation::ProcessImage(CircleScanResult* circles, QRScanResult* QR){
  //Process image..

  delete circles;
  delete QR;
}
