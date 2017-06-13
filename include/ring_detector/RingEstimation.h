#pragma once
#include "MessageFormats.h"
#include "RingBucket.h"
#include <deque>
#include <condition_variable>
#include <mutex>

class RingEstimation{
  std::deque<CircleScanResult*> m_CircleResults;
  std::deque<QRScanResult*> m_ScanResults;
  std::mutex m_lock;
  std::condition_variable m_cond;
  RingBucket m_Bucket;
  bool m_Running;
 public:
  
  //Called from thread 1
  void Recieve(CircleScanResult* result);
  //Called from thread 2
  void Recieve(QRScanResult* result);

  void Run();
  void inline ProcessImage(CircleScanResult* circles, QRScanResult* QR);
};
