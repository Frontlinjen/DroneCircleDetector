#pragma once
#include "MessageFormats.h"
#include "RingBucket.h"

class RingEstimation{
  std::vector<CircleScanResult*> circleResults;
  std::vector<QRScanResult*> scanResults;
  RingBucket bucket;
  bool running;
 public:

  
  //Called from thread 1
  void Recieve(CircleScanResult* result);
  //Called from thread 2
  void Recieve(QRScanResult* result);

  void Run();
};
