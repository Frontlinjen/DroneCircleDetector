#pragma once
#include <vector>
typedef unsigned long FrameID;

struct CircleData{
  unsigned int x, y;
  unsigned int radius;
  float angle;
  float distance;
};

struct QRData{
  unsigned int x, y;
  float angle;
  char ring_number;
  float distance;
};

template<typename T>
struct ScanResult{
  FrameID frameID;
  std::vector<T> objects;
};

typedef ScanResult<CircleData> CircleScanResult;
typedef ScanResult<QRData> QRScanResult;

struct RingDataInternal{
  float GetAccuracy(){
	  return (0.01*ringViewCount + 0.1*QRViewCount) / (0.01*ringViewCount + 0.1*QRViewCount + 10);
  }
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y, delta_z; //The rings position relative to the camera
  float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  float norm_x, norm_y; //The direction of the ring
  float distance;
  float lastBroadcastAccuracy;
  unsigned long timestamp;
  unsigned long ringViewCount;
  unsigned long QRViewCount;
};

struct RingDataUpdate{
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y, delta_z; //The rings position relative to the camera
  float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  float norm_x, norm_y; //The direction of the ring
  unsigned long timestamp;
  char possibility;
};



