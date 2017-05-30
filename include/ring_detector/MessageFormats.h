#include <vector>
typedef unsigned int FrameID;

struct CircleData{
  unsigned int x, y;
  unsigned int radius;
  float angle;
};

struct QRData{
  unsigned int x, y;
  float angle;
  char ring_number;
  unsigned int px_w, px_h;
};

template<typename T>
struct ScanResult{
  FrameID frameID;
  std::vector<T> objects;
};

typedef ScanResult<CircleData> CircleScanResult;
typedef ScanResult<QRData> QRScanResult;

ScanResult<QRData> QRScanResult;

struct RingData{
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y; //The rings position relative to the camera
  unsigned float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  unsigned float norm_x, norm_y; //The direction of the ring 
  struct RingData{
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y; //The rings position relative to the camera
  unsigned float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  unsigned float norm_x, norm_y; //The direction of the ring 
  unsigned long timestamp;
  //char possibility;
};
unsigned long timestamp;
  //char possibility;
};
struct RingData{
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y, delta_z; //The rings position relative to the camera
  unsigned float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  unsigned float norm_x, norm_y; //The direction of the ring 
  unsigned long timestamp;
  unsigned long viewcount;
};

struct RingDataUpdate{
  char ring_number; //The ring number, if known. Calculated from QR
  float delta_x, delta_y, delta_z; //The rings position relative to the camera
  unsigned float abs_x, abs_y, abs_z; //The rings position in world coordinates.
  unsigned float norm_x, norm_y; //The direction of the ring 
  unsigned long timestamp;
  char possibility;
};



