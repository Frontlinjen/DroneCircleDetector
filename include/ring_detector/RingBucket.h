#include <vector>
#include "ring_detector/MessageFormats.h"
constexpr unsigned int grid_height = 11;
constexpr unsigned int grid_width = 10;
typedef std::vector<RingData*> RingBucketContainer;


class RingBucket{
RingBucketContainer m_buckets[grid_height][grid_width];
private:

public:
//Inserts the ring into a bucket and takes ownership
void Insert(RingData* data);

//Gets the buckets at coordinates
RingBucketContainer * Get(float x, float y);

//Removes the entry from the container and frees ownership
void Remove(RingData* data);

//Deletes all owned RingData
~RingBucket();

};
