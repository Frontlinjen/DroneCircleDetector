#include <vector>
#include "ring_detector/RingBucket.h"

void RingBucket::Insert(RingData* data){
	RingBucketContainer * bucket = Get(data->abs_x, data->abs_y);
	bucket-> push_back(data);

}

RingBucketContainer * RingBucket::Get(float x, float y){
	int bucket_x = static_cast<int>(x);
	int bucket_y = static_cast<int>(y);
	return m_buckets[bucket_x, bucket_y];
}

void RingBucket::Remove(RingData* data){
	RingBucketContainer * bucket = Get(data->abs_x, data->abs_y);
	for(RingBucketContainer::iterator itr = bucket->begin(); itr != bucket->end(); ++itr){
		if(*itr == data)
		{
			bucket->erase(itr);
			break;
		}
	}
}

RingBucket::~RingBucket(){
  RingBucketContainer::iterator itr;
  for(unsigned int i = 0;i<grid_height;++i){
    for(unsigned int m = 0; m<grid_width;++m){
      RingBucketContainer * container = &m_buckets[i][m];
      itr = container->begin();
      while(itr != container->end()){
	delete *itr;
	++itr;
      }
      container->clear();
    }
  }
}
