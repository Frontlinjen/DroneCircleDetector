#include <shared_mutex>
#include <mutex>




template<typename T>
class SharedResource{
  mutable std::shared_timed_mutex m_Mutex;
  T resource;
 public:
  SharedResource(const T& init){
    resource = init;
  }
  SharedResource(){
  }
  T Get(){
    std::shared_lock<std::shared_timed_mutex>  lock(m_Mutex);
    return resource;
  }
    
  void Set(const T& element){
    std::unique_lock<std::shared_timed_mutex> lock(m_Mutex);
    resource = element;
  }


};
