#include <shared_mutex>
#include <mutex>
#include <condition_variable>
#include <ros/console.h>


template<typename T>
class SharedResource{
  mutable std::shared_timed_mutex m_Mutex;
  std::condition_variable_any m_Condition;
  T resource;
  
 public:
  SharedResource(const T& init){
    resource = init;
  }
  SharedResource(){
  }
  T Get(){
    std::shared_lock<std::shared_timed_mutex>  lock(m_Mutex);
    m_Condition.wait(lock);
    return resource;
  }
    
  void Set(const T& element){
    std::unique_lock<std::shared_timed_mutex> lock(m_Mutex);
    resource = element;
    m_Condition.notify_all();
  }


};
