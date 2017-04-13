#include <mutex>
#include <vector>
#include <deque>
#include <condition_variable>
template<typename T>
class AtomicQueue{
  std::deque<T> m_Queue;
  std::mutex m_Mutex;
  std::condition_variable m_Cond;
 public:
  T Dequeue(){
    std::unique_lock<std::mutex> lock(m_Mutex);
    while(m_Queue.empty())
    {
      m_Cond.wait(lock);
    }
    T obj = m_Queue.pop_front();
    lock.unlock();
  }

  void EnqueueAll(const std::vector<T> & toQueue){
    std::lock_guard<std::mutex> lock(m_Mutex);
    typename std::vector<T>::iterator itr = toQueue.begin();
    while(itr != toQueue.end()){
      m_Queue.push_back(*itr);
      m_Cond.notify_one();
    }
  }
  
  void Enqueue(T obj)
  {
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Queue.push_back(obj);
    m_Cond.notify_one();
  }
};
