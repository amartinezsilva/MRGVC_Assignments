#pragma once

#include <atomic>
#include <functional>
#include <vector>

#include<join_threads.hpp>
#include<threadsafe_queue.hpp>

/*Added*/
#include<chrono>
using namespace std::chrono;
/*End added*/

class thread_pool
{
  
  /*Added*/
  std::atomic<bool> _done;
  size_t _thread_count;
  threadsafe_queue<std::function<void()>> _work_queue;
  std::vector<std::thread> _threads;
  /*End added*/

  join_threads _joiner;

  using task_type = void();

  void worker_thread()
  {
    while(!_done){
      std::function<task_type> task;
      if(_work_queue.try_pop(task)){
        task();
      } else {
        std::this_thread::yield();
      }
    }
  }

  public:
  thread_pool(size_t num_threads = std::thread::hardware_concurrency())
    : _done(false), _thread_count(num_threads), _joiner(_threads)
  {

      for(size_t i = 0; i < _thread_count; ++i){
        _threads.push_back(std::thread(&thread_pool::worker_thread, this));
      }


  }

  ~thread_pool()
  {
    wait();
    _done = true; 
  }

  void wait()
  {
      // wait for completion
      while(!_work_queue.empty()){
        std::this_thread::sleep_for(milliseconds(100));
      }
      // active waiting
  }

  template<typename F>
    void submit(F f)
    {
      _work_queue.push(std::function<task_type>(f));
    }
};
