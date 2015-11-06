#ifndef _CONTAINER_H_
#define _CONTAINER_H_

#include <deque>

template< typename T >
class container1
{

  public:
   
  container1()
  {

  }

  ~container1()
  {

  }

  int size()
  {
   return queue.size();
  }

  T first()
  {
   return queue.front();
  }
  T last()
  {
   return queue.back();
  }

  void push_back(const T& object)
  {
    queue.push_back(object);
  }

  void pop_front()
  {
    queue.pop_front();
  }

  
  private:

  std::deque< T > queue;


};



#endif //_CONTAINER_H_
