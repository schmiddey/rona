/*
 * Timer.h
 *
 *  Created on: 02.03.2016
 *      Author: m1ch1
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <iostream>
#include <string>
#include <chrono>

namespace rona{


class Timer {
public:
   Timer(std::string msg = std::string("")) : _msg(msg)
   {  this->start(); }
   ~Timer() { }

   void start()
   {
      _t_start = std::chrono::steady_clock::now();
   }

   unsigned int elapsed_ms()
   {
      std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
      return std::chrono::duration_cast<std::chrono::milliseconds>(t_end - _t_start).count();
   }

   double elapsed_s()
   {
      return static_cast<double>(this->elapsed_ms()) / 1000.0;
   }

   void elapsed_ms_cout()
   {
      std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
      std::cout << _msg << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - _t_start).count()<< " ms" << std::endl;
   }

private:
   std::chrono::steady_clock::time_point _t_start;
   std::string _msg;
};


class Timer_auto_ms {
public:
   Timer_auto_ms(std::string msg = std::string("")) : _msg(msg)
   {
      _t_start = std::chrono::steady_clock::now();
   }

   ~Timer_auto_ms()
   {
      std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
      std::cout << _msg << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - _t_start).count()<< " ms" << std::endl;
   }

private:
   std::chrono::steady_clock::time_point _t_start;
   std::string _msg;
};


class Timer_auto_us {
public:
   Timer_auto_us(std::string msg = std::string("")) : _msg(msg)
   {
      _t_start = std::chrono::steady_clock::now();
   }

   ~Timer_auto_us()
   {
      std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
      std::cout << _msg << std::chrono::duration_cast<std::chrono::microseconds>(t_end - _t_start).count()<< " us" << std::endl;
   }

private:
   std::chrono::steady_clock::time_point _t_start;
   std::string _msg;
};

}

#endif /* TIMER_H_ */
