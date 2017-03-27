/*
 * ObstacleEventHandler.h
 *
 *  Created on: 29.02.2016
 *      Author: m1ch1
 */

#ifndef SIRONAEVENT_H_
#define SIRONAEVENT_H_

#include <iostream>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace rona
{

/**
 * Singelton
 */
class SironaEvent
{
public:

   static std::shared_ptr<SironaEvent> getInstance()
   {
      static std::shared_ptr<SironaEvent> instance(new SironaEvent);
      return instance;
   }
   virtual ~SironaEvent() { }


   void fireMovePause_event()
   {
      std_msgs::Bool msg;
      msg.data = true;
      _pubPauseMove.publish(msg);
   }

   void fireMoveContinue_event()
   {
      std_msgs::Bool msg;
      msg.data = false;
      _pubPauseMove.publish(msg);
   }

private: // funktions:

   SironaEvent()
   {
      //rosParam
      ros::NodeHandle privNh("~");
      //std::string map_frame;
      //double double_value;
      //int int_val;
      std::string pub_move_pause_topic;


      privNh.param("pub_move_pause_topic",             pub_move_pause_topic,      std::string("path_control/pause"));

      //privNh.param("string_value",string_value,std::string("std_value"));
      //privNh.param<double>("double_value",double_value, 12.34);
      //privNh.param<int>("int_val",int_val, 1234);

      //init publisher
      //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
      _pubPauseMove = _nh.advertise<std_msgs::Bool>(pub_move_pause_topic, 2);
   }

   SironaEvent(const SironaEvent& o) = delete;
   SironaEvent(SironaEvent&& o) = delete;
   SironaEvent& operator=(const SironaEvent& o) = delete;
   SironaEvent& operator=(SironaEvent&& o) = delete;

private:
   ros::NodeHandle _nh;

   ros::Publisher _pubPauseMove;

};


} /* namespace rona */

#endif /* SIRONAEVENT_H_ */
