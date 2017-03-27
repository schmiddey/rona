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
#include <rona_msgs/NodeCtrl.h>

namespace rona
{

/**
 * Singelton
 */
class SironaMoveHandler
{
public:

   static std::shared_ptr<SironaMoveHandler> getInstance()
   {
      static std::shared_ptr<SironaMoveHandler> instance(new SironaMoveHandler);
      return instance;
   }
   virtual ~SironaMoveHandler() { }

   void start()
   {
      rona_msgs::NodeCtrl ctrl_msg;
      ctrl_msg.cmd = ctrl_msg.START;
      ctrl_msg.cmd_str = "START";
      _pubMoveCtrl.publish(ctrl_msg);
   }

   void stop()
   {
      rona_msgs::NodeCtrl ctrl_msg;
      ctrl_msg.cmd = ctrl_msg.STOP;
      ctrl_msg.cmd_str = "STOP";
      _pubMoveCtrl.publish(ctrl_msg);
   }

   void pause()
   {
      rona_msgs::NodeCtrl msg;
      msg.cmd = msg.PAUSE;
      msg.cmd_str = "PAUSE";
      _pubMoveCtrl.publish(msg);
   }

   void pause_auto_unpause()
   {
      this->pause();
      //start timer for auto unpause
      _moveTimer.setPeriod(_stopDuration); //restart in case it is already running
      _moveTimer.start();
   }

   void unpause()
   {
      rona_msgs::NodeCtrl msg;
      msg.cmd = msg.CONTINUE;
      msg.cmd_str = "CONTINUE";
      _pubMoveCtrl.publish(msg);
   }

private: // funktions:

   SironaMoveHandler()
   {
      //rosParam
      ros::NodeHandle privNh("~");
      //std::string map_frame;
      //double double_value;
      //int int_val;
      std::string pub_move_ctrl        ;
      double      obstacle_block_time       ;



      //init publisher
      _pubMoveCtrl    = _nh.advertise<rona_msgs::NodeCtrl>(pub_move_ctrl, 100);
      _stopDuration = ros::Duration(obstacle_block_time);

      _moveTimer = _nh.createTimer(_stopDuration, &SironaMoveHandler::timer_moveCallback, this, true, false);
      _moveTimer.stop();
   }

   SironaMoveHandler(const SironaMoveHandler& o) = delete;
   SironaMoveHandler(SironaMoveHandler&& o) = delete;
   SironaMoveHandler& operator=(const SironaMoveHandler& o) = delete;
   SironaMoveHandler& operator=(SironaMoveHandler&& o) = delete;




   void timer_moveCallback(const ros::TimerEvent& e)
   {
      ROS_INFO("TimerCallback -> unpause PathController");
      _moveTimer.stop();
      //continue moving
      this->unpause();
   }

private:
   ros::NodeHandle _nh;

   ros::Publisher _pubMoveCtrl;

   ros::Timer _moveTimer;
   ros::Duration _stopDuration;


};


} /* namespace rona */

#endif /* SIRONAEVENT_H_ */
