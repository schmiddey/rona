/*
 * ObstacleEventHandler.h
 *
 *  Created on: 29.02.2016
 *      Author: m1ch1
 */

#ifndef OBSTACLEEVENTHANDLER_H_
#define OBSTACLEEVENTHANDLER_H_

#include <iostream>
#include <string>
#include <memory>

#include <rona_lib/Map/map_types.h>
#include <rona_lib/Utility.h>

#include <rona_msgs/Obstacle.h>
#include <std_msgs/String.h>

namespace rona
{

/**
 * Singelton
 *
 */
class ObstacleEventHandler
{
public:

   static std::shared_ptr<ObstacleEventHandler> getInstance()
   {
      static std::shared_ptr<ObstacleEventHandler> instance(new ObstacleEventHandler);
      return instance;
   }
   virtual ~ObstacleEventHandler() { }

   void newObstacle_event(map::Polygon ob, unsigned int id)
   {
      ROS_INFO_STREAM("ObstacleHandler-> newObstacle Event(polygon: " << ob << ", id: " << id << ")");
      rona_msgs::Obstacle msg;
      msg.header.frame_id = _map_frame;
      msg.header.stamp = ros::Time();
      msg.polygon = Utility::toRosPolygon(ob,_map_frame);
      msg.identifier = std::to_string(id);
      _pubAddOb.publish(msg);
   }

   //void renewObstacle_event(map::Polygon ob, unsigned int id)
   //{
   //
   //}

   void dropObstacle_event(unsigned int id)
   {
      ROS_INFO_STREAM("ObstacleHandler-> dropObstacle Event(id: " << id << ")");
      std_msgs::String msg;
      msg.data = std::to_string(id);
      _pubRmOb.publish(msg);
   }
private: // funktions:

   ObstacleEventHandler()
   {
      //rosParam
      ros::NodeHandle privNh("~");
      std::string pub_add_obstacle_topic;
      std::string pub_rm_obstacle_topic;
      std::string map_frame;
      //double double_value;
      //int int_val;

      //privNh.param("string_value",string_value,std::string("std_value"));
      //privNh.param<double>("double_value",double_value, 12.34);
      //privNh.param<int>("int_val",int_val, 1234);
      privNh.param("pub_add_obstacle_topic", pub_add_obstacle_topic, std::string("add_obstacle"));
      privNh.param("pub_rm_obstacle_topic",  pub_rm_obstacle_topic,  std::string("rm_obstacle" ));
      privNh.param("map_frame",              map_frame,              std::string("map"));

      _map_frame = map_frame;
      //init publisher
      //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
      _pubAddOb = _nh.advertise<rona_msgs::Obstacle>(pub_add_obstacle_topic, 1);
      _pubRmOb  = _nh.advertise<std_msgs::String>(pub_rm_obstacle_topic, 1);
   }

   ObstacleEventHandler(const ObstacleEventHandler& o) = delete;
   ObstacleEventHandler(ObstacleEventHandler&& o) = delete;
   ObstacleEventHandler& operator=(const ObstacleEventHandler& o) = delete;
   ObstacleEventHandler& operator=(ObstacleEventHandler&& o) = delete;

private:
   ros::NodeHandle _nh;

   ros::Publisher _pubAddOb;  //adding obstacle (ob) to planner
   ros::Publisher _pubRmOb;   //removing obstacle from planner

   std::string _map_frame;
};


} /* namespace rona */

#endif /* OBSTACLEEVENTHANDLER_H_ */
