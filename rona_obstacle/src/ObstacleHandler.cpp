
#include "ObstacleHandler.h"

ObstacleHandler::ObstacleHandler()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_marker_topic;
    std::string sub_obstacle_topic;
    std::string map_frame;
    double robot_radius;
    double duration_obstacle_valid;
    //int int_val;

    //privNh.param("string_value",string_value,std::string("std_value"));
    //privNh.param<double>("double_value",double_value, 12.34);
    //privNh.param<int>("int_val",int_val, 1234);
    privNh.param        ("pub_marker_topic"        ,   pub_marker_topic,         std::string("obstacle_fix_marker"));
    privNh.param        ("sub_obstacle_topic"      ,   sub_obstacle_topic,       std::string("obstacle"           ));
    privNh.param        ("map_frame"               ,   map_frame,                std::string("map"                ));
    privNh.param<double>("robot_radius"            ,   robot_radius,             0.4 );
    privNh.param<double>("duration_obstacle_valid" ,   duration_obstacle_valid,  60  );

    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
   _pubMarker  = _nh.advertise<visualization_msgs::MarkerArray>(pub_marker_topic, 1);
    //inti subscriber
   _sub = _nh.subscribe(sub_obstacle_topic,1 ,&ObstacleHandler::subCallbackObstacle, this);

   _eventHandler = rona::ObstacleEventHandler::getInstance();

   _robot_radius = robot_radius;
   _duration_valid = duration_obstacle_valid;
   _map_frame = map_frame;
}

ObstacleHandler::~ObstacleHandler()
{
}

void ObstacleHandler::start(const double duration)
{
   //init timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &ObstacleHandler::timerLoop_callback, this);
   ros::spin();
}

void ObstacleHandler::timerLoop_callback(const ros::TimerEvent& e)
{
//   if(!ros::ok())
//   {
//      ROS_INFO("ros::ok() -> false");
//      exit(EXIT_SUCCESS);
//   }

   //do loop stuff here...
   //ROS_INFO("blabla");
   //pub...
   //_pub.publish(..)
   _marker.markers.clear();
   std_msgs::ColorRGBA color;
   color.r = 1;
   color.g = 0;
   color.b = 0;
   color.a = 0.5;

   for(auto& e : _obstacleHandles)
   {
      visualization_msgs::Marker m = rona::Utility::toRosMarker(e->getObstacleScaled(),color,e->getId(), _map_frame);
      if(!e->isValid())
         m.action = visualization_msgs::Marker::DELETE;

      _marker.markers.push_back(m);
   }
   _pubMarker.publish(_marker);

}

void ObstacleHandler::subCallbackObstacle(const geometry_msgs::PolygonStamped& msg)
{
   //rona::Timer_auto_us timer("ObstacleHandler prove obstacle: ");
   //ROS_INFO("bla");
   rona::map::Polygon polygon = rona::Utility::toPolygon(msg);
   std::vector<rona::ObstacleInfo> matched;

   for(auto& e : _obstacleHandles)
   {
      rona::ObstacleInfo info = e->isSameObstacle(polygon);
      if(info.state == rona::ObstacleState::EXISTING || info.state == rona::ObstacleState::RENEW)
      {
         matched.push_back(info);
      }
   }

   if(matched.empty())
   {
      //new Obstacle..
      unsigned int id = _obstacleHandles.size();

      rona::map::Rect2D rect = rona::map::Operations::getBoundingRect(polygon);
      double w = (rect.w > rect.h ? rect.w : rect.h);
      if(w < 0.05)
         w = 0.05;

      double scale = ( (2 * _robot_radius) / rect.w ) + 1;
      std_msgs::ColorRGBA c;
      c.b = 1;
      c.a = 1;
      _obstacleHandles.push_back(std::unique_ptr<rona::ObstacleHandle>(new rona::ObstacleHandle(polygon, id, scale, _duration_valid)));

      //fire event
      _eventHandler->newObstacle_event(polygon, id);
      return;
   }

   double min_fit = std::numeric_limits<double>::max();
   unsigned int min_id = 0;
   for(auto& e : matched)
   {
      if(e.fitAccuracy < min_fit)
      {
         min_fit = e.fitAccuracy;
         min_id = e.id;
      }
   }

   if(_obstacleHandles[min_id]->isValid())
   {//do nothing
      return;
   } else
   {
      ///@todo
      /// after some renew events in same obstacle set ob as permanent obstacle
      //pub newOb event
      _obstacleHandles[min_id]->setValid(true);
      ROS_INFO("RenewCnt: %d", _obstacleHandles[min_id]->getRenewCnt());
      _eventHandler->newObstacle_event(_obstacleHandles[min_id]->getObstacle(), min_id);

   }


//
//   else if(matchedcnt == 1)
//   {//only one matched ok...
//      if(matchedId_renew.size())
//      {//renewhttp://community.arduboy.com/t/contest-nyan-cat-for-arduboy/868
//      }
//   } else
//   {//more than one matched
//      //
//      ROS_WARN("detected in more then one handle do nothing for now");
//   }

}



//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_obstacle_handler_node");
    ros::NodeHandle nh("~");

    ObstacleHandler node;
    node.start(0.5);

}

