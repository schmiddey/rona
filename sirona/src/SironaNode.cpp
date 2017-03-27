
#include "SironaNode.h"

#include <opencv2/opencv.hpp>

#include <rona_lib/Map/GridMap_extended.h>
#include <rona_lib/Map/GridMap_extended2.h>
#include <rona_lib/Map/GridMap_reduced.h>


SironaNode::SironaNode()
{
   ROS_INFO("Started sirona_node");
    //rosParam
    ros::NodeHandle privNh("~");
    //std::string string_value;
    //double double_value;
    //int int_val;
    std::string sub_map_topic   ;
    std::string sub_target_topic;
    std::string sub_add_obstacle_topic;
    std::string sub_rm_obstacle_topic;
    std::string pub_path_topic  ;
    std::string pub_state_topic;
    std::string pub_grid_topic;
    std::string pub_grid_ob_topic;
    std::string pub_move_ctrl_topic;
    std::string map_frame       ;
    std::string robot_frame     ;

    double robot_radius;
    double dt_radius;
    double obstacle_react_dist;
    double obstacle_block_time;

    double free_robot_pos_factor;

    //privNh.param("string_value",string_value,std::string("std_value"));
    //privNh.param<double>("double_value",double_value, 12.34);
    //privNh.param<int>("int_val",int_val, 1234);
    privNh.param        ("sub_map_topic"          ,    sub_map_topic   ,          std::string("map"                   ));
    privNh.param        ("sub_target_topic"       ,    sub_target_topic,          std::string("/move_base_simple/goal"));
    privNh.param        ("sub_add_obstacle_topic" ,    sub_add_obstacle_topic,    std::string("add_obstacle"          ));
    privNh.param        ("sub_rm_obstacle_topic"  ,    sub_rm_obstacle_topic,     std::string("rm_obstacle"           ));
    privNh.param        ("pub_state_topic"        ,    pub_state_topic,           std::string("navigation/state"      ));
    privNh.param        ("pub_path_topic"         ,    pub_path_topic  ,          std::string("path"                  ));
    privNh.param        ("pub_grid_topic"         ,    pub_grid_topic  ,          std::string("navigation/map"        ));
    privNh.param        ("pub_grid_ob_topic"      ,    pub_grid_ob_topic  ,       std::string("navigation/obstacles"  ));
    privNh.param        ("pub_move_ctrl_topic"    ,    pub_move_ctrl_topic  ,     std::string("move/ctrl"  ));
    privNh.param        ("map_frame"              ,    map_frame       ,          std::string("map"                   ));
    privNh.param        ("robot_frame"            ,    robot_frame     ,          std::string("base_footprint"        ));
    privNh.param<double>("robot_radius"           ,    robot_radius,              0.35 ); //[m]
    privNh.param<double>("dt_radius"              ,    dt_radius,                 0.2  ); //[m]
    privNh.param<double>("free_robot_pos_factor"  ,    free_robot_pos_factor,     0.5  ); ///< not save user must be knowing what he is doing... advanced
    privNh.param<double>("obstacle_react_dist"    ,    obstacle_react_dist,       1.5  ); //[m]
    privNh.param<double>("obstacle_block_time"    ,    obstacle_block_time,       2    ); //[s]

    _map_frame = map_frame;
    _robot_frame = robot_frame;

    _robot_radius = robot_radius;
    _dt_radius    = dt_radius;

    _obstacle_react_dist = obstacle_react_dist;
    _durationMove = ros::Duration(obstacle_block_time);

    if(std::abs(free_robot_pos_factor) > 1)
       free_robot_pos_factor = 1;
    _free_robot_pos_factor = std::abs(free_robot_pos_factor);

    _gotMap = false;

    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
    _pubPath      = _nh.advertise<nav_msgs::Path>(pub_path_topic, 2);
    _pubState     = _nh.advertise<rona_msgs::State>(pub_state_topic, 2);
    _pubGrid      = _nh.advertise<nav_msgs::GridCells>(pub_grid_topic, 1);
    _pubGridOb    = _nh.advertise<nav_msgs::GridCells>(pub_grid_ob_topic, 1);
    _pubMoveCtrl  = _nh.advertise<rona_msgs::NodeCtrl>(pub_move_ctrl_topic, 100);

    //inti subscriber
    //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
    _subMap         = _nh.subscribe(sub_map_topic, 1, &SironaNode::sub_mapCallback, this);
    _subTarget      = _nh.subscribe(sub_target_topic, 2, &SironaNode::sub_targetCallback, this);
    _subAddObstacle = _nh.subscribe(sub_add_obstacle_topic, 2, &SironaNode::sub_addObCallback, this);
    _subRmObstacle  = _nh.subscribe(sub_rm_obstacle_topic, 2, &SironaNode::sub_rmObCallbach, this);

    _timerMove = _nh.createTimer(_durationMove, &SironaNode::timer_moveCallback, this, true, false);
    _timerMove.stop();//started in obstacle callback

    _state = State::MOVING;

    _eventHandler = rona::SironaEvent::getInstance();
}

SironaNode::~SironaNode()
{

}

void SironaNode::start(const double duration)
{
   //init timer
   //_loopTimer = _nh.createTimer(ros::Duration(duration), &SironaNode::timer_loopCallback, this);
   ros::spin();
}

//void SironaNode::timer_loopCallback(const ros::TimerEvent& e)
//{
//   if(!ros::ok())
//   {
//      ROS_INFO("ros::ok() -> false");
//      exit(EXIT_SUCCESS);
//   }
//
//   //do loop stuff here...
//   ROS_INFO("blabla");
//   //pub...
//   //_pub.publish(..)
//
//   std::weak_ptr<rona::map::Map> tmp;
//   _planner.setMap(tmp);
//
//
//}

void SironaNode::timer_moveCallback(const ros::TimerEvent& e)
{
   ROS_INFO("----------DEBUG timer_moveCallback()");
   _timerMove.stop();
   _eventHandler->fireMoveContinue_event();
}

void SironaNode::debug_save_as_img(std::string file,
                                   std::shared_ptr<rona::map::Grid> grid,
                                   std::vector<rona::map::Node> path)
{
   if(!grid)
   {
      ROS_ERROR("debug_save_as_img: no valid grid given...");
      return;
   }
   //debug: draw path in cv::Mat
   cv::Mat cvmap = grid->toCvMat();

   cv::cvtColor(cvmap, cvmap, CV_GRAY2RGB);

   if(path.size())
   {
      rona::map::Pixel p = grid->toPixel(path[0].pos);
      cv::Point old;
      cv::Point curr;
      old.x = p.x;
      old.y = p.y;
      //std::cout << "PixelPath: " << std::endl;
      for(unsigned int i = 1; i < path.size(); ++i)
      {
         p = grid->toPixel(path[i].pos);
//         std::cout << p;// << " ,(" << path[i].pos.x << "," << path[i].pos.y << ")" <<std::endl;
//         ROS_INFO(", (%f, %f)", path[i].pos.x, path[i].pos.y);
         curr.x = p.x;
         curr.y = p.y;
         cv::line(cvmap, curr, old, cv::Scalar(0,255,0),2);
         old = curr;
      }

   }

   cv::flip(cvmap, cvmap, -1);
   cv::flip(cvmap, cvmap, 1);
   cv::imwrite(file.c_str(),cvmap);
}


std::shared_ptr<rona::map::GridMap> SironaNode::doMapOperations(const std::weak_ptr<rona::map::GridMap> map_raw, const rona::map::Point2D robot_pos)
{
   if(map_raw.expired())
   {
      ROS_ERROR("Given Map in doMapOperations is expired... will exit");
      exit(EXIT_FAILURE);
   }

   rona::Timer_auto_ms timer("MapOperations: ");
   //copy new map...
   std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap(map_raw.lock()));

   //for visual inflation
   std::shared_ptr<rona::map::Grid> grid_raw(new rona::map::Grid(map->getGrid()));
   rona::map::Operations::binarize(grid_raw, 0,10,0,255);
   //this->debug_save_as_img("/tmp/grid_raw.png", grid_raw);
   //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_extended(map_raw.lock()));
   //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_extended2(map_raw.lock()));
   //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_reduced(map_raw.lock()));

   //this->debug_save_as_img("/tmp/map_raw.png", map->getGrid());

   //draw obstacles in map;
   //with dt on ob
   //--
   for(auto& e : _obstacles)
   {
      rona::map::Operations::drawFilledPolygon(map->getGrid(), e.second, 120);
   }
   //--

   //vis
   std::shared_ptr<rona::map::Grid> grid_map(new rona::map::Grid(map->getGrid()));
   rona::map::Operations::binarize(grid_map, 0, 10, 0, 255);
   std::shared_ptr<rona::map::Grid> grid_diff_ob = rona::map::Operations::diff(grid_raw, grid_map);
   _pubGridOb.publish(grid_diff_ob->toGridCells(200,255,_map_frame));

   ///@note dont inflate unknown area
   rona::map::Operations::inflateCirc(map->getGrid(), 10, 127, _robot_radius);
   rona::map::Operations::binarize(map->getGrid(), 0, 10, 0, 255);

   //for visual inflation
   std::shared_ptr<rona::map::Grid> grid_diff = rona::map::Operations::diff(grid_raw, map->getGrid());
   //this->debug_save_as_img("/tmp/grid_map.png", map->getGrid());
   //this->debug_save_as_img("/tmp/grid_diff.png", grid_diff);
   _pubGrid.publish(grid_diff->toGridCells(200,255,_map_frame));

   //free RobotPose if robot new target is out of range
   if(_free_robot_pos_factor > 0.001)
      rona::map::Operations::drawFilledCircle(map->getGrid(), robot_pos, _robot_radius * _free_robot_pos_factor, 0);

   //this->debug_save_as_img("/tmp/map.png", map->getGrid());

   //dt
   std::shared_ptr<rona::map::Grid> costmap(new rona::map::Grid(map->getGrid()));
   rona::map::Operations::distnaceTransformCirc(costmap, _dt_radius, 255);

   //this->debug_save_as_img("/tmp/map_dt.png", costmap);

   map->addCostMap("dt_map", costmap);

   return map;
}

rona::map::Path SironaNode::computePath(const std::weak_ptr<rona::map::GridMap> map, const rona::map::Point2D start, rona::map::Point2D end)
{
   rona::Timer_auto_ms timer("Planning: ");
   //do planning
   _planner.setMap(map);

   rona::map::Node n_start;
   rona::map::Node n_end;
   //todo check normaly only id and pos needed
   n_start.id = _map->getGrid()->toIdx(start);
   n_start.pos = start;
   n_end.id = _map->getGrid()->toIdx(end);
   n_end.pos = end;

   rona::map::Path path = _planner.computePath(n_start, n_end);
   return path;
}

void SironaNode::sub_mapCallback(const nav_msgs::OccupancyGrid& msg)
{
   if(!_gotMap)
   {
      ROS_INFO("Got Map");
      _gotMap = true;

      std::cout << "map.w" << msg.info.width << std::endl;
      std::cout << "map.h" << msg.info.height << std::endl;
      std::cout << "map.res" << msg.info.resolution << std::endl;
      std::cout << "map.x" << msg.info.origin.position.x << std::endl;
      std::cout << "map.y" << msg.info.origin.position.y << std::endl;
   }

   std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap(msg));

   _map = map;
   //_planner.setMap(_map);
}



void SironaNode::sub_targetCallback(const geometry_msgs::PoseStamped& msg)
{
   if(!_gotMap)
   {
      ROS_WARN("No Map given until now.... ignore Target");
      return;
   }
   rona::Timer_auto_ms timer("Overall sub_targetCallback: ");
   ROS_INFO("sub_targetCallback() got target");

   //get robot pose
   rona::map::Point2D start = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);
   rona::map::Point2D end = rona::Utility::toPoint2D(msg);

   std::cout << "start: " << start << std::endl;
   std::cout << "end: " << end << std::endl;
   std::cout << "startPixel: " << _map->getGrid()->toPixel(start) << std::endl;
   std::cout << "endPixel: " <<_map->getGrid()->toPixel(end) << std::endl;

   rona::map::Path path = this->computePath(this->doMapOperations(_map,start), start, end);

   if(!path.size())
   {
      ROS_INFO("--- NO Path found");
   }
   ROS_INFO("Path found: num nodes: %d", (int)path.size());

   //std::cout << path << std::endl;

   //pub path;
   nav_msgs::Path ros_path = rona::Utility::toRosPath(path);
   //set target orientation.
   ros_path.poses[ros_path.poses.size()-1].pose.orientation = msg.pose.orientation;
   _pubPath.publish(ros_path);
   rona_msgs::NodeCtrl node_ctrl;
   node_ctrl.cmd = node_ctrl.START;
   node_ctrl.cmd_str = "START";
   _pubMoveCtrl.publish(node_ctrl);

   //debug
   //this->debug_save_as_img("/tmp/map_path.png", _map->getGrid(), path);
}



void SironaNode::sub_addObCallback(const rona_msgs::Obstacle& msg)
{
   try {
      _obstacles.at(msg.identifier) = rona::Utility::toPolygon(msg.polygon);
   } catch (std::out_of_range& e) {
      _obstacles.insert(std::make_pair(msg.identifier, rona::Utility::toPolygon(msg.polygon)));
   }

   rona::map::Polygon p = rona::Utility::toPolygon(msg.polygon);
   rona::map::Point2D c = rona::map::Operations::computeCentroid(p);
   rona::map::Point2D robot_pos = rona::Utility::getTransformPoint2D(_tf_listnener, _map_frame, _robot_frame);
   double dist = rona::map::Operations::computeDistance(robot_pos, c);

   ///@note for now just dist... later maybe polygon...
   if(dist < _obstacle_react_dist)
   {
      //pause + timer +  for blocking... -> pauseEvent
      _eventHandler->fireMovePause_event();
      ROS_INFO("--------------Debug -> start Timer");
      _timerMove.setPeriod(_durationMove); //restart timer..
      _timerMove.start();
   }

   if(_state == State::MOVING || _state == State::PAUSED)
   {
      ROS_INFO("Replan path - new Obstacle");
      //replan
      _planner.setMap(this->doMapOperations(_map, robot_pos));
      rona::map::Node n_start;
      n_start.id = _map->getGrid()->toIdx(robot_pos);
      n_start.pos = robot_pos;
      rona::map::Path path = _planner.recomputePath(n_start);

      nav_msgs::Path rosPath = rona::Utility::toRosPath(path, _map_frame);
      _pubPath.publish(rosPath);
   }
}



void SironaNode::sub_rmObCallbach(const std_msgs::String& msg)
{
   _obstacles.erase(msg.data);

   if(_state == State::MOVING || _state == State::PAUSED)
   {
      ROS_INFO("Replan path - remove Obstacle");
      //replan
      rona::map::Point2D robot_pos = rona::Utility::getTransformPoint2D(_tf_listnener, _map_frame, _robot_frame);
      _planner.setMap(this->doMapOperations(_map, robot_pos));
      rona::map::Node n_start;
      n_start.id = _map->getGrid()->toIdx(robot_pos);
      n_start.pos = robot_pos;
      rona::map::Path path = _planner.recomputePath(n_start);

      nav_msgs::Path rosPath = rona::Utility::toRosPath(path, _map_frame);
      _pubPath.publish(rosPath);
   }
}



//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sirona_node");
    ros::NodeHandle nh("~");

    SironaNode node;
    node.start(0.1);

}
