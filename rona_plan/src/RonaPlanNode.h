/**
 * @file   RonaPlanNode.h
 * @author Michael Schmidpeter
 * @date   2018-03-15
 * @brief  todo
 * 
 * PROJECT: rona
 * @see https://github.com/schmiddey/rona
 */

#ifndef RONAPLANNODE_H_
#define RONAPLANNODE_H_

#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <stdexcept>
#include <vector>

//HACK
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <rona_lib/Map/GridMap.h>
#include <rona_lib/Map/Grid.h>
#include <rona_lib/Planner/AStar.h>
#include <rona_lib/Utility.h>
#include <rona_lib/Map/Operations.h>
#include <rona_lib/Timer.h>

//msg
#include <rona_msgs/Obstacle.h>
//srv
#include <rona_msgs/PlanPath.h>
#include <rona_msgs/PlanPathMultiple.h>

class RonaPlanNode
{
public:
  RonaPlanNode();
  virtual ~RonaPlanNode();

  /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] duration  ->  duration of the working loop in [s] -> 1/rate
     *
     *
     * @return  void
     */
  void start(const double duration = 0.1);

private: //functions
  void debug_save_as_img(std::string file,
                         std::shared_ptr<rona::map::Grid> grid,
                         std::vector<rona::map::Node> path = std::vector<rona::map::Node>(0));

  /**
     * @brief does all MapOperations on a new/copied map
     * @param map_raw
     * @return
     */
  std::shared_ptr<rona::map::GridMap> doMapOperations(const std::weak_ptr<rona::map::GridMap> map_raw);

  /**
   * @brief frees robot if curr pose is occupied
   * 
   * @param map 
   * @param robot_pos 
   * @return true if robot is freed, false if robot doesent need to be freed
   */
  bool freeRobot(std::weak_ptr<rona::map::GridMap> map, const rona::map::Point2D robot_pos);

  rona::map::Path computePath(const std::weak_ptr<rona::map::GridMap> map, const rona::map::Point2D start, rona::map::Point2D end);

  void sub_mapCallback(const nav_msgs::OccupancyGrid &msg);
  void sub_targetCallback(const geometry_msgs::PoseStamped &msg);
  void sub_replanCallback(const std_msgs::Bool &msg);
  void sub_addObCallback(const rona_msgs::Obstacle &msg);
  void sub_rmObCallbach(const std_msgs::String &msg);

  bool srv_plan_callback(rona_msgs::PlanPathRequest& req,
                         rona_msgs::PlanPathResponse& res);

  bool srv_plan_multiple_callback(rona_msgs::PlanPathMultipleRequest& req,
                                  rona_msgs::PlanPathMultipleResponse& res);

private: //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubPath;
  ros::Publisher _pubGrid;
  ros::Publisher _pubGridOb;
  ros::Publisher _pubMapCost;
  ros::Publisher _pubMapResize;

  ros::Subscriber _subMap;
  ros::Subscriber _subTarget;
  ros::Subscriber _subReplan;
  ros::Subscriber _subAddObstacle;
  ros::Subscriber _subRmObstacle;

  ros::ServiceServer _srv_plan_path_multiple;
  ros::ServiceServer _srv_plan_path;

  //ros::Timer _loopTimer;

  rona::planner::AStar _planner;

  tf::TransformListener _tf_listnener;

  std::string _map_frame;
  std::string _robot_frame;

  double _robot_radius;
  double _dt_radius;

  double _free_robot_pos_factor; ///< unsave in some cases... user must know what he is doing

  unsigned int _map_scale;

  bool _continuous_map_operation;

  unsigned int _map_cnt;
  unsigned int _map_op_cnt;

  bool _replanRdy;

  std::map<std::string, rona::map::Polygon> _obstacles;

  std::shared_ptr<rona::map::GridMap> _map_raw;
  std::shared_ptr<rona::map::GridMap> _map;
  std::shared_ptr<rona::map::GridMap> _map_op;

  rona::map::Point2D _lastTarget;
  geometry_msgs::Quaternion _lastTarget_orientation;

};

#endif //RONAPLANNODE_H_
