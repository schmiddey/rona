
#ifndef SIRONANODE_H_
#define SIRONANODE_H_

#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <stdexcept>
#include <vector>

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


#include <rona_msgs/Obstacle.h>
#include <rona_msgs/State.h>
#include <rona_msgs/NodeCtrl.h>


#include "Event/SironaEvent.h"



class SironaNode
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubPath;
    ros::Publisher _pubState;
    ros::Publisher _pubGrid;
    ros::Publisher _pubGridOb;
    ros::Publisher _pubMoveCtrl;
    ros::Subscriber _subMap;
    ros::Subscriber _subTarget;
    ros::Subscriber _subAddObstacle;
    ros::Subscriber _subRmObstacle;

    //ros::Timer _loopTimer;
    ros::Timer _timerMove;
    ros::Duration _durationMove;

    rona::planner::AStar _planner;

    tf::TransformListener _tf_listnener;

    std::string _map_frame;
    std::string _robot_frame;

    double _robot_radius;
    double _dt_radius;

    double _free_robot_pos_factor; ///< unsave in some cases... user must know what he is doing
    double _obstacle_react_dist;

    bool _gotMap;

    std::map<std::string, rona::map::Polygon> _obstacles;

    std::shared_ptr<rona::SironaEvent> _eventHandler;
    std::shared_ptr<rona::map::GridMap> _map;

    enum class State{
       MOVING = 0,
       PAUSED,
       ARRIVED,
       ABORTED,
       ERROR
    };

    State _state;

public:
    SironaNode();
    virtual ~SironaNode();

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

private:    //functions

    /**
     * @brief loop callback function
     *
     * @param e
     */
    //void timer_loopCallback(const ros::TimerEvent& e);
    void timer_moveCallback(const ros::TimerEvent& e);

    void debug_save_as_img(std::string file,
                          std::shared_ptr<rona::map::Grid> grid,
                          std::vector<rona::map::Node> path = std::vector<rona::map::Node>(0));

    /**
     * @brief does all MapOperations on a new/copied map
     * @param map_raw
     * @return
     */
    std::shared_ptr<rona::map::GridMap> doMapOperations(const std::weak_ptr<rona::map::GridMap> map_raw, const rona::map::Point2D robot_pos);
    rona::map::Path computePath(const std::weak_ptr<rona::map::GridMap> map, const rona::map::Point2D start, rona::map::Point2D end);

    void sub_mapCallback(const nav_msgs::OccupancyGrid& msg);
    void sub_targetCallback(const geometry_msgs::PoseStamped& msg);
    void sub_addObCallback(const rona_msgs::Obstacle& msg);
    void sub_rmObCallbach(const std_msgs::String& msg);

};

#endif /* SIRONANODE_H_ */
