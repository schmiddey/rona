
#ifndef SIRONASM_H_
#define SIRONASM_H_

#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <stdexcept>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>

#include <rona_lib/Map/GridMap.h>
#include <rona_lib/Map/Grid.h>
#include <rona_lib/Utility.h>
#include <rona_lib/Map/Operations.h>
#include <rona_lib/Timer.h>

#include <rona_msgs/Obstacle.h>
#include <rona_msgs/NodeCtrl.h>
#include <rona_msgs/State.h>



/**
 * @todo Add proving current map for path collitoins ... (as parameter yes or no)
 */
class SironaSM
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubRelayTarget;
    ros::Publisher _pubRelayAddOb;
    ros::Publisher _pubRelayRmOb;
    ros::Publisher _pubReplan;
    ros::Publisher _pubPathMove;
    ros::Publisher _pubState;
    ros::Publisher _pubMoveCtrl;

    ros::Subscriber _subTarget;
    ros::Subscriber _subAddOb;
    ros::Subscriber _subRmOb;
    ros::Subscriber _subPath;
    ros::Subscriber _subStateMove;
    ros::Subscriber _subObstacleGrid;

    ros::Timer _loopTimer;

    ros::Timer _moveTimer;
    ros::Duration _stopDuration;


    tf::TransformListener _tf_listnener;

    std::string _map_frame;
    std::string _robot_frame;

    rona::map::Polygon _reactPolygon;

    std::map<std::string, rona::map::Polygon> _obstacles;

    //std::shared_ptr<rona::SironaMoveHandler> _moveHandler;

    enum class PathType{
       NORMAL = 0,
       REPLAN
    };

    std::queue<PathType> _pathTypeQueue;

    enum class State{
       IDLE = 0,
       MOVING,
       PAUSED,
       ARRIVED,
       ABORTED,
       UNREACHABLE,
//       BLOCKED,   //when could not plan

    };

    State _state;

    bool _oldMoveState;

    rona_msgs::State _state_msg;

public:
    SironaSM();
    virtual ~SironaSM();

    /**
     * @fn void start(const double duration = 0.1)
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
    void timerLoop_callback(const ros::TimerEvent& e);
    void timer_moveCallback(const ros::TimerEvent& e);


    void sub_targetCallback(const geometry_msgs::PoseStamped& msg);
    void sub_addObCallback(const rona_msgs::Obstacle& msg);
    void sub_rmObCallback(const std_msgs::String& msg);
    void sub_pathCallback(const nav_msgs::Path& msg);
    void sub_stateMoveCallback(const std_msgs::Bool& msg);
    void sub_obstacleGrid(const nav_msgs::GridCells& msg);

    void startMove();
    void stopMove();
    void pauseMove();
    void pause_auto_unpauseMove();
    void unpauseMove();

    void setState(const State state);
};

#endif /* SIRONASM_H_ */
