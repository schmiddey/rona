
#ifndef PATHREPEAT_H_
#define PATHREPEAT_H_

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


#include <rona_msgs/PlanPath.h>
#include <rona_msgs/NodeCtrl.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <rona_lib/Utility.h>
#include <rona_lib/Map/Operations.h>


enum class NodeState{
   IDLE = 0,
   WAIT_FOR_START,
   MOVING,
   STOP
};

class PathRepeat
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubPath;
    ros::Publisher _pubState;
    ros::Publisher _pubMoveCtrl;

    ros::Subscriber _subSetEndPoint;
    ros::Subscriber _subStart;
    ros::Subscriber _subPathControlState;

    ros::ServiceClient _srv_plan;
    ros::ServiceClient _srv_move_sw_reverse;

    tf::TransformListener _tf_listnener;

    geometry_msgs::Pose _endPose;
    geometry_msgs::Pose _startPose;

    nav_msgs::Path _path;


    NodeState _state;
    std::string _state_str;


    bool _gotNewPose;
    bool _statePathCtrl_old;

    std::string _mapFrame;
    std::string _robotFrame;


    ros::Timer _loopTimer;
public:
    PathRepeat();
    virtual ~PathRepeat();

    /**
     *
     * @brief
     *
     * @return  void
     */
    void start(double duration = 0.01);

private:    //functions

    /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();


    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void subSetEndPoint_callback(const std_msgs::Bool& msg);
    void subPathControlState_callback(const std_msgs::Bool& msg);
    void subStart_callback(const std_msgs::Bool& msg);

    void event_arrived();


    void startMove();
    void swReverseMove();

};

#endif /* PATHREPEAT_H_ */
