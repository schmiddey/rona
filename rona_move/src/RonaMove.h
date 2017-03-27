
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <rona_msgs/NodeCtrl.h>
#include <visualization_msgs/Marker.h>

//#include <ohm_autonomy/PathControlInfo.h>
//#include <ohm_srvs/NodeControl.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
//#include <limits>

#include <rona_lib/Utility.h>

#include "PathAnalyser/PathAnalyser_base.h"
#include "Controller/Controller_base.h"

using namespace Eigen;


/*
 *
 * Funktion:
 * Start -> beginnt
 * Pause -> pause
 * Continue -> wenn in pause dann weiter, sonst ignore
 *
 * Pfad bekommen -> nur neuen Pfad setzen state beibehalten
 *
 * Pfad ankommen -> State zu Stop.
 *
 * Reached final goal nur true wenn kein empty pfad ist
 *
 */

class RonaMove
{
private:    //dataelements

    ros::NodeHandle _nh;

    ros::Publisher _pub_cmd_vel;
    ros::Publisher _pub_state;
    ros::Publisher _pub_progress;
    ros::Publisher _pub_marker;

    ros::Subscriber _sub_path;
    //ros::Subscriber _sub_pause;
    ros::Subscriber _sub_ctrl;

    ros::ServiceServer _srv_reverse_on;
    ros::ServiceServer _srv_reverse_off;
    ros::ServiceServer _srv_reverse_sw;

    ros::Timer _loopTimer;

    tf::TransformListener _tf_listnener;

    enum State{
       STOP = 0,
       MOVE = 1,
       PAUSE = 2
    };

    State _state;

    analyser::PathAnalyser_base* _pathAnalyser;
    controller::Controller_base* _controller;

    bool _enable_analyse;
    bool _oldState;
    //bool _pause;
    bool _gotPath;

    bool _reverseMode;

    std::string _tf_map_frame;
    std::string _tf_robot_frame;
    std::string _tf_robot_reverse_frame;

    double _min_vel_value;

public:
    RonaMove();
    virtual ~RonaMove();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
     *
     *
     * @return  void
     */
    void start(const double duration = 0.1);

private:    //functions

    /**
     * @brief publishs current states: reached goal... and process
     */
    void pubState();


    /**
     * @brief loop callback fct.
     *
     * @param e
     */
    void timerLoop_callback(const ros::TimerEvent& e);

    /**
     * @brief working loop for computing twist
     *
     * @param msg -> ros msg
     */
    void doPathControl(void);

    /**
     * @brief callback for next path
     *
     * @param msg -> ros msg
     */
    void subPath_callback(const nav_msgs::Path& msg);

    /**
     * @brief if msg == true... then pause is activated.... stop moving untill msg == false
     *
     * @param msg
     */
    //void subPause_callback(const std_msgs::Bool& msg);


    /**
     *
     * @param msg
     */
    void subCtrl_callback(const rona_msgs::NodeCtrl& msg);

    bool srvReverseOn_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    bool srvReverseOff_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    bool srvReverseSw_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
};

#endif /* TEMPLATE_H_ */
