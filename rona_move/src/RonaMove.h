#ifndef RONAMOVE_NODE_H_
#define RONAMOVE_NODE_H_

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <rona_msgs/NodeCtrlSRV.h>
#include <rona_msgs/NodeCtrl.h>
#include <visualization_msgs/Marker.h>

//#include <ohm_autonomy/PathControlInfo.h>
//#include <ohm_srvs/NodeControl.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
//#include <limits>

#include <rona_lib/Utility.h>
#include <rona_lib/Timer.h>

#include "PathAnalyser/PathAnalyser_base.h"
#include "Controller/Controller_base.h"

using namespace Eigen;

/*
 *
 *
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
 */
class RonaMove
{
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
  void start();

private:  //functions

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
  void doPathControl();

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
  void subPause_callback(const std_msgs::Bool& msg);

  /**
   *
   * @param msg
   */
  void subCtrl_callback(const rona_msgs::NodeCtrl& msg);

  bool srvNodeCtrl_callback(rona_msgs::NodeCtrlSRVRequest& req, rona_msgs::NodeCtrlSRVResponse& res);
  bool srvReverseOn_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool srvReverseOff_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool srvReverseSw_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);


  bool processNodeCtrl(const rona_msgs::NodeCtrl& msg);

private:
  //dataelements

  ros::NodeHandle _nh;

  ros::Publisher _pub_cmd_vel;
  ros::Publisher _pub_state;
  ros::Publisher _pub_progress;
  ros::Publisher _pub_marker;

  ros::Subscriber _sub_path;
  ros::Subscriber _sub_pause;
  ros::Subscriber _sub_ctrl;
  
  ros::ServiceServer _srv_node_ctrl;
  ros::ServiceServer _srv_reverse_on;
  ros::ServiceServer _srv_reverse_off;
  ros::ServiceServer _srv_reverse_sw;

  ros::Timer _loopTimer;

  tf::TransformListener _tf_listnener;

  //todo add pos hold
  enum State
  {
    STOP = 0,
    MOVE = 1,
    PAUSE = 2,
    HOLD_POS = 3
  };

  State _state;

  std::unique_ptr<analyser::PathAnalyser_base> _pathAnalyser;
//  analyser::PathAnalyser_base* _pathAnalyser;
  std::unique_ptr<controller::Controller_base> _controller;
//  controller::Controller_base* _controller;

  bool _gotPath;

  bool _hold_pos;

  bool _reverseMode;

  std::string _tf_map_frame;
  std::string _tf_robot_frame;
  std::string _tf_robot_reverse_frame;

  double _loop_duration;

  double _min_vel_value;
  double _robot_radius;

  double _path_truncate;

  double _tf_stamp_offset;
  bool _use_tf_stamp_offset;
};

#endif /* TEMPLATE_H_ */
