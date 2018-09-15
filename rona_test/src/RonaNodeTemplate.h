/**
 * @file   RonaNodeTemplate.h
 * @author Michael Schmidpeter
 * @date   2018-09-15
 * @brief  Template/Example for RonaNode
 * 
 * PROJECT: rona
 * @see https://github.com/schmiddey/rona
 */


#ifndef RONANODETEMPLATE_H_
#define RONANODETEMPLATE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <rona_lib/Node/RonaNode.h>
//dyn reconfig
// #include <dynamic_reconfigure/server.h>
//#include <template/TemplateConfig.h>

class RonaNodeTemplate : public RonaNode
{

public:
  RonaNodeTemplate();
  virtual ~RonaNodeTemplate();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void start(double duration = 0.01);


private: //functions
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

  //void dynreconfig_callback(RonaNodeTemplate::RonaNodeTemplateConfig &config, uint32_t level);

  //rona node ctrl callbacks
  std::pair<bool, std::string> on_stop_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    this->pub_log("got stop cmd");
    if(!_run)
      return std::make_pair(false, "not running");
    _run = false;
    return std::make_pair(true, "test stop ok");
  }

  std::pair<bool, std::string> on_start_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    this->pub_log("got start cmd");
    if(_run)
      return std::make_pair(false, "already running");
    _run = true;
    return std::make_pair(true, "test start ok");
  }

  // std::pair<bool, std::string> on_restart_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

  // std::pair<bool, std::string> on_pause_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

  // std::pair<bool, std::string> on_continue_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

  // std::pair<bool, std::string> on_enable_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

  // std::pair<bool, std::string> on_disable_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

  // std::pair<bool, std::string> on_singeshot_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  // {
  //   return std::make_pair(false, "not implemented");
  // }

private: //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pub;
  ros::Subscriber _sub;

  // dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

  ros::Timer _loopTimer;

  bool _run;
};


#endif  //RONANODETEMPLATE_H_