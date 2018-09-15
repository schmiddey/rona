/**
 * @file   RonaNode.h
 * @author Michael Schmidpeter
 * @date   2018-09-14
 * @brief  Base class for rona ROS nodes
 * 
 * PROJECT: rona
 * @see https://github.com/schmiddey/rona
 */


#ifndef RONANODE_H_
#define RONANODE_H_

#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rona_msgs/NodeCtrl.h>
#include <rona_msgs/NodeCtrlSRV.h>



class RonaNode{
public:
  RonaNode(const ros::NodeHandle& nh, const std::string& node_name) :
    //_nh(nh)
    _node_name(node_name)
  {
    //_node_name = node_name;
    //get param
    ros::NodeHandle privNh("~");
    bool auto_start;
    privNh.param<bool>  ("auto_start" , auto_start , false);

    ROS_INFO_STREAM("aut_start: " << std::boolalpha << auto_start);

    _srv_node_ctrl = _nh.advertiseService(node_name + "/node_ctrl", &RonaNode::srv_node_ctrl_callback, this);

    _pub_log        = _nh.advertise<std_msgs::String>(node_name + "/log", 1, true);
    _pub_log_global = _nh.advertise<std_msgs::String>("rona/log", 1, true);

    //do auto start -> timer single shot ... called after spin called
    if(auto_start)
    {
      _timer_auto_start  = std::make_unique<ros::Timer>();
      *_timer_auto_start = _nh.createTimer(ros::Duration(0.01), &RonaNode::auto_start_callback, this, true); //oneshot
    }
  }
  ~RonaNode()
  { }

protected:

  void pub_log(const std::string& log_msg)
  {
    std_msgs::String msg;
    msg.data = log_msg;
    _pub_log.publish(msg);
    msg.data = _node_name + ": " + log_msg;
    _pub_log_global.publish(msg);
  }

virtual   std::pair<bool, std::string> on_stop_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_start_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_restart_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_pause_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_continue_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_enable_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_disable_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

  virtual std::pair<bool, std::string> on_singeshot_callback(const rona_msgs::NodeCtrl& node_ctrl) noexcept
  {
    return std::make_pair(false, "not implemented");
  }

private: //fnc
  bool srv_node_ctrl_callback(rona_msgs::NodeCtrlSRV::Request& req, rona_msgs::NodeCtrlSRV::Response& res)
  {
    auto ret = this->process_node_ctrl(req.ctrl);
    res.accepted = ret.first;
    res.ret_msg = ret.second;
    return true;
  }

  std::pair<bool, std::string> process_node_ctrl(const rona_msgs::NodeCtrl& node_ctrl)
  {
    const auto cmd = node_ctrl;
    //call callbacks
    switch(cmd.cmd) {
      case cmd.STOP:
        return this->on_stop_callback(cmd);
        break;
      case cmd.START:
        return this->on_start_callback(cmd);
        break;
      case cmd.RESTART:
        return this->on_restart_callback(cmd);
        break;
      case cmd.PAUSE:
        return this->on_pause_callback(cmd);
        break;
      case cmd.CONTINUE:
        return this->on_continue_callback(cmd);
        break;
      case cmd.ENABLE:
        return this->on_enable_callback(cmd);
        break;
      case cmd.DISABLE:
        return this->on_disable_callback(cmd);
        break;
      case cmd.SINGLESHOT:
        return this->on_singeshot_callback(cmd);
        break;
      default:
        break;
    }
    return std::make_pair(false, "invalid command");
  }

   void auto_start_callback(const ros::TimerEvent& e)
   {
    //  ROS_INFO("Autostart callback called");
     rona_msgs::NodeCtrl node_ctrl;
     node_ctrl.cmd = node_ctrl.START;
     node_ctrl.cmd_str = "AUTOSTART";
     this->on_start_callback(node_ctrl);
   }
private: //data
  ros::NodeHandle _nh;

  ros::ServiceServer _srv_node_ctrl;

  ros::Publisher _pub_log;
  ros::Publisher _pub_log_global;
  std::unique_ptr<ros::Timer> _timer_auto_start;
  const std::string _node_name;
};


#endif  //RONANODE_H_