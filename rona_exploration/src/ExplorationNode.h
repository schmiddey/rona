#ifndef EXPLORATIONNODE_H_
#define EXPLORATIONNODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <rona_msgs/State.h>
#include <rona_msgs/NodeCtrl.h>

//dyn reconfig
//#include <dynamic_reconfigure/server.h>
//#include <template/TemplateConfig.h>

namespace exploration
{

//wait for start
  //if start then request frontiers.
//wait for frontiers
  //no frontiers rdy
//bestFrontier
//moveToFrontier
//wait for Arival/Abort
  //if abort next
    //if no more frontiers rdy
  //if abort after moving new frontiers
  //if arrived new Frontiers
enum explorationState
{
  IDLE = 0,
  WAIT_FRONTIERS,
  NEXT_FRONTIER,
  MOVING,
  READY
};
}

class ExplorationNode
{

public:
  ExplorationNode();
  virtual ~ExplorationNode();

  /**
   *
   * @brief
   *
   * @return  void
   */
  void start(double duration = 0.01);

private:
  //functions

  /**
   *
   * @brief this function containts the main working loop
   *
   * @param[in,out]  void
   *
   * @return 		   void
   */
  void run();

  void setState(const exploration::explorationState state);


  void requestFrontiers();

  void loop_callback(const ros::TimerEvent& e);

  //void subCallback(const ROS_PACK::MESSAGE& msg);
  void subSironaState_callback(const rona_msgs::State& msg);
  void subFrontiers(const geometry_msgs::PoseArray& msg);
  void subNodeCtrl(const rona_msgs::NodeCtrl& msg);

  //void dynreconfig_callback(template::TemplateConfig &config, uint32_t level);
private:
  //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubTarget;
  ros::Publisher _pubNodeCtrl_frontier;
  //ros::Publisher _pubState;
  ros::Subscriber _subSironaState;
  ros::Subscriber _subFrontiers;
  ros::Subscriber _subNodeCtrl_own;  //start //pause ....

  //dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

  ros::Timer _loopTimer;

  exploration::explorationState _state;
  rona_msgs::NodeCtrl::_cmd_type _mode;

  geometry_msgs::PoseArray _frontiers;
  geometry_msgs::PoseArray _frontiersOld;

  rona_msgs::State _sironaState;
  rona_msgs::State _sironaStateOld;

  std::string _map_frame;

  ros::Time _t_targe_start;
};

#endif /* EXPLORATIONNODE_H_ */
