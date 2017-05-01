#include "ExplorationNode.h"

ExplorationNode::ExplorationNode()
{
  //rosParam
  ros::NodeHandle privNh("~");
  std::string map_frame;
//  double double_val;
//  int int_val;
//  bool bool_val;
//
  privNh.param("map_frame", map_frame, std::string("map"));
//  privNh.param<double>("double_val", double_val, 100.0);
//  privNh.param<int>("int_val", int_val, 1.0);
//  privNh.param<bool>("bool_val", bool_val, true);

  //init publisher
  //_pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);
  _pubTarget = _nh.advertise<geometry_msgs::PoseStamped>("rona/exploration/target", 1);
  _pubNodeCtrl_frontier = _nh.advertise<rona_msgs::NodeCtrl>("rona/frontier/node_ctrl", 1);

  //inti subscriber
  //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
  _subSironaState = _nh.subscribe("rona/sirona/state", 1, &ExplorationNode::subSironaState_callback, this);
  _subFrontiers = _nh.subscribe("rona/frontiers", 1, &ExplorationNode::subFrontiers, this);
  _subNodeCtrl_own = _nh.subscribe("rona/exploration/node_ctrl", 1, &ExplorationNode::subNodeCtrl, this);

  //dyn reconfig
//    dynamic_reconfigure::Server<template::TemplateConfig>::CallbackType f;
//    f = boost::bind(&Template::dynreconfig_callback, this, _1, _2);
//    _drServer.setCallback(f);
  _state = exploration::IDLE;
  _mode  = rona_msgs::NodeCtrl::STOP;

  _sironaState.state = -1;
  _sironaStateOld.state = -1;

}

ExplorationNode::~ExplorationNode()
{
}

void ExplorationNode::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &ExplorationNode::loop_callback, this);
  this->run();
}

void ExplorationNode::run()
{
  ros::spin();
}

void ExplorationNode::setState(const exploration::explorationState state)
{
  _state = state;
  //todo set String... for state publisher
  ROS_INFO("set State to %d", _state);
}



void ExplorationNode::requestFrontiers()
{
  rona_msgs::NodeCtrl msg;
  msg.cmd = msg.SINGLESHOT;
  msg.cmd_str = "SINGLESHOT";
  _pubNodeCtrl_frontier.publish(msg);
}


void ExplorationNode::loop_callback(const ros::TimerEvent& e)
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

  const double T_MIN = 4.0;

  switch (_state) {
    case exploration::IDLE:
    {//wait for start;
      if(_mode != rona_msgs::NodeCtrl::START)
        break;

      ROS_INFO("rona_exploration -> start");
      this->requestFrontiers();

      this->setState(exploration::WAIT_FRONTIERS);

      break;
    }
    case exploration::WAIT_FRONTIERS:
    {
      if(_frontiers.poses.empty())
        break;

      //got frontiers
      ROS_INFO("rona_exploratoin -> Got Frontiers");
      //move frontiers to old...
      _frontiersOld = _frontiers;
      _frontiers.poses.clear();

      this->setState(exploration::NEXT_FRONTIER);

      break;
    }
    case exploration::NEXT_FRONTIER:
    {//needed as extra state in case thath frontier ist no reachable ... so take next one ...

      if(_frontiersOld.poses.empty())
      {//rdy
        this->setState(exploration::READY);
        break;
      }
      ROS_INFO("rona_exploration -> next Frontier");
      //best frontier ist first one...
      geometry_msgs::PoseStamped target;
      target.header.frame_id = _map_frame;
      target.header.stamp = ros::Time::now();
      target.pose = _frontiersOld.poses.front();

      //pub target...
      _pubTarget.publish(target);
      _t_targe_start = ros::Time::now();

      //next state moving
      this->setState(exploration::MOVING);

      break;
    }
    case exploration::MOVING:
    {
      //prove state

      double d_target_s = (ros::Time::now() - _t_targe_start).toSec();
      ROS_INFO("rona_exploration -> MOVING dur: %f", d_target_s);

      if(_sironaState.state == _sironaState.UNREACHABLE &&  d_target_s > T_MIN )
      {
        //not able to plan...
        //if no more frontiers rdy
        if(_frontiersOld.poses.size() <= 1)
        {
          this->setState(exploration::READY);
          break;
        }

        //if UNREACHABLE next
        //delete first frontier cause not reachable...
        _frontiersOld.poses.erase(_frontiersOld.poses.begin());
        //plan again...
        this->setState(exploration::NEXT_FRONTIER);

        break;
      }



      if(_sironaState.state == _sironaState.ABORTED &&  d_target_s > T_MIN )
      {
        //if abort after moving new frontiers
        //todo in Sirona SM -> or cut path for exploration...

        break;
      }

      if(_sironaState.state == _sironaState.ARRIVED &&  d_target_s > T_MIN )
      {

        this->requestFrontiers();
        this->setState(exploration::WAIT_FRONTIERS);

        break;
      }

      break;
    }
    case exploration::READY:
    {
      ROS_INFO("rona_exploration ->RDY");

      break;
    }
    default:
      ROS_ERROR("Error in Statemachine ... RONA Exploration");
      break;
  }
  //do nothing after this... only if need to do afer each state call...
}

void ExplorationNode::subSironaState_callback(const rona_msgs::State& msg)
{
  //ROS_INFO("------------> Sirona state callback");
  _sironaState = msg;
}

void ExplorationNode::subFrontiers(const geometry_msgs::PoseArray& msg)
{
  _frontiers = msg; //copy
  std::reverse(_frontiers.poses.begin(), _frontiers.poses.end());
}



void ExplorationNode::subNodeCtrl(const rona_msgs::NodeCtrl& msg)
{
  switch (msg.cmd) {
    case rona_msgs::NodeCtrl::START:
      _mode = rona_msgs::NodeCtrl::START;
      break;
    case rona_msgs::NodeCtrl::STOP:
      _mode = rona_msgs::NodeCtrl::STOP;
      //todo stop all stuff and go back to idle
      break;
      //todo do pause stuff or not...
//    case rona_msgs::NodeCtrl::PAUSE:
//      _mode = rona_msgs::NodeCtrl::PAUSE;
//      break;
//    case rona_msgs::NodeCtrl::RESTART:
//      _mode = rona_msgs::NodeCtrl::RESTART;
//      break;
    default:
      ROS_WARN("rona_expoloration -> Command not available");
      break;
  }

}

// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rona_exploration_node");
  ros::NodeHandle nh("~");

  ExplorationNode node;
  node.start(0.5);

}


