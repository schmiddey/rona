/**
 * @file   RonaNodeTemplate.cpp
 * @author Michael Schmidpeter
 * @date   2018-09-15
 * @brief  Template/Example for RonaNode
 * 
 * PROJECT: rona
 * @see https://github.com/schmiddey/rona
 */

#include "RonaNodeTemplate.h"

RonaNodeTemplate::RonaNodeTemplate() : RonaNode(_nh, std::string("rona_test_ronanode"))
{
  //rosParam
  ros::NodeHandle privNh("~");
  std::string string_val;
  double double_val;
  int int_val;
  bool bool_val;


  privNh.param(         "string_val" ,    string_val,   std::string("string"));
  privNh.param<double>( "double_val" ,    double_val,   100.0);
  privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);
  //init publisher
  _pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);

  //inti subscriber
  //_sub = _nh.subscribe("subname", 1, &RonaNodeTemplate::subCallback, this);

  //dyn reconfig
  // dynamic_reconfigure::Server<RonaNodeTemplate ::RonaNodeTemplateConfig>::CallbackType f;
  // f = boost::bind(&RonaNodeTemplate::dynreconfig_callback, this, _1, _2);
  // _drServer.setCallback(f);

  _run = false;
}

RonaNodeTemplate::~RonaNodeTemplate()
{
}

void RonaNodeTemplate::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &RonaNodeTemplate::loop_callback, this);
  this->run();
}

void RonaNodeTemplate::run()
{
  ros::spin();
}

void RonaNodeTemplate::loop_callback(const ros::TimerEvent& e)
{
  if(!_run)
    return;

  ROS_INFO("WÖRK WÖRK WÖRK");
  //do loop stuff here!!!
}

// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rona_node_template_node");
  ros::NodeHandle nh("~");

  RonaNodeTemplate node;
  node.start(1.0);
}
