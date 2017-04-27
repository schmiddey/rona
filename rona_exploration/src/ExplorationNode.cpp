
#include "ExplorationNode.h"

ExplorationNode::ExplorationNode()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string string_val;
    double      double_val;
    int         int_val;
    bool        bool_val;

    privNh.param(         "string_val" ,    string_val,   std::string("string"));
    privNh.param<double>( "double_val" ,    double_val,   100.0);
    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
    _pub = _nh.advertise<std_msgs::Bool>("pub_name",1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);

        //dyn reconfig
//    dynamic_reconfigure::Server<template::TemplateConfig>::CallbackType f;
//    f = boost::bind(&Template::dynreconfig_callback, this, _1, _2);
//    _drServer.setCallback(f);


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



void ExplorationNode::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}










// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh("~");

    ExplorationNode node;
    node.start();

}
