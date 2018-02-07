
#include "RonaWPExecute.h"

RonaWPExecute::RonaWPExecute()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string wp_file_load;
//    double      double_val;
//    int         int_val;
//    bool        bool_val;

    privNh.param(         "wp_file_load" ,    wp_file_load,   _cfg.wp_file_load);
//    privNh.param<double>( "double_val" ,    double_val,   100.0);
//    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
//    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
    _pub_path   = _nh.advertise<nav_msgs::Path>("rona/move/path",1);      //todo check....
    _pub_state  = _nh.advertise<std_msgs::String>("rona/waypoint/state", 1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _sub_move_state = _nh.subscribe("rona/move/state", 1, &RonaWPExecute::sub_move_state_callback, this);
    _sub_node_ctrl  = _nh.subscribe("rona/waypoint/node_ctrl", 1, &RonaWPExecute::sub_node_ctrl_callback, this);
    _sub_load_wp    = _nh.subscribe("rona/waypoint/load", 1, &RonaWPExecute::sub_load_wp_callback, this);

    _curr_wp_id = 0;

    _state = state::IDLE;

    _last_move_state = false;
    _stop_at_next_wp = false;

}

RonaWPExecute::~RonaWPExecute()
{
}

void RonaWPExecute::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &RonaWPExecute::loop_callback, this);
   this->run();
}

void RonaWPExecute::run()
{
   ros::spin();
}



void RonaWPExecute::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!
  ros::Duration d = ros::Time::now() - _wait_time;
  if(d.toSec() >= _cfg.wait_duration_s)
  {
    //next wp...
    this->pub_wp_path(this->next_wp());
  }

  //pub state
  std_msgs::String ros_str;
  ros_str = state::toString(_state);
  _pub_state.publish(ros_str);
}


void RonaWPExecute::sub_move_state_callback(const std_msgs::Bool& msg)
{
  if(!_last_move_state && msg.data)
  {
    //arrived
    if(_stop_at_next_wp)
    {
      _state = state::STOP;
    }
    else
    {
      _state = state::WAIT;
      _wait_time = ros::Time::now();
    }
  }
  _last_move_state = msg.data;
}

void RonaWPExecute::sub_node_ctrl_callback(const rona_msgs::NodeCtrl& msg)
{
  if(msg.cmd == msg.START && (_state == state::IDLE || _state == state::STOP ))
  {
    _state = state::MOVE;
    this->pub_wp_path(_curr_wp_id);
  }
}

void RonaWPExecute::sub_load_wp_callback(const std_msgs::String& msg)
{
  _wp_handler.clear();
  _wp_handler.load(msg.data);
}





// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh("~");

    RonaWPExecute node;
    node.start();

}


