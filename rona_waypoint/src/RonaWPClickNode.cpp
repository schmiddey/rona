
#include "RonaWPClickNode.h"

RonaWPClickNode::RonaWPClickNode()
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
//    _pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pub_wp_path = _nh.advertise<nav_msgs::Path>("rona/waypoint/path", 1);
    _pub_marker  = _nh.advertise<visualization_msgs::MarkerArray>("rona/waypoint/marker", 1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _sub_clicked_point = _nh.subscribe("/clicked_point", 1, &RonaWPClickNode::sub_clicked_point_callback, this);
    _sub_estimate_pose = _nh.subscribe("/initialpose", 1, &RonaWPClickNode::sub_estimate_pose_callback, this);

}

RonaWPClickNode::~RonaWPClickNode()
{
}

void RonaWPClickNode::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &RonaWPClickNode::loop_callback, this);
   this->run();
}

void RonaWPClickNode::run()
{
   ros::spin();
}

void RonaWPClickNode::publish_waypoints()
{
  //tmp code
  nav_msgs::Path path;

  auto ros_time = ros::Time::now();
  path.header.frame_id = "map";
  path.header.stamp = ros_time;

  for(const auto& e : _wp_handler.getWaypoints())
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros_time;

    p.pose.position = e;

    path.poses.push_back(p);
  }

  _pub_wp_path.publish(path);
}


void RonaWPClickNode::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}


void RonaWPClickNode::sub_clicked_point_callback(const geometry_msgs::PointStamped& p)
{
  _wp_handler.push(p.point);
  this->publish_waypoints();
}



void RonaWPClickNode::sub_estimate_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  if(_wp_handler.empty())
    return;

  _wp_handler.pop_back();
  this->publish_waypoints();
}







// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_wp_click_node");
    ros::NodeHandle nh("~");

    RonaWPClickNode node;
    node.start();

}


