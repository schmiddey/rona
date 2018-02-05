
#include "RonaWPClickNode.h"

RonaWPClickNode::RonaWPClickNode()
{
    //rosParam
//    ros::NodeHandle privNh("~");
//    std::string string_val;
//    double      double_val;
//    int         int_val;
//    bool        bool_val;
//
//    privNh.param(         "string_val" ,    string_val,   std::string("string"));
//    privNh.param<double>( "double_val" ,    double_val,   100.0);
//    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
//    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
//    _pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pub_wp_path = _nh.advertise<nav_msgs::Path>("rona/waypoint/path", 1);
    _pub_marker  = _nh.advertise<visualization_msgs::MarkerArray>("rona/waypoint/marker", 1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _sub_clicked_point = _nh.subscribe("/clicked_point", 100, &RonaWPClickNode::sub_clicked_point_callback, this);
    _sub_estimate_pose = _nh.subscribe("/initialpose", 1, &RonaWPClickNode::sub_estimate_pose_callback, this);
    _sub_map           = _nh.subscribe("map", 1, &RonaWPClickNode::sub_map_callback, this);

    _srv_plan_path     = _nh.serviceClient<rona_msgs::PlanPath>("todo");


    ROS_INFO_STREAM("map valid: " << (_map ? "true" : "false"));
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
  _pub_wp_path.publish(_wp_handler.getPathComplete());
  _pub_marker.publish(this->toMarkerArray(_wp_handler));
}


void RonaWPClickNode::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}


void RonaWPClickNode::sub_clicked_point_callback(const geometry_msgs::PointStamped& p)
{
  if(_wp_handler.empty())
  {
    _wp_handler.push(p.point);
    return;
  }
  auto d_path = this->compute_direct_path(_wp_handler.back().first, p.point);
  if(d_path.first)
  {//is occupied
    //todo compute path with sirona plan...
  }

  ROS_INFO("Cnt path: %d", (int)d_path.second.poses.size());

  _wp_handler.push(p.point, d_path.second);

  this->publish_waypoints();
}



void RonaWPClickNode::sub_estimate_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  if(_wp_handler.empty())
    return;

  _wp_handler.pop_back();
  this->publish_waypoints();
}

void RonaWPClickNode::sub_map_callback(const nav_msgs::OccupancyGridPtr map)
{
  _map = map;
}


std::pair<bool, nav_msgs::Path> RonaWPClickNode::compute_direct_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end)
{
  auto ros_time = ros::Time::now();
  nav_msgs::Path path;
  path.header.frame_id = _cfg.frame_id;
  path.header.stamp    = ros_time;
  bool occupied = false;


  tf::Vector3 st(start.x, start.y, 0.0);
  tf::Vector3 en(end.x, end.y, 0.0);

  double dist = st.distance(en);

  tf::Vector3 v_step = ((en - st).normalize()) * _cfg.step_length;

  unsigned int steps = std::round(dist / _cfg.step_length);

  geometry_msgs::PoseStamped p;
  p.header.frame_id = _cfg.frame_id;
  p.header.stamp    = ros_time;
  //todo do custom ori may be in line with path ...
  p.pose.orientation = _orientation;

  for(unsigned int i=0; i<steps; ++i)
  {
    //todo prove if occupied
    st += v_step;

    p.pose.position.x = st.x();
    p.pose.position.y = st.y();
    p.pose.position.z = 0.0;

    path.poses.push_back(p);
  }

  return std::make_pair(occupied, path);
}

nav_msgs::Path RonaWPClickNode::compute_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end)
{
  geometry_msgs::Pose ori;
  geometry_msgs::Pose tar;

  ori.position = start;
  tar.position = end;

  rona_msgs::PlanPath srv;
  srv.request.origin = ori;
  srv.request.target = tar;

  if(_srv_plan_path.call(srv))
  {
    ROS_INFO("Got Path, cnt: %d, length: %f", (int)srv.response.path.poses.size(), srv.response.length);
  }
  else
  {
    return nav_msgs::Path();
  }

  return srv.response.path;
}

visualization_msgs::MarkerArray RonaWPClickNode::toMarkerArray(const WayPointHandler& wp_handler)
{
  //black bigger sphere for startpoint, small spherer for interpolated path and LineList for wps
  rona::MarkerArrayHandler m_handler;

  //push start
  geometry_msgs::PoseStamped pose_start;
  pose_start.header.frame_id = "map";
  pose_start.header.stamp    = ros::Time::now();
  pose_start.pose.position = wp_handler.front().first;

  m_handler.push_back(rona::Marker::createSphere(pose_start, 0.1, rona::Color(rona::Color::BLUE) ) );

  std::vector<geometry_msgs::Point> waypoints;

  for(auto& e : wp_handler.getWaypoints())
  {
    //add waypoints
    waypoints.push_back(e.first);
    //add path
    for(auto& p : e.second.poses)
    {
       m_handler.push_back(rona::Marker::createSphere(p, 0.05, rona::Color(rona::Color::ORANGE) ) );
    }

  }
  //push waypoints
  m_handler.push_back(rona::Marker::createLineList(waypoints, 0.2, 0.02, rona::Color(rona::Color::RED) ) );
  return m_handler.get();
}

// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_wp_click_node");
    ros::NodeHandle nh("~");

    RonaWPClickNode node;
    node.start();

}


