
#include "RonaWPClickNode.h"

RonaWPClickNode::RonaWPClickNode()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string wp_file_load;
    std::string wp_file_save;
    std::string map_frame;
    std::string robot_frame;
//    double      double_val;
//    int         int_val;
    bool        save_at_exit;
//
    privNh.param(         "wp_file_load" ,    wp_file_load,   _cfg.wp_file_load);
    privNh.param(         "wp_file_save" ,    wp_file_save,   _cfg.wp_file_save);
    privNh.param(         "map_frame" ,       map_frame,       std::string("map"));
    privNh.param(         "robot_frame" ,     robot_frame,     std::string("base_footprint"));
//    privNh.param<double>( "double_val" ,    double_val,   100.0);
//    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
    privNh.param<bool>(   "save_at_exit"   ,    save_at_exit  ,   _cfg.save_at_exit);

    _cfg.wp_file_load = wp_file_load;
    _cfg.wp_file_save = wp_file_save;
    _cfg.save_at_exit = save_at_exit;

    _map_frame = map_frame;
    _robot_frame = robot_frame;

    //init publisher
//    _pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pub_wp_path = _nh.advertise<nav_msgs::Path>("rona/waypoint/path", 1);
    _pub_marker  = _nh.advertise<visualization_msgs::MarkerArray>("rona/waypoint/marker", 1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _sub_clicked_point = _nh.subscribe("/clicked_point", 100, &RonaWPClickNode::sub_clicked_point_callback, this);
    _sub_estimate_pose = _nh.subscribe("/initialpose", 1, &RonaWPClickNode::sub_estimate_pose_callback, this);
    _sub_nav_goal      = _nh.subscribe("/move_base_simple/goal", 1, &RonaWPClickNode::sub_nav_goal_callback, this);

    _srv_save             = _nh.advertiseService("rona/waypoint/save_waypoints", &RonaWPClickNode::srv_save_wp_callback, this);
    _srv_set_curr_tf_pose = _nh.advertiseService("rona/waypoint/tf_wp", &RonaWPClickNode::srv_set_curr_tf_pose_callback, this);

    _srv_plan_path     = _nh.serviceClient<rona_msgs::PlanPath>("todo");

    _orientation.w = 1.0;

    if(!_cfg.wp_file_load.empty())
    {
      //todo may sleep...
      ROS_INFO_STREAM("Load waypoints: " << _cfg.wp_file_load);
      if(!_wp_handler.load(_cfg.wp_file_load))
      {
        ROS_ERROR_STREAM("Unable to open waypoint file: " << _cfg.wp_file_load << ", use empty waypoints...");
      }
      else
      {
        ROS_INFO_STREAM("Loaded waypoints (cnt: " << _wp_handler.size() << ")");
        this->publish_waypoints();
      }
    }

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

   if(_cfg.save_at_exit)
   {
     //compute path from last wp to first wp...
     //  auto path = this->compute_direct_path(_wp_handler.back().first.position, _wp_handler.front().first.position, _orientation);
     auto path = WayPointHelper::compute_direct_path(_wp_handler.back().first.position, _wp_handler.front().first.position, _orientation, _cfg.step_length, _cfg.frame_id);
     _wp_handler.front().second = path.second;
     std::cout << "Save Waypoints at exit... wait 2 sec then exit..." << std::endl;
     _wp_handler.serialize(_cfg.wp_file_save);
    //  this->publish_waypoints();
     ::usleep(2000000);
   }

   std::cout << "Exit..." << std::endl;

}

void RonaWPClickNode::publish_waypoints()
{
  if(_wp_handler.empty())
    return;

  _pub_wp_path.publish(_wp_handler.getPathComplete());

  //auto marker = this->toMarkerArray(_wp_handler);

//  std::cout << "marker size: " << marker.markers.size() << std::endl;

  _pub_marker.publish(_wp_handler.toMarkerArray());
}


void RonaWPClickNode::add_waypoint(const geometry_msgs::Pose& p)
{
  auto pose = p;
  //set z to zero
  pose.position.z = 0.0;

  //std::string str_pose = _wp_handler.pose2string(pose);

  // ROS_INFO_STREAM("test output: " << pose);

  // ROS_INFO_STREAM("got Point: " << str_pose);

  // ROS_INFO_STREAM("from string to Pose: " << _wp_handler.string2pose(str_pose));
  
  ROS_INFO_STREAM("Add Waypoint... ID: " << _wp_handler.size());

  if(_wp_handler.empty())
  {
    _wp_handler.push(pose);
    this->publish_waypoints();
    return;
  }
  auto d_path = WayPointHelper::compute_direct_path(_wp_handler.back().first.position, pose.position, p.orientation, _cfg.step_length, _cfg.frame_id);
  // auto d_path = this->compute_direct_path(_wp_handler.back().first.position, pose.position, p.orientation);
  if(d_path.first)
  {//is occupied
    //todo compute path with sirona plan...
  }

  //ROS_INFO("Cnt path: %d", (int)d_path.second.poses.size());

  _wp_handler.push(pose, d_path.second);

  this->publish_waypoints();
}

void RonaWPClickNode::loop_callback(const ros::TimerEvent& e)
{
  this->publish_waypoints();
}


void RonaWPClickNode::sub_clicked_point_callback(const geometry_msgs::PointStamped& p)
{
  geometry_msgs::Pose pose;

  pose.position = p.point;
  pose.orientation = _orientation;

  this->add_waypoint(pose);
}



void RonaWPClickNode::sub_estimate_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  if(_wp_handler.empty())
    return;

  _wp_handler.pop_back();
  this->publish_waypoints();
}

void RonaWPClickNode::sub_nav_goal_callback(const geometry_msgs::PoseStamped& pose)
{
  _orientation = pose.pose.orientation;
  if(!_wp_handler.empty())
  {
    _wp_handler.back().first.orientation = _orientation;
    this->publish_waypoints();
  }
}

bool RonaWPClickNode::srv_save_wp_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  //compute path from last wp to first wp...
  auto path = WayPointHelper::compute_direct_path(_wp_handler.back().first.position, _wp_handler.front().first.position, _orientation, _cfg.step_length, _cfg.frame_id);
  // auto path = this->compute_direct_path(_wp_handler.back().first.position, _wp_handler.front().first.position, _orientation);
  _wp_handler.front().second = path.second;
  _wp_handler.serialize(_cfg.wp_file_save);
  return true;
}

bool RonaWPClickNode::srv_set_curr_tf_pose_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  auto tf_stamped = rona::Utility::getTransform(_tf_listener, _map_frame, _robot_frame);
  
  geometry_msgs::Pose pose;
  pose.position.x = tf_stamped.getOrigin().getX();
  pose.position.y = tf_stamped.getOrigin().getY();
  pose.position.z = tf_stamped.getOrigin().getZ();
  pose.orientation.x = tf_stamped.getRotation().getX();
  pose.orientation.y = tf_stamped.getRotation().getY();
  pose.orientation.z = tf_stamped.getRotation().getZ();
  pose.orientation.w = tf_stamped.getRotation().getW();

  _orientation = pose.orientation;

  this->add_waypoint(pose);
  return true;
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



//visualization_msgs::MarkerArray RonaWPClickNode::toMarkerArray(const WayPointHandler& wp_handler)
//{
//  //black bigger sphere for startpoint, small spherer for interpolated path and LineList for wps
//  rona::MarkerArrayHandler m_handler("wp");
//
//  //push start
//  geometry_msgs::PoseStamped pose_start;
//  pose_start.header.frame_id = "map";
//  pose_start.header.stamp    = ros::Time::now();
//  pose_start.pose.position = wp_handler.front().first.position;
//
//  m_handler.push_back(rona::Marker::createSphere(pose_start, 0.1, rona::Color(rona::Color::BLUE) ) );
//
//  std::vector<geometry_msgs::Point> waypoints;
//
//  for(auto& e : wp_handler.getWaypoints())
//  {
//    //add waypoints
//    //waypoints.push_back(e.first);
//    m_handler.push_back(rona::Marker::createCyliner(e.first.position, 0.2, 0.02, rona::Color(rona::Color::RED) ) );
//    m_handler.push_back(rona::Marker::createArrow(e.first, 0.4, 0.02, rona::Color(rona::Color::BLACK) ) );
//    //add path
//    for(auto& p : e.second.poses)
//    {
//       m_handler.push_back(rona::Marker::createSphere(p, 0.05, rona::Color(rona::Color::ORANGE) ) );
//       m_handler.push_back(rona::Marker::createArrow(p.pose, 0.2, 0.01, rona::Color(rona::Color::BLACK) ) );
//    }
//
//  }
//  //push waypoints
//  //m_handler.push_back(rona::Marker::createLineList(waypoints, 0.2, 0.02, rona::Color(rona::Color::RED) ) );
//  return m_handler.get();
//}

// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_waypoint_click_node");
    ros::NodeHandle nh("~");

    RonaWPClickNode node;
    node.start(0.2);

}


