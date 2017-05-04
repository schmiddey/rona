
#include "PathRepeat.h"

PathRepeat::PathRepeat()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string map_frame;
    std::string robot_frame;
    //double      double_val;
    //int         int_val;
    //bool        bool_val;

    //privNh.param(         "string_val" ,    string_val,   std::string("string"));
    //privNh.param<double>( "double_val" ,    double_val,   100.0);
    //privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
    //privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);
    privNh.param("map_frame" ,    map_frame,     std::string("map"));
    privNh.param("robot_frame" ,  robot_frame,   std::string("base_footprint"));


    _mapFrame = map_frame;
    _robotFrame = robot_frame;

    //init publisher
    //_pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pubPath = _nh.advertise<nav_msgs::Path>("rona/move/path", 1);
    _pubState = _nh.advertise<std_msgs::String>("path_repeat/state", 1);
    _pubMoveCtrl    = _nh.advertise<rona_msgs::NodeCtrl>("rona/move/ctrl", 10);
    _pubMarker      = _nh.advertise<visualization_msgs::Marker>("rona/path_repeat/end_marker", 1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _subSetEndPoint      = _nh.subscribe("rona/set_end_point", 1, &PathRepeat::subSetEndPoint_callback, this);
    _subPathControlState = _nh.subscribe("rona/move/state",1 , &PathRepeat::subPathControlState_callback, this);
    _subStart            = _nh.subscribe("rona/start_repeat", 1, &PathRepeat::subStart_callback, this);

    _srv_plan = _nh.serviceClient<rona_msgs::PlanPath>("/rona/plan/path");
    _srv_move_sw_reverse = _nh.serviceClient<std_srvs::Empty>("/rona/move/set_reverse_sw");


    _state = NodeState::IDLE;
    _state_str = "IDLE";
    _gotNewPose = false;
    _statePathCtrl_old = false;

    _startPose.position.x = 4.5;
    _startPose.position.y = 2.5;

}

PathRepeat::~PathRepeat()
{
}

void PathRepeat::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &PathRepeat::loop_callback, this);
   this->run();
}

void PathRepeat::run()
{
   //do init stuff


   //end init stuff

   ros::spin();
}



void PathRepeat::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!
   std_msgs::String msg;
   msg.data = _state_str.c_str();
   this->pubMarker();
   _pubState.publish(msg);
}



void PathRepeat::subSetEndPoint_callback(const std_msgs::Bool& msg)
{
   if(msg.data == true)
   {
      tf::StampedTransform tf = rona::Utility::getTransform(_tf_listnener,_mapFrame, _robotFrame);
      _endPose.position.x = tf.getOrigin().x();
      _endPose.position.y = tf.getOrigin().y();

      ROS_INFO("PathRepeat -> Got Ent Pose...");

      _state = NodeState::WAIT_FOR_START;
      _state_str = "WAIT_FOR_START";
   }
}



void PathRepeat::subPathControlState_callback(const std_msgs::Bool& msg)
{
   if(_state != NodeState::MOVING)
      return;
   if(msg.data && !_statePathCtrl_old) //positive flank when arrived
   {
      this->event_arrived();
   }
   _statePathCtrl_old = msg.data;
}

void PathRepeat::subStart_callback(const std_msgs::Bool& msg)
{
   if(_state == NodeState::MOVING)
   {
      ROS_INFO("PathRepeat -> already moving... do nothing");
      return;
   }

   //state definitly not Moving...
   if(_state != NodeState::WAIT_FOR_START)
   {//
      ROS_ERROR("PathRepeat -> Got no End Pose .... could not Plan...");
      _state_str = "PathRepeat -> Got no End Pose .... could not Plan...";
      return;
   }

   //plan path;
   ROS_INFO("PathRepeat -> wait for plan service");
   _srv_plan.waitForExistence();
   ROS_INFO("PathRepeat -> got plan service");

   rona_msgs::PlanPath plan_srv;
   plan_srv.request.origin = _startPose;
   plan_srv.request.target = _endPose;

   if(_srv_plan.call(plan_srv))
   {//succesful srv call
      _path = plan_srv.response.path;
      ROS_INFO("PathRepeat -> Path length -> %d", (int)_path.poses.size());
      if(_path.poses.size() < 2)
      {
         ROS_ERROR("PathRepeat -> Unable to plan path ---> Fatal error ...");
         ROS_ERROR("PathRepeat -> Do Nothing!!!!");
         return;
      }
   }
   else
   {
      ROS_ERROR("PathRepeat -> unable to call plan service");
      ROS_ERROR("PathRepeat -> Do Nothing!!!!");
      return;
   }

   //then move to start
   ROS_INFO("PathRepeat -> Move to Start");

   //plan path to start
   //get current pose
   geometry_msgs::Pose currPose;
   tf::StampedTransform tf = rona::Utility::getTransform(_tf_listnener,_mapFrame, _robotFrame);
   currPose.position.x = tf.getOrigin().x();
   currPose.position.y = tf.getOrigin().y();

   plan_srv.request.origin = currPose;
   plan_srv.request.target = _startPose;

   nav_msgs::Path tmp_path;

   if(_srv_plan.call(plan_srv))
   {//succesful srv call
      tmp_path = plan_srv.response.path;
      ROS_INFO("PathRepeat -> Path length -> %d", (int)tmp_path.poses.size());
      if(tmp_path.poses.size() < 2)
      {
         ROS_ERROR("PathRepeat -> Unable to plan path ---> Fatal error ...");
         ROS_ERROR("PathRepeat -> Do Nothing!!!!");
         return;
      }
   }
   else
   {
      ROS_ERROR("PathRepeat -> unable to call plan service");
      ROS_ERROR("PathRepeat -> Do Nothing!!!!");
      return;
   }

   _pubPath.publish(tmp_path);

   //start path controller
   this->startMove();
   _state = NodeState::MOVING;
   _state_str = "MOVING";

   //when end point is current pos of the robot...
   //this->event_arrived(); //start moving...
}

void PathRepeat::event_arrived()
{
   ROS_INFO("PathRepeat -> arrived");
   //flip path....
   //std::reverse(_path.poses.begin(), _path.poses.end());

   //find nearest point (start or end point)
   rona::map::Point2D currPos = rona::Utility::getTransformPoint2D(_tf_listnener, _mapFrame, _robotFrame);
   double dist_end = rona::map::Operations::computeDistance(rona::Utility::toPoint2D(_endPose), currPos);
   double dist_start = rona::map::Operations::computeDistance(rona::Utility::toPoint2D(_startPose), currPos);

   nav_msgs::Path ros_path = _path; //copy

   if(dist_end < dist_start)
   {//start at end
      //flip path....
      std::reverse(ros_path.poses.begin(), ros_path.poses.end());
   }
//   else
//   {//start at start
//
//   }

   this->swReverseMove();

   usleep(200000);      //wait

   //pub path to path controller controller
   _pubPath.publish(ros_path);

   //start path controller
   this->startMove();
   //_state = NodeState::MOVING;
   //_state_str = "MOVING";
}


void PathRepeat::startMove()
{
   ROS_INFO("PathRepeat -> StartMove");
   rona_msgs::NodeCtrl ctrl_msg;
   ctrl_msg.cmd = ctrl_msg.START;
   ctrl_msg.cmd_str = "START";
   _pubMoveCtrl.publish(ctrl_msg);
}


void PathRepeat::swReverseMove()
{
   ROS_INFO("PathRepeat -> sw reverse Mode");
   _srv_move_sw_reverse.waitForExistence();
   //call move sw reverse srv
   std_srvs::Empty sw_srv;
   if(_srv_move_sw_reverse.call(sw_srv))
   {
      //succes
      ROS_INFO("PathRepeat -> switched reverse Mode");
   }
   else
   {
      ROS_ERROR("PathRepeat -> Unable to call sw_reverse Mode in RonaMove");
   }

}

void PathRepeat::pubMarker()
{
  if(_state == NodeState::IDLE)
  {
    return;
  }

  visualization_msgs::Marker m;

  geometry_msgs::Vector3 scale_m;
  scale_m.x = 0.4;
  scale_m.y = 0.4;
  scale_m.z = 0.5;

  std_msgs::ColorRGBA color;
  color.r = 1;
  color.g = 0;
  color.b = 0;
  color.a = 1;

  m.header.frame_id = _mapFrame;
  m.id = 0;
  m.type = m.CYLINDER;
  m.action = m.ADD;
  m.pose = _endPose;
  m.scale = scale_m;
  m.color = color;

  _pubMarker.publish(m);
}

// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sirona_path_repeat_node");
    ros::NodeHandle nh("~");

    PathRepeat node;
    node.start();

}


