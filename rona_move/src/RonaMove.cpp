
#include "PathAnalyser/BasicAnalyser.h"
#include "Controller/ParabolaTransfere.h"
#include "Controller/PDController.h"
#include "RonaMove.h"


#include <rona_lib/Utility.h>

RonaMove::RonaMove()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_cmd_vel;
    std::string pub_name_state;
    std::string pub_name_process;
    std::string pub_name_marker;
    std::string sub_name_path;
    //std::string sub_name_pause;
    std::string sub_ctrl_topic;

    std::string tf_map_frame;
    std::string tf_robot_frame;
    std::string tf_robot_reverse_frame;

    double vel_lin_max;
    double vel_ang_max;

    double target_radius       ;
    double target_radius_final ;
    int    cos_pwr_n           ;
    double cos_fac_n           ;
    double ang_reached_range   ;
    double lin_end_approach    ;
    double lin_ctrl_scale      ;
    double ang_ctrl_scale      ;

    bool do_endrotate;

    double min_vel_value;        //mainly for simluator issue

    privNh.param        ("pub_cmd_vel_topic"    ,   pub_name_cmd_vel       , std::string("cmd_vel"           ));
    privNh.param        ("pub_state_topic"      ,   pub_name_state         , std::string("rona/move/state"   ));
    privNh.param        ("pub_process_topic"    ,   pub_name_process       , std::string("rona/move/process" ));
    privNh.param        ("pub_name_marker"      ,   pub_name_marker        , std::string("rona/robot_marker" ));
    privNh.param        ("sub_path_topic"       ,   sub_name_path          , std::string("rona/move/path"    ));
    privNh.param        ("sub_ctrl_topic"       ,   sub_ctrl_topic         , std::string("rona/move/ctrl"    ));
    privNh.param        ("map_frame"            ,   tf_map_frame           , std::string("map"               ));
    privNh.param        ("robot_frame"          ,   tf_robot_frame         , std::string("base_footprint"    ));
    privNh.param        ("robot_reverse_frame"  ,   tf_robot_reverse_frame , std::string("base_footprint_reverse"    ));
    privNh.param<double>("vel_lin_max"          ,   vel_lin_max            , 0.4  );
    privNh.param<double>("vel_ang_max"          ,   vel_ang_max            , 0.8  );
    privNh.param<double>("target_radius"        ,   target_radius          , 0.24 );
    privNh.param<double>("target_radius_final"  ,   target_radius_final    , 0.1  );
    privNh.param<int>   ("cos_pwr_n"            ,   cos_pwr_n              , 4    );
    privNh.param<double>("cos_fac_n"            ,   cos_fac_n              , 3.0  );
    privNh.param<double>("ang_reached_range"    ,   ang_reached_range      , 0.05  );
    privNh.param<double>("lin_end_approach"     ,   lin_end_approach       , 1.0  );
    privNh.param<double>("lin_ctrl_scale"       ,   lin_ctrl_scale         , 2.0  );
    privNh.param<double>("ang_ctrl_scale"       ,   ang_ctrl_scale         , 4.0  );
    privNh.param<double>("min_vel_value"        ,   min_vel_value          , 0.001);
    privNh.param<bool>  ("do_endrotate"         ,   do_endrotate           , false);


    _tf_map_frame = tf_map_frame;
    _tf_robot_frame = tf_robot_frame;
    _tf_robot_reverse_frame = tf_robot_reverse_frame;

    ROS_INFO("robot_frame:         %s", _tf_robot_frame.c_str());
    ROS_INFO("robot_reverse_frame: %s", _tf_robot_reverse_frame.c_str());

    //init publisher
    _pub_cmd_vel  = _nh.advertise<geometry_msgs::Twist>(pub_name_cmd_vel,10);
    _pub_state    = _nh.advertise<std_msgs::Bool>(pub_name_state,1);
    _pub_progress = _nh.advertise<std_msgs::UInt32>(pub_name_process,1);
    _pub_marker   = _nh.advertise<visualization_msgs::Marker>(pub_name_marker, 1);

    //inti subscriber
    _sub_path = _nh.subscribe(sub_name_path , 10, &RonaMove::subPath_callback, this);
    _sub_ctrl = _nh.subscribe(sub_ctrl_topic, 1000, &RonaMove::subCtrl_callback, this);
    //_sub_pause = _nh.subscribe(sub_name_pause, 1, &RonaMove::subPause_callback, this);

    _srv_reverse_on  = _nh.advertiseService("/rona/move/set_reverse_on", &RonaMove::srvReverseOn_callback, this);
    _srv_reverse_off = _nh.advertiseService("/rona/move/set_reverse_off", &RonaMove::srvReverseOff_callback, this);
    _srv_reverse_sw = _nh.advertiseService("/rona/move/set_reverse_sw", &RonaMove::srvReverseSw_callback, this);


    //_pathAnalyser = new analyser::BasicAnalyser(0.24, 0.1, 4, 1, 0.1, 1.0);
    _pathAnalyser = new analyser::BasicAnalyser(target_radius, target_radius_final, cos_pwr_n, cos_fac_n, ang_reached_range, lin_end_approach);
    _pathAnalyser->setDoEndRotate(do_endrotate);

    _controller = new controller::ParabolaTransfere(vel_lin_max, vel_ang_max, lin_ctrl_scale, ang_ctrl_scale);
    //_controller = new controller::PIController(vel_lin_max, vel_ang_max, lin_ctrl_scale, 1.0/15.0, 4, 5);
    //_controller = new controller::PDController(vel_lin_max, vel_ang_max, lin_ctrl_scale, 0.2, 1.0/15.0, 2);
    _enable_analyse = false;
    _oldState = false;
    _gotPath = false;

    _reverseMode = false;

    //_pause = false;
    _state = State::STOP;

    _min_vel_value = min_vel_value;
}

RonaMove::~RonaMove()
{
}

void RonaMove::start(const double duration)
{

   //wait for first transform
   bool rdy = false;
   do{
      try {
         ros::Time time = ros::Time::now();
         _tf_listnener.waitForTransform(_tf_map_frame, _tf_robot_frame, time, ros::Duration(1));
         rdy = true;

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("sim_move -> Exeption at tf: %s", e.what());
         return;
      }
   }while(!rdy);

   _loopTimer = _nh.createTimer(ros::Duration(duration), &RonaMove::timerLoop_callback, this);
   ros::spin();
}


void RonaMove::pubState()
{

   //for old msg
   std_msgs::Bool state_msg_old;
   state_msg_old.data = false;
   std_msgs::UInt32 process;
   process.data = _pathAnalyser->getInfo().current_goal_id;

   if(_pathAnalyser->isReachedFinalGoal())
      state_msg_old.data = true;

   _pub_progress.publish(process);
   _pub_state.publish(state_msg_old);

   //problem... if empty path is sent -> target is reached.
   if(state_msg_old.data)
   {
//      if(_pathAnalyser->isEmpty())
//      {//stopped with empty path
//
//      }
//      else
      {//arrived:
         _enable_analyse = false;
         _state = State::STOP;
      }
   }

}

void RonaMove::timerLoop_callback(const ros::TimerEvent& e)
{
   this->doPathControl();

   //pub Marker
   rona::map::Point2D pose = rona::Utility::getTransformPoint2D(_tf_listnener, _tf_map_frame, _tf_robot_frame);
   visualization_msgs::Marker m;

   geometry_msgs::Vector3 scale_m;
   scale_m.x = 0.6;
   scale_m.y = 0.6;
   scale_m.z = 0.6;

   std_msgs::ColorRGBA color;
   color.r = 0;
   color.g = 0;
   color.b = 0;
   color.a = 0.5;

   m.header.frame_id = _tf_map_frame;
   m.id = 0;
   m.type = m.SPHERE;
   m.action = m.ADD;
   m.pose = rona::Utility::toRosPoseStamped(pose).pose;  //orientation not used
   m.scale = scale_m;
   m.color = color;

   _pub_marker.publish(m);
}


void RonaMove::doPathControl(void)
{

   if(!_enable_analyse)
   {
      this->pubState();

      geometry_msgs::Twist msgTwist;
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;
      if(_gotPath)
         _pub_cmd_vel.publish(msgTwist);
      return;
   }

   //get tf
   tf::StampedTransform tf;

   std::string robot_frame;

   if(!_reverseMode)
      robot_frame = _tf_robot_frame;
   else
      robot_frame = _tf_robot_reverse_frame;


   try {
      ros::Time time = ros::Time(0);
      _tf_listnener.lookupTransform(_tf_map_frame, robot_frame, time, tf);

   } catch (tf::TransformException& e)
   {
      ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
      return;
   }

   analyser::pose pose;
   pose.position = Vector3d( tf.getOrigin().x(), tf.getOrigin().y(), 0);

   Quaternion<double> tmp_q(tf.getRotation().w(),
                            tf.getRotation().x(),
                            tf.getRotation().y(),
                            tf.getRotation().z() );
   pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);

   //get diff scale
   analyser::diff_scale diff_scale = _pathAnalyser->analyse(pose);

   //controll diffscale
   controller::velocity vel = _controller->control(diff_scale.linear, diff_scale.angular);

   //set twist msg
   geometry_msgs::Twist msgTwist;;
   msgTwist.angular.z = rona::Utility::proveMin(vel.angular, _min_vel_value);
   msgTwist.linear.x = rona::Utility::proveMin(vel.linear, _min_vel_value);

   if(_reverseMode)
      msgTwist.linear.x *= -1;

//   if(pathInfo.reached_final_goal)
//   {
//      //_enable_analyse = false;
//   }


   //todo bug -> sents max speed comman one times if arrived

   //if(_pause)
   if(_state != State::MOVE)
   {
      ROS_INFO("Rona_move->Pause");    //todo bug hier!!!! pause kommt ned!!!
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;
   }

   this->pubState();

   //publish Twist:
   _pub_cmd_vel.publish(msgTwist);
}

void RonaMove::subPath_callback(const nav_msgs::Path& msg_)
{
   _gotPath = true;
   nav_msgs::Path msg = msg_;
   //path with length 1 is invalid, if invalid then clear
   if(msg.poses.size() <= 1)
   {
      ROS_INFO("Rona_move-> got emptyPath");
      _state = State::STOP;
      msg.poses.clear();
   }
   std::vector<analyser::pose> path_comp;
   std::vector<analyser::pose> path_trunc;

   path_comp.resize(msg.poses.size());
   path_trunc.resize(msg.poses.size());

   ROS_INFO("ohm_path_control -> GOT Path");

   //set path
   for(unsigned int i=0; i<msg.poses.size(); i++)
   {
      analyser::pose tmp_pose;
      tmp_pose.position = Vector3d(msg.poses[i].pose.position.x,
                                   msg.poses[i].pose.position.y,
                                   msg.poses[i].pose.position.z);

      Quaternion<double> tmp_q(msg.poses[i].pose.orientation.w,
                               msg.poses[i].pose.orientation.x,
                               msg.poses[i].pose.orientation.y,
                               msg.poses[i].pose.orientation.z);

      tmp_pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);
      path_comp[i] = tmp_pose;
      path_trunc[i] = tmp_pose;
   }

   //get tf
   bool tf_rdy = true;
   tf::StampedTransform tf;
   do{
      try {
         ros::Time time = ros::Time(0);
         _tf_listnener.lookupTransform(_tf_map_frame, _tf_robot_frame, time, tf);

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
         tf_rdy = false;
      }
   }while(!tf_rdy);
   //tf rdy

   //now porve if path is in radius

   analyser::BasicAnalyser* tmp_analyser = dynamic_cast<analyser::BasicAnalyser*>(_pathAnalyser);
   if(tmp_analyser)
   {//just do if cast is valid
      double detectionRaidus = tmp_analyser->getDetectionRadius();

      detectionRaidus *= 0.5; // todo use parameter to config this

      //px, py : robot pose
      double px = tf.getOrigin().x();
      double py = tf.getOrigin().y();

      unsigned int trunc_idx = 0;
      for(int i = (int)path_comp.size() - 1; i >= 0; --i)
      {
         //cx, cy : current path point
         double cx = path_comp[i].position.x();
         double cy = path_comp[i].position.y();

         //prove if path point is in robot pose + detection radius: (x-x0)^2 + (y-y0)^2 < r^2
         if( (cx - px)*(cx - px) + (cy - py)*(cy - py) < (detectionRaidus * detectionRaidus) )
         {//point is in radius
            //ROS_INFO("Found stuff in Radius!!!!!!!!!!!!!");
            trunc_idx = i;
            break;
         }
      }

      if(trunc_idx)
         path_trunc.erase(path_trunc.begin(), path_trunc.begin() + trunc_idx);

      if(!path_trunc.size())
      {//if path_tranc contains no points than use path_comp;
         path_trunc = path_comp;
      }
   }
   else
   {
      path_trunc = path_comp;
   }


   ROS_INFO("Path.size: %d",(int)path_trunc.size());
   _pathAnalyser->setPath(path_trunc);
   //_enable_analyse = true;
   std_msgs::Bool reachedTarget;
   reachedTarget.data = false;

   //pub false
   _pub_state.publish(reachedTarget);

}



//void RonaMove::subPause_callback(const std_msgs::Bool& msg)
//{
//   _pause = msg.data;
//}


void RonaMove::subCtrl_callback(const rona_msgs::NodeCtrl& msg)
{
   if(msg.cmd == msg.START)
   {
      _enable_analyse = true;
      _state = State::MOVE;
   }
   else if(msg.cmd == msg.PAUSE)
   {
      _state = State::PAUSE;
   }
   else if(msg.cmd == msg.CONTINUE)
   {//continue only if is in pause state
      if(_state == State::PAUSE)
      {
         _state = State::MOVE;
      }
   }
   else if(msg.cmd == msg.STOP)
   {
      _enable_analyse = false;
      _state = State::STOP;
      //clean analyser path...
      _pathAnalyser->clear();
   }
}


bool RonaMove::srvReverseOn_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
   ROS_INFO("RonaMove -> reverseMode = true");
   _reverseMode = true;
   return true;
}

bool RonaMove::srvReverseOff_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
   ROS_INFO("RonaMove -> reverseMode = false");
   _reverseMode = false;
   return true;
}

bool RonaMove::srvReverseSw_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
   ROS_INFO("RonaMove -> sw reverseMode caled -> curr state: %s", _reverseMode ? "true":"false");
   _reverseMode = !_reverseMode;
   ROS_INFO("RonaMove -> sw reverseMode caled -> after state: %s", _reverseMode ? "true":"false");
   return true;
}


//--------main-----------------

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_move_node");
    ros::NodeHandle nh("~");

    RonaMove node;
    node.start(0.05);

}


