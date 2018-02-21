
#include "SironaSM.h"

SironaSM::SironaSM()
{
   //rosParam
   ros::NodeHandle privNh("~");
   //std::string string_value;
   //double double_value;
   //int int_val;
   std::string pub_re_target_topic  ;
   std::string pub_re_addob_topic   ;
   std::string pub_re_rmob_topic    ;
   std::string pub_replan_topic     ;
   std::string pub_path_topic       ;
   std::string pub_state_topic      ;
   std::string sub_target_topic     ;
   std::string sub_addob_topic      ;
   std::string sub_rmob_topic       ;
   std::string sub_path_topic       ;
   std::string sub_state_topic      ;
   std::string robot_frame          ;
   std::string map_frame            ;
   std::string pub_move_ctrl        ;
   double      obstacle_block_time  ;

   double ob_react_poly_0_x;
   double ob_react_poly_0_y;
   double ob_react_poly_1_x;
   double ob_react_poly_1_y;
   double ob_react_poly_2_x;
   double ob_react_poly_2_y;
   double ob_react_poly_3_x;
   double ob_react_poly_3_y;


   //privNh.param("string_value",string_value,std::string("std_value"));
   //privNh.param<double>("double_value",double_value, 12.34);
   //privNh.param<int>("int_val",int_val, 1234);
   privNh.param        ("pub_re_target_topic"  ,pub_re_target_topic  ,std::string("rona/plan/target"      ));
   privNh.param        ("pub_re_addob_topic"   ,pub_re_addob_topic   ,std::string("rona/plan/add_obstacle"));
   privNh.param        ("pub_re_rmob_topic"    ,pub_re_rmob_topic    ,std::string("rona/plan/rm_obstacle"));
   privNh.param        ("pub_replan_topic"     ,pub_replan_topic     ,std::string("rona/plan/replan"      ));
   privNh.param        ("pub_path_topic"       ,pub_path_topic       ,std::string("rona/move/path"        ));
   privNh.param        ("pub_state_topic"      ,pub_state_topic      ,std::string("rona/sirona/state"            ));
   privNh.param        ("pub_move_ctrl_topic"  ,pub_move_ctrl        ,std::string("rona/move/ctrl"        ));
   privNh.param        ("sub_target_topic"     ,sub_target_topic     ,std::string("/move_base_simple/goal"));
   privNh.param        ("sub_addob_topic"      ,sub_addob_topic      ,std::string("rona/add_obstacle"     ));
   privNh.param        ("sub_rmob_topic"       ,sub_rmob_topic       ,std::string("rona/rm_obstacle"     ));
   privNh.param        ("sub_path_topic"       ,sub_path_topic       ,std::string("rona/plan/path"        ));
   privNh.param        ("sub_state_topic"      ,sub_state_topic      ,std::string("rona/move/state"       ));
   privNh.param        ("robot_frame"          ,robot_frame          ,std::string("base_footprint"        ));
   privNh.param        ("map_frame"            ,map_frame            ,std::string("map"                   ));
   privNh.param<double>("obstacle_block_time"  ,obstacle_block_time  , 2.0    );
   privNh.param<double>("ob_react_poly_0_x"    ,ob_react_poly_0_x    , 0.0  );
   privNh.param<double>("ob_react_poly_0_y"    ,ob_react_poly_0_y    , 0.3  );
   privNh.param<double>("ob_react_poly_1_x"    ,ob_react_poly_1_x    , 0.0  );
   privNh.param<double>("ob_react_poly_1_y"    ,ob_react_poly_1_y    , -0.3 );
   privNh.param<double>("ob_react_poly_2_x"    ,ob_react_poly_2_x    , 1.2  );
   privNh.param<double>("ob_react_poly_2_y"    ,ob_react_poly_2_y    , -0.5 );
   privNh.param<double>("ob_react_poly_3_x"    ,ob_react_poly_3_x    , 1.2  );
   privNh.param<double>("ob_react_poly_3_y"    ,ob_react_poly_3_y    , 0.5  );

   //init publisher
   //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
   _pubRelayTarget = _nh.advertise<geometry_msgs::PoseStamped>(pub_re_target_topic, 100);
   _pubRelayAddOb  = _nh.advertise<rona_msgs::Obstacle>(pub_re_addob_topic, 2);
   _pubRelayRmOb   = _nh.advertise<std_msgs::String>(pub_re_rmob_topic, 2);
   _pubReplan      = _nh.advertise<std_msgs::Bool>(pub_replan_topic, 100);
   _pubPathMove    = _nh.advertise<nav_msgs::Path>(pub_path_topic, 100);
   _pubState       = _nh.advertise<rona_msgs::State>(pub_state_topic, 100);
   _pubMoveCtrl    = _nh.advertise<rona_msgs::NodeCtrl>(pub_move_ctrl, 100);

   //inti subscriber
   //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
   _subTarget      = _nh.subscribe(sub_target_topic, 2, &SironaSM::sub_targetCallback, this);
   _subAddOb       = _nh.subscribe(sub_addob_topic, 100, &SironaSM::sub_addObCallback, this);
   _subRmOb        = _nh.subscribe(sub_rmob_topic, 100, &SironaSM::sub_rmObCallback, this);
   _subPath        = _nh.subscribe(sub_path_topic, 100, &SironaSM::sub_pathCallback, this);
   _subStateMove   = _nh.subscribe(sub_state_topic, 100, &SironaSM::sub_stateMoveCallback, this);

   _map_frame = map_frame;
   _robot_frame = robot_frame;
   _stopDuration = ros::Duration(obstacle_block_time);


   rona::map::Point2D p0(ob_react_poly_0_x, ob_react_poly_0_y);
   rona::map::Point2D p1(ob_react_poly_1_x, ob_react_poly_1_y);
   rona::map::Point2D p2(ob_react_poly_2_x, ob_react_poly_2_y);
   rona::map::Point2D p3(ob_react_poly_3_x, ob_react_poly_3_y);
   _reactPolygon.points = { p0, p1, p2, p3 };

   //_moveHandler = rona::SironaMoveHandler::getInstance();

   _moveTimer = _nh.createTimer(_stopDuration, &SironaSM::timer_moveCallback, this, true, false);
   _moveTimer.stop();

   _state = State::IDLE;  //only to satisfy eclipse
//   _state_msg.state = _state_msg.IDLE;
//   _state_msg.state_str = "IDLE";
   this->setState(State::IDLE);

   _oldMoveState = false;
}

SironaSM::~SironaSM()
{
}

void SironaSM::start(const double duration)
{
   //init timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &SironaSM::timerLoop_callback, this);
   ros::spin();
}

void SironaSM::timerLoop_callback(const ros::TimerEvent& e)
{
   _state_msg.estimatedArival = 0;
   _state_msg.targetDistance = 0;
   _pubState.publish(_state_msg);
}


void SironaSM::sub_targetCallback(const geometry_msgs::PoseStamped& msg)
{
   if(!_pathTypeQueue.empty())
   {
      ROS_WARN("sirona_sm_node -> other planning in process.. but will continue");
   }
//   if(_isPlanning)
//   {
//      ROS_WARN("sirona_sm_node -> Ignore current Target... is currently planning");
//      return;
//   }

   //trigger path planning
   _pubRelayTarget.publish(msg);

   //set flag-> currentPlanning
      //for now block further targets...
   //_isPlanning = true;
   _pathTypeQueue.push(PathType::NORMAL);

   //clear moveNode - stop moving
   this->stopMove();

}

void SironaSM::timer_moveCallback(const ros::TimerEvent& e)
{
   ROS_INFO("TimerCallback -> unpause PathController");
   _moveTimer.stop();
   //continue moving
   this->unpauseMove();
}

void SironaSM::sub_addObCallback(const rona_msgs::Obstacle& msg)
{
   ROS_INFO("sirona_sm -> addObstacle");
   //re pub ob
   _pubRelayAddOb.publish(msg);

   if(_state != State::MOVING && _state != State::PAUSED)
   {
      return;
   }

   //prove in polygon
   //transform ob polygon in robot frame
   rona::map::Polygon obstacle = rona::Utility::toPolygon(rona::Utility::transformPoylgon(_tf_listnener, _robot_frame, msg.polygon));
   //prove if one ob_point is in robot safety polygon
   for(auto& e : obstacle.points)
   {
      if(rona::map::Operations::pointInPolygon(_reactPolygon,e))
      {
         //wait
         ROS_INFO("sirona_sm -> addObstacle->wait");
        this->pause_auto_unpauseMove();
         break;
      }
   }

   //replan
   std_msgs::Bool rep_msg;
   rep_msg.data = true;

   _pathTypeQueue.push(PathType::REPLAN);
   ROS_INFO("sirona_sm -> addObstacle->replan");
   _pubReplan.publish(rep_msg);
}
void SironaSM::sub_rmObCallback(const std_msgs::String& msg)
{
   ROS_INFO("sirona_sm -> rmObstacle");
   //re pub ob
   _pubRelayRmOb.publish(msg);

   if(_state != State::MOVING && _state != State::PAUSED)
   {
      return;
   }

   //replan
   std_msgs::Bool rep_msg;
   rep_msg.data = true;

   _pathTypeQueue.push(PathType::REPLAN);
   ROS_INFO("sirona_sm -> rmObstacle->replan");
   _pubReplan.publish(rep_msg);
}

void SironaSM::sub_pathCallback(const nav_msgs::Path& msg)
{
   nav_msgs::Path path_t = msg;
   ROS_INFO("rona_sm -> PathCallback");

   if(_pathTypeQueue.empty())
   {
      ROS_WARN("sirona_sm_node -> pathTypeQueue empty ... should not happen");
   }
   //reset flag -> current planning
   //_isPlanning = false;
   PathType type = _pathTypeQueue.front();
   _pathTypeQueue.pop();

   ROS_INFO("sirona_smnode-> type of callback: %s", (type == PathType::NORMAL ? "NORMAL" : "REPLAN"));

   //repub path to move


   //save last ori
   auto ori = path_t.poses.back().pose.orientation;

   path_t.poses.back().pose.orientation = ori;

//   //hack for exploratoin...
//   if(path_t.poses.size() > 40)
//   {//cut path ... last 75 elements...
//     path_t.poses.erase((path_t.poses.end() - 30), path_t.poses.end());
//   }
//   //hack end...
   _pubPathMove.publish(path_t);

   if(type == PathType::NORMAL)
   {//start moving only when normal mode
      if(path_t.poses.size() <= 1)
      {//empty
         this->setState(State::UNREACHABLE);

         this->stopMove();
      }
      else
      {
         this->startMove();
         this->setState(State::MOVING);
      }
   }
   else if(type == PathType::REPLAN)
   {
     //todo when replan fails .... set state to aport
     if(path_t.poses.size() <= 1)
     {//empty
        this->setState(State::ABORTED);

        this->stopMove();
     }
     else if(_state == State::MOVING)
     {
       this->startMove();
     }
   }
}

void SironaSM::sub_stateMoveCallback(const std_msgs::Bool& msg)
{
   if(_state != State::MOVING && _state != State::PAUSED)
   {
      return;
   }

   //prove state of move
   if(!_oldMoveState && msg.data)
   {//arrived edge
      this->setState(State::ARRIVED);

   }
   _oldMoveState = msg.data;
}

void SironaSM::sub_obstacleGrid(const nav_msgs::GridCells& msg)
{
  //get gridcells

  //prove conflict with path within given distance ...

  //if collission
    //abort
  //else
    //do nothing ... all ok...

}


void SironaSM::startMove()
{
   ROS_INFO("rona_sm -> StartMove");
   rona_msgs::NodeCtrl ctrl_msg;
   ctrl_msg.cmd = ctrl_msg.START;
   ctrl_msg.cmd_str = "START";
   _pubMoveCtrl.publish(ctrl_msg);
}

void SironaSM::stopMove()
{
   ROS_INFO("rona_sm -> Stop Move ");
   rona_msgs::NodeCtrl ctrl_msg;
   ctrl_msg.cmd = ctrl_msg.STOP;
   ctrl_msg.cmd_str = "STOP";
   _pubMoveCtrl.publish(ctrl_msg);
}

void SironaSM::pauseMove()
{
   if(_state != State::MOVING && _state != State::PAUSED)
   {
      return;
   }
   ROS_INFO("rona_sm -> PauseMove");
   this->setState(State::PAUSED);


   rona_msgs::NodeCtrl msg;
   msg.cmd = msg.PAUSE;
   msg.cmd_str = "PAUSE";
   _pubMoveCtrl.publish(msg);
}

void SironaSM::pause_auto_unpauseMove()
{
   this->pauseMove();
   //start timer for auto unpause
   _moveTimer.setPeriod(_stopDuration); //restart in case it is already running
   _moveTimer.start();
}

void SironaSM::unpauseMove()
{
   if(_state != State::MOVING && _state != State::PAUSED)
   {
      return;
   }
   ROS_INFO("rona_sm -> UnPauseMove");
   this->setState(State::MOVING);

   rona_msgs::NodeCtrl msg;
   msg.cmd = msg.CONTINUE;
   msg.cmd_str = "CONTINUE";
   _pubMoveCtrl.publish(msg);
}

void SironaSM::setState(const State state)
{
  _state = state;

  switch (state) {
    case State::IDLE:
      _state_msg.state = _state_msg.IDLE;
      _state_msg.state_str = "IDLE";
      break;
    case State::MOVING:
      _state_msg.state = _state_msg.MOVING;
      _state_msg.state_str = "MOVING";
      break;
    case State::PAUSED:
      _state_msg.state = _state_msg.PAUSED;
      _state_msg.state_str = "PAUSED";
      break;
    case State::ARRIVED:
      _state_msg.state = _state_msg.ARRIVED;
      _state_msg.state_str = "ARRIVED";
      break;
    case State::ABORTED:
      _state_msg.state = _state_msg.ABORTED;
      _state_msg.state_str = "ABORTED";
      break;
    case State::UNREACHABLE:
      _state_msg.state = _state_msg.UNREACHABLE;
      _state_msg.state_str = "UNREACHABLE";
      break;
//    case State::BLOCKED:
//      _state_msg.state = _state_msg.ABORTED;
//      _state_msg.state_str = "ABORTED";
//      break;
    default:
      ROS_ERROR("Undefined sate in Sirona SM setState()");
      break;
  }
  //todo
  _state_msg.estimatedArival = 13.37;
  _state_msg.targetDistance = 47.11;

}

//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sirona_sm_node");
    ros::NodeHandle nh("~");

    SironaSM node;
    node.start(0.1);

}


