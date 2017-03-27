
#include "MuronaSimpleNode.h"

MuronaSimpleNode::MuronaSimpleNode()
{
   ros::NodeHandle privNh("~");
   //std::string string_value;
   //double double_value;
   //int int_val;
   double max_share_path_lenght;
   double average_vel;
   double refresh_motion;
   double time_margin_collision;
   double robot_radius;
   double path_length_diff_fak;
   double color_r;
   double color_g;
   double color_b;
   double color_a;
   int robot_id;
   int robot_prio;
   std::string pub_motion_topic;
   std::string pub_path_topic;
   std::string pub_marker_topic;
   std::string pub_state_topic;
   std::string pub_move_ctrl_topic;
   std::string sub_target_topic;
   std::string sub_map_topic;
   std::string sub_motion_topic;
   std::string sub_move_state_topic;
   std::string sub_move_process_topic;
   std::string robot_frame;
   std::string map_frame;

   //privNh.param("string_value",string_value,std::string("std_value"));
   //privNh.param<double>("double_value",double_value, 12.34);
   //privNh.param<int>("int_val",int_val, 1234);


   privNh.param("pub_motion_topic"        , pub_motion_topic,          std::string("/robot_motion"    ));
   privNh.param("pub_path_topic"          , pub_path_topic,            std::string("rona/move/path"   ));
   privNh.param("pub_marker_topic"        , pub_marker_topic,          std::string("collision_marker" ));
   privNh.param("pub_state_topic"         , pub_state_topic,           std::string("planner/state"    ));
   privNh.param("pub_move_ctrl_topic"     , pub_move_ctrl_topic,       std::string("rona/move/ctrl"   ));
   privNh.param("sub_target_topic"        , sub_target_topic,          std::string("target"           ));
   privNh.param("sub_map_topic"           , sub_map_topic,             std::string("/map"             ));
   privNh.param("sub_motion_topic"        , sub_motion_topic,          std::string("/robot_motion"    ));
   privNh.param("sub_move_state_topic"    , sub_move_state_topic,      std::string("rona/move/state"  ));
   privNh.param("sub_move_process_topic"  , sub_move_process_topic,    std::string("rona/move/process"));
   privNh.param("robot_frame"             , robot_frame,               std::string("robot0"           ));
   privNh.param("map_frame"               , map_frame,                 std::string("map"              ));

   privNh.param<double>("max_share_path_lenght", max_share_path_lenght,  3.0 );
   privNh.param<double>("average_vel"          , average_vel,            0.2 );
   privNh.param<double>("refresh_motion"       , refresh_motion,         2.0 );
   privNh.param<double>("time_margin_collision", time_margin_collision,  3.0 );
   privNh.param<double>("robot_radius"         , robot_radius,           0.3 );
   privNh.param<double>("path_length_diff_fak" , path_length_diff_fak,   1.2 );
   privNh.param<double>("color_r"              , color_r,                1.0 );
   privNh.param<double>("color_g"              , color_g,                0.0 );
   privNh.param<double>("color_b"              , color_b,                0.5 );
   privNh.param<double>("color_a"              , color_a,                0.5 );
   privNh.param<int>(   "robot_id"             , robot_id,               0   );
   privNh.param<int>(   "robot_prio"           , robot_prio,             0   );

   _refreshMotion = ros::Duration(refresh_motion);

   _maxPathLength        = max_share_path_lenght;
   _time_margin_collison = time_margin_collision;
   _pathLengthDiffFak    = path_length_diff_fak;
   _robotRadius          = robot_radius;
   _robot_id    = robot_id;
   _robot_prio  = robot_prio;
   _robot_frame = robot_frame;
   _map_frame   = map_frame;

   _color.r = color_r;
   _color.g = color_g;
   _color.b = color_b;
   _color.a = color_a;

   //init publisher
   //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
   _pubMotion       = _nh.advertise<rona_msgs::RobotMotion>(pub_motion_topic, 1000);
   _pubPathMove     = _nh.advertise<nav_msgs::Path>(pub_path_topic, 1000);
   _pubState        = _nh.advertise<rona_msgs::State>(pub_state_topic, 10);
   _pubMarker       = _nh.advertise<visualization_msgs::MarkerArray>(pub_marker_topic, 1);
   _pubMoveCtrl     = _nh.advertise<rona_msgs::NodeCtrl>(pub_move_ctrl_topic, 1000);


   //inti subscriber
   //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
   _subTarget      = _nh.subscribe(sub_target_topic, 10, &MuronaSimpleNode::subTarget_callback, this);
   _subMap         = _nh.subscribe(sub_map_topic, 1, &MuronaSimpleNode::subMap_callback, this);
   _subMotion      = _nh.subscribe(sub_motion_topic, 100, &MuronaSimpleNode::subMotion_callback, this);
   _subMoveState   = _nh.subscribe(sub_move_state_topic, 1, &MuronaSimpleNode::subStateControl_callback, this);
   _subMoveProcess = _nh.subscribe(sub_move_process_topic, 1, &MuronaSimpleNode::subMoveProcess_callback, this);

   _timerRepubMotion = _nh.createTimer(_refreshMotion, &MuronaSimpleNode::timerRepubMotion_callback, this);

   _stateMove_old = false;
   _repubMotion = false;
   _currentMovePorcess = 0;
   _expectedArival = 0;
   _length_path_curr = 0;

   _planner = std::unique_ptr<rona::planner::AStar>(new rona::planner::AStar);
   _motionPredictor = std::unique_ptr<rona::motion::MotionPredictor>(new rona::motion::MotionPredictor(average_vel));
   //_conflictAnalyser = std::unique_ptr<rona::motion::ConflictAnalyser>(new rona::motion::ConflictAnalyser);

   ROS_INFO("Robot %d Constructor Planner rdy",_robot_id);
}

MuronaSimpleNode::~MuronaSimpleNode()
{
}

void MuronaSimpleNode::start(const double duration)
{
   //init timer
   //_timerRepubMotion = _nh.createTimer(ros::Duration(duration), &MuronaSimpleNode::timerLoop_callback, this);
   ros::spin();
}

void MuronaSimpleNode::timerRepubMotion_callback(const ros::TimerEvent& e)
{
   if(!_repubMotion)
      return;
   //repub motion
   _pubMotion.publish(this->computeRobotMotion(_path_curr));
}

void MuronaSimpleNode::subTarget_callback(const geometry_msgs::PoseStamped& msg)
{
   ROS_INFO("Got Target -> Plan path:");
   if(!_map)
   {
      ROS_WARN("Got no map until yet... will not accept mission");
      return;
   }
   _planner->setMap(_map);

   //plan path with raw_map(without other robots)
   //get robot pose
   rona::map::Point2D start = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);
   rona::map::Point2D end = rona::Utility::toPoint2D(msg);
   _currentTarget = end;
   _currentTarget_orientation = msg.pose.orientation;

   std::cout << "start: " << start << std::endl;
   std::cout << "end: " << end << std::endl;
   std::cout << "startPixel: " << _map->getGrid()->toPixel(start) << std::endl;
   std::cout << "endPixel: " <<_map->getGrid()->toPixel(end) << std::endl;

   rona::map::Node n_start;
   rona::map::Node n_end;
   n_start.id = _map->getGrid()->toIdx(start);
   n_start.pos = start;
   n_end.id = _map->getGrid()->toIdx(end);
   n_end.pos = end;

   rona::map::Path path = _planner->computePath(n_start, n_end);

   //debug

   cv::Mat cvmap = _map->getGrid()->toCvMat();
   cv::imwrite("/tmp/initmap.png", cvmap);


   _path_raw = path;
   _path_curr = path;
   ROS_INFO("CurrPathLenght: %d", _path_curr.size());
   _length_path_curr = rona::map::Operations::computePathLength(path);

   if(!path.size())
   {
      ROS_INFO("--- NO Path found");
   }
   else
   {
      //enable replan if first path was found
//      _replanRdy = true;
   }
   ROS_INFO("Path found: num nodes: %d", (int)path.size());

   //compute Motion
   rona_msgs::RobotMotion robot_motion = this->computeRobotMotion(path);
   ROS_INFO("Expected Arival: %f", _expectedArival);

   //pub motion
   _pubMotion.publish(robot_motion);
   _repubMotion = true;

   //pub path to path controller
   _pubPathMove.publish(robot_motion.path);
   //start moving:
   this->startMove();
   ROS_INFO("TargetCallback -> RDY");
}

void MuronaSimpleNode::subMap_callback(const nav_msgs::OccupancyGrid& msg)
{
   if(!_map)
   {//init stuff
      ROS_INFO("Got Map");

      _map = std::shared_ptr<rona::map::GridMap>(new rona::map::GridMap(msg));
      rona::map::Operations::inflateCirc(_map->getGrid(), 10, 127, _robotRadius);
      rona::map::Operations::binarize(_map->getGrid(), 0, 1, 0, 255);

      //dt
      std::shared_ptr<rona::map::Grid> costmap(new rona::map::Grid(_map->getGrid()));
      rona::map::Operations::distnaceTransformCirc(costmap, 0.15, 255);
      _map->addCostMap("dt_map", costmap);

      _currMap = _map;
      auto map_conflict = std::shared_ptr<rona::map::Grid>(new rona::map::Grid(_map->getGrid()));
      _conflictAnalyser = std::unique_ptr<rona::motion::ConflictAnalyser>(new rona::motion::ConflictAnalyser(map_conflict, _robotRadius, _time_margin_collison, _maxPathLength));
   }else
   {//update map
      _map = std::shared_ptr<rona::map::GridMap>(new rona::map::GridMap(msg));
      rona::map::Operations::inflateCirc(_map->getGrid(), 10, 127, _robotRadius);
      rona::map::Operations::binarize(_map->getGrid(), 0, 1, 0, 255);
      //todo check if good... normalerweise müssen noch mapobj reingezeichnet werden egal für simulation
      //_currMap = _map;
   }
}

void MuronaSimpleNode::subMotion_callback(const rona_msgs::RobotMotion& msg)
{
   /*
   Gedanken experiment:
   konflikt pfüfen nur mit niedriger prio
   wenn konflikt (zeit und ort)

   nur nächsten konflikt berücksichtigen

   falls konflikt ... // fragen ob konflikt patner ankommt (ankommen heist pfad bei neupalnung darf nich länger als schwellwert-faktor sein)
   (bei konflikt niedriger priorer sollte stehen bleiben -> somit als static obstacle eintragen oder so)
   falls ja
      sofort stehen bleiben
      bis dieser konflikt nicht mehr da ist dann wieder fahren.
   falls nein //sackgasse oder so...
      prio wechsel -> bisher kein priowechsel.... vllt später ... jetzt konzept von ressourcen managment (mutex)


   --

   Vorhehen in sw(Gedanken experiment):

   nach jeder empfangenen Motion conflikt prüfen

   Konflikte merken/speichern...


   */

   ros::Time t_start = ros::Time::now();

   if(!_map)
   {
      ROS_WARN("Got no map until yet... will not handle robot Motion");
      return;
   }

   //ROS_INFO("Got robotMotion from id: %d", msg.id);
   if(msg.id == _robot_id)
   {//dont process own motion
      return;
   }

   //ROS_INFO("CurrPathLenght: %d", _path_curr.size());

   if(msg.priority > _robot_prio)   //if lower prio robot
   {
      if(!this->isRobotInRange(rona::Utility::toPoint2D(msg.robotPose)))
      {
         //not in range set as free ( egal ob static oder nicht)
         if(!this->isRobotObj(msg.id))
         {//insert as free
            _robot_types.insert(std::make_pair(msg.id, rona::map::Object::FREE));
         }
         else if(_robot_types.at(msg.id) == rona::map::Object::ROBOT_OBJECT)
         {//now free
            _robot_types.at(msg.id) = rona::map::Object::FREE;
            this->robotIsFree_event(msg);
         }
         //ros::Time t_end = ros::Time::now();
         //ROS_INFO("robot%d : Time for computeing Motion: %f", _robot_id, (t_end - t_start).toSec());
         return;
      }

      //if static robot add as obstacle
      //only lower prio robots can be static obstacle
      if(msg.isStatic)
      {
         if(!this->isRobotObj(msg.id))
         {//insert in robot_obj
            _robot_types.insert(std::make_pair(msg.id, rona::map::Object::ROBOT_OBJECT));

            //fire robot static event
            this->robotIsStaic_event(msg);
         }
         if(_robot_types.at(msg.id) != rona::map::Object::ROBOT_OBJECT)
         {
            //fire robot static event
            _robot_types.at(msg.id) = rona::map::Object::ROBOT_OBJECT;
            this->robotIsStaic_event(msg);
         }
      }
      else
      {//non static.. prove event to set robot as free
         if(!this->isRobotObj(msg.id))
         {//insert robot in robot_obj as free
            _robot_types.insert(std::make_pair(msg.id, rona::map::Object::FREE));
         }
         else if(_robot_types.at(msg.id) == rona::map::Object::ROBOT_OBJECT)
         {
            //fire robot free event
            _robot_types.at(msg.id) = rona::map::Object::FREE;
            this->robotIsFree_event(msg);
         }
      }
      //ros::Time t_end = ros::Time::now();
      //ROS_INFO("robot%d : Time for computeing Motion: %f", _robot_id, (t_end - t_start).toSec());
      return;
   }


   //if higher prio robot
   //prove conflict
   rona::map::Path path_partner = rona::Utility::toRonaPath(msg.path);
   //std::cout << "partnerPath: "<< path_partner << std::endl;
   rona::motion::Motion motion_abs_partner = msg.motion_abs;
   rona::motion::Conflict conf = _conflictAnalyser->analyseConflict(_motion_abs,
                                                                    _path_curr,
                                                                    _currentMovePorcess,
                                                                    motion_abs_partner,
                                                                    path_partner,
                                                                    msg.process);

   //pubCollissions
   this->pubMarker(conf.poses);
   //--

   if(!conf.isValid) //if no coll
   {
      if(this->isCollission(msg.id))
      {
         this->noWait_event(msg.id);
      }
      //no conflicts do nothing
      return;
   }

   //prove is already waiting for this robot
   if(this->isCollission(msg.id))
   {//no need to replan just wait untill no conf
      return;
   }

   //conflicts detected....
   //only higer prior robots...

   //try to replan with path of the other robot
   //if path not found or found path longer then x*fac
   //  -> wait
   //  -> remove path from planner map
   //  -> set itself as static

   //  -> todo check... try to remove all partner path... cause not needed maybe ... because of waiting
   //else
   // -> add partner path to mapObjects...
   // -> new path is std_path from now on


   //do replan
   //copy map for tmp path drawing
   auto currPlanMap = std::shared_ptr<rona::map::GridMap>(new rona::map::GridMap(_currMap));


   //truncate partner path
   rona::map::Path trunc_path_partner;
   if(msg.process < path_partner.size())
   {
      for(unsigned int i=msg.process; i<path_partner.size(); ++i)
      {
         trunc_path_partner.push_back(path_partner[i]);
      }
   }
   else{
      ROS_WARN("Partnerpath size(%d) smaller than process(%d) ... ",path_partner.size(), msg.process);
   }


   rona::map::Point2D currPos = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);


   auto tmp_path_obj = std::shared_ptr<rona::map::PathObject>(new rona::map::PathObject(trunc_path_partner, _robotRadius));
   tmp_path_obj->setRobotPos(currPos);
   tmp_path_obj->draw(currPlanMap->getGrid());

   _planner->setMap(currPlanMap);


   rona::map::Path tmp_path;
   rona::map::Node node_s;
   node_s.id = currPlanMap->getGrid()->toIdx(currPos);
   node_s.pos = currPos;
   tmp_path = _planner->recomputePath(node_s);


   cv::Mat cvmap = currPlanMap->getGrid()->toCvMat();
   cv::imwrite("/tmp/tmpplanmap.png", cvmap);

   double length_path_curr = rona::map::Operations::computePathLength(_path_curr, _currentMovePorcess);
   double length_path_new  = rona::map::Operations::computePathLength(tmp_path);

   ROS_INFO("Old Pathlength: %f, pathSize: %d", length_path_curr, _path_curr.size());
   ROS_INFO("New Pathlenght: %f, pathSize: %d", length_path_new, tmp_path.size());

   //prove wait or move around
   if(length_path_new >= length_path_curr * _pathLengthDiffFak || tmp_path.size() == 0)
   {//longer path found stop
      //wait...
      if(this->isCollission(msg.id))
      {// is already collission
         //do nothing
      }
      else
      {
         //fire wait event
         this->wait_event(msg.id);
      }

      //remove map from planner (replace with old one)
      _planner->setMap(_currMap);
   }
   else
   {//path ok to move
      //add partner path to obj list
      MapObject mo;
      mo.id = msg.id;
      mo.type = rona::map::Object::PATH_OBJECT;
      //prove if already exist
      try {
         _map_objects.at(mo);
         //already exist
         _map_objects.at(mo) = tmp_path_obj;
      } catch (std::out_of_range& e) {
         //if not existing just insert
         _map_objects.insert(std::make_pair(mo, tmp_path_obj));
      }

      this->doReplan();
   }

   ROS_INFO("RDY-MoveCallback");

}

void MuronaSimpleNode::subStateControl_callback(const std_msgs::Bool& msg)
{
//   if(_state != State::MOVING && _state != State::PAUSED)
//   {
//      return;
//   }

   //prove state of move
   if(!_stateMove_old && msg.data)
   {//arrived edge
//      _state = State::ARRIVED;
//      _state_msg.state = _state_msg.ARRIVED;
//      _state_msg.state_str = "ARRIVED";
      ROS_INFO("Arrived");
      _repubMotion = false;

      //todo may add robot as static obstacle...
   }
   _stateMove_old = msg.data;
}

void MuronaSimpleNode::subMoveProcess_callback(const std_msgs::UInt32& msg)
{
   _currentMovePorcess = msg.data;
}


void MuronaSimpleNode::subAddOb_callback(const rona_msgs::Obstacle& msg)
{
   //add ob
}

void MuronaSimpleNode::subRmOb_callback(const std_msgs::String& msg)
{

}

rona_msgs::RobotMotion MuronaSimpleNode::computeRobotMotion(rona::map::Path path)
{
   rona_msgs::RobotMotion ros_motion;

   //ROS_INFO("---Current MoveProcess: %d", _currentMovePorcess);
   //update motion_abs
   _motion     = _motionPredictor->computeMotionPrediction(path);
   _motion_abs = _motionPredictor->toAbsolutTimeMotion(_motion, ros::Time::now().toSec(), _currentMovePorcess);


   ros_motion.id = _robot_id;
   ros_motion.isStatic = _collissions.size() ? true : false; //if size 0 then false...
   ros_motion.motion = _motion;
   ros_motion.motion_abs = _motion_abs;
   ros_motion.path = rona::Utility::toRosPath(_path_curr, _map_frame);
   ros_motion.robotPose = rona::Utility::toRosPoseStamped(rona::Utility::getTransformPoint2D(_tf_listnener, _map_frame, _robot_frame), _map_frame);
   ros_motion.process = _currentMovePorcess;

   if(ros_motion.motion_abs.size() > 0)
      _expectedArival = ros_motion.motion_abs[ros_motion.motion_abs.size() - 1];

   ros_motion.priority = _robot_prio;

   return ros_motion;
}


void MuronaSimpleNode::startMove()
{
   ROS_INFO("murona -> StartMove");
   rona_msgs::NodeCtrl ctrl_msg;
   ctrl_msg.cmd = ctrl_msg.START;
   ctrl_msg.cmd_str = "START";
   _pubMoveCtrl.publish(ctrl_msg);
}

void MuronaSimpleNode::stopMove()
{
   ROS_INFO("murona -> Stop Move ");
   rona_msgs::NodeCtrl ctrl_msg;
   ctrl_msg.cmd = ctrl_msg.STOP;
   ctrl_msg.cmd_str = "STOP";
   _pubMoveCtrl.publish(ctrl_msg);
}

void MuronaSimpleNode::pauseMove()
{
//   if(_state != State::MOVING && _state != State::PAUSED)
//   {
//      return;
//   }
   ROS_INFO("murona -> PauseMove");
//   _state = State::PAUSED;
//   _state_msg.state = _state_msg.PAUSED;
//   _state_msg.state_str = "PAUSED";

   rona_msgs::NodeCtrl msg;
   msg.cmd = msg.PAUSE;
   msg.cmd_str = "PAUSE";
   _pubMoveCtrl.publish(msg);
}

void MuronaSimpleNode::pause_auto_unpauseMove()
{
   this->pauseMove();
   //start timer for auto unpause
//   _moveTimer.setPeriod(_stopDuration); //restart in case it is already running
//   _moveTimer.start();
}

void MuronaSimpleNode::unpauseMove()
{
//   if(_state != State::MOVING && _state != State::PAUSED)
//   {
//      return;
//   }
   ROS_INFO("murona -> UnPauseMove");
//   _state = State::MOVING;
//   _state_msg.state = _state_msg.MOVING;
//   _state_msg.state_str = "MOVING";

   rona_msgs::NodeCtrl msg;
   msg.cmd = msg.CONTINUE;
   msg.cmd_str = "CONTINUE";
   _pubMoveCtrl.publish(msg);
}


bool MuronaSimpleNode::isRobotInRange(rona::map::Point2D pos_r)
{
   rona::map::Point2D currPose = rona::Utility::getTransformPoint2D(_tf_listnener, _map_frame, _robot_frame);
   double dist = rona::map::Operations::computeDistance(currPose, pos_r);
   if(dist > _maxPathLength)
   {//robot too far away
      return false;
   }
   return true;
}

bool MuronaSimpleNode::isRobotObj(unsigned int id)
{
   try {
      _robot_types.at(id);
   } catch (std::out_of_range& e) {
      return false;
   }
   return true;
}

bool MuronaSimpleNode::isCollission(unsigned int id)
{
   try {
      _collissions.at(id);
   } catch (std::out_of_range& e) {
      return false;
   }
   return true;
}


void MuronaSimpleNode::robotIsStaic_event(
      const rona_msgs::RobotMotion& msg)
{
   ROS_INFO("RobotIsStaic_event() from +++ %d , size: %d", msg.id, _map_objects.size());
   //add robot in map_objects
   MapObject mo;
   mo.id = msg.id;
   mo.type = rona::map::Object::ROBOT_OBJECT;
   auto ro = std::shared_ptr<rona::map::RobotObject>(new rona::map::RobotObject(rona::Utility::toPoint2D(msg.robotPose), _robotRadius*1.5));

   unsigned int dbg_size = _map_objects.size();

   _map_objects.insert(std::make_pair(mo, ro));

   ROS_INFO("size: %d", _map_objects.size());

   if(dbg_size == _map_objects.size())
   {
      ROS_WARN("Size is same after inserting element");
   }

   //replan
   this->doReplan();
}

void MuronaSimpleNode::robotIsFree_event(
      const rona_msgs::RobotMotion& msg)
{
   ROS_INFO("RobotIsFree_event() from --- %d , size: %d", msg.id, _map_objects.size());
   //remove robot from map_objects
   MapObject mo;
   mo.id = msg.id;
   mo.type = rona::map::Object::ROBOT_OBJECT;

   unsigned int dbg_size = _map_objects.size();

   try {
      _map_objects.at(mo);
   } catch (std::out_of_range& e) {
      ROS_WARN("Robot_planner object(id: %d) to delete not exist -> robotIsFree_event()", msg.id);
      //debug output of map:
      std::cout << "identifier: " << std::endl;
      std::cout << "-- id: " << mo.id << ", type: " << static_cast<int>(mo.type) << std::endl;
      std::cout << "_map_object conten__t" << std::endl;
//      for(map_object_container::iterator it = _map_objects.begin(); it!=_map_objects.end(); it++)
//      {
//         std::cout << "-- id: " << it->first.id_robot << ", type: " << it->first.type << ", sec: " << it->second->isObject() << std::endl;
//      }
      try {
         _map_objects.at(mo);
      } catch (std::out_of_range& e) {
         std::cout << "+++ still out of range" << std::endl;
      }
   }

   _map_objects.erase(mo);


   if(dbg_size == _map_objects.size())
   {
      ROS_WARN("Size is same after deleting element");
   }


   //replan
   this->doReplan();
}

void MuronaSimpleNode::wait_event(unsigned int id)
{
   ROS_INFO("wait_event() from --- %d", id);
   //insert to collissions
   _collissions.insert(std::make_pair(id,0));

   //pause controller
   this->pauseMove();
}

void MuronaSimpleNode::noWait_event(unsigned int id)
{
   ROS_INFO("noWait_event() from --- %d", id);
   //remove id from _collission list
   _collissions.erase(id);

   usleep(2000000); //hack sleep 4s

   //enable controller if no collissions (collission.size must be 0)
   if(_collissions.size() == 0)
   {
      //enable controller
      this->unpauseMove();
   }
}

void MuronaSimpleNode::doReplan()
{
   ROS_INFO("Do Replann and repub path");
   //create new map:
   auto currPlanMap = std::shared_ptr<rona::map::GridMap>(new rona::map::GridMap(_map));
   rona::map::Point2D currPos = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);

   for(auto& e : _map_objects)
   {
      if(e.second->whatObject() == rona::map::Object::PATH_OBJECT)
      {
         std::dynamic_pointer_cast<rona::map::PathObject>(e.second)->setRobotPos(currPos);
      }
      e.second->draw(currPlanMap->getGrid());
   }

   //dt
   std::shared_ptr<rona::map::Grid> costmap(new rona::map::Grid(currPlanMap->getGrid()));
   rona::map::Operations::distnaceTransformCirc(costmap, 0.15, 255);
   currPlanMap->addCostMap("dt_map", costmap);

   _planner->setMap(currPlanMap);
   _currMap = currPlanMap;

   //debug
   cv::Mat cvmap = currPlanMap->getGrid()->toCvMat();
   // draw start
   cv::Point x(currPlanMap->getGrid()->toPixel(currPos).x,currPlanMap->getGrid()->toPixel(currPos).y);
   cv::line(cvmap, x, x, cv::Scalar(128), 3);
   x.x = currPlanMap->getGrid()->toPixel(_currentTarget).x;
   x.y = currPlanMap->getGrid()->toPixel(_currentTarget).y;
   cv::line(cvmap, x, x, cv::Scalar(128), 3);
   cv::imwrite("/tmp/replanmap.png", cvmap);

   rona::map::Path new_path;
   rona::map::Node node_s;
   node_s.id = currPlanMap->getGrid()->toIdx(currPos);
   node_s.pos = currPos;
   new_path = _planner->recomputePath(node_s);



   if(new_path.size() == 0)
   {
      ROS_WARN("Error at replanning .... in fct doReplan(), will exit");
      //exit(EXIT_FAILURE);

      //todo pause move is no vlaid path found... an unpause if one is found....

      return;
   }

   //set new path to current path;
   _path_curr = new_path;


   rona_msgs::RobotMotion ros_motion = this->computeRobotMotion(new_path);

   //ROS_INFO("Expected Arival Time: %f", _expectedArival);

   _pubMotion.publish(ros_motion);
   //_repubMotion = true;

   //pub path to path controller
   _pubPathMove.publish(ros_motion.path);
}



void MuronaSimpleNode::pubMarker(std::vector<rona::map::Point2D> p)
{
   visualization_msgs::MarkerArray marker;

   geometry_msgs::Vector3 scale_m;
   scale_m.x = 0.2;
   scale_m.y = 0.2;
   scale_m.z = 0.2;
   std_msgs::ColorRGBA color = _color;



   for(unsigned int i=0; i<p.size(); ++i)
   {
      visualization_msgs::Marker m;

      m.header.frame_id = _map_frame;
      m.id = i;
      m.type = m.CUBE;
      m.action = m.ADD;
      m.pose = rona::Utility::toRosPoseStamped(p[i]).pose;  //orientation not used
      m.scale = scale_m;
      m.color = color;

      marker.markers.push_back(m);
   }

   _pubMarker.publish(marker);
}


//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "murona_node");
    ros::NodeHandle nh("~");

    MuronaSimpleNode node;
    node.start(0.1);

}


