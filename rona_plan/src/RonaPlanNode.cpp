
#include "RonaPlanNode.h"

RonaPlanNode::RonaPlanNode()
{
  ROS_INFO("Started rona_plan_node");
  //rosParam
  ros::NodeHandle privNh("~");
  //std::string string_value;
  //double double_value;
  //int int_val;
  std::string map_frame;
  std::string robot_frame;

  double robot_radius;
  double dt_radius;
  double free_robot_pos_factor;
  int map_scale;
  bool continuous_map_operation;

  //privNh.param("string_value",string_value,std::string("std_value"));
  //privNh.param<double>("double_value",double_value, 12.34);
  //privNh.param<int>("int_val",int_val, 1234);
  privNh.param        ("map_frame"                , map_frame                , std::string("map"));
  privNh.param        ("robot_frame"              , robot_frame              , std::string("base_footprint"));
  privNh.param<double>("robot_radius"             , robot_radius             , 0.35);                  //[m]
  privNh.param<double>("dt_radius"                , dt_radius                , 0.2);                         //[m]
  privNh.param<double>("free_robot_pos_factor"    , free_robot_pos_factor    , 0.5); ///< not save user must be knowing what he is doing... advanced
  privNh.param<int>   ("map_scale"                , map_scale                , 4);
  privNh.param<bool>  ("continuous_map_operation" , continuous_map_operation , false);

  ROS_INFO("rona_plan_node -> Params:");
  ROS_INFO_STREAM("rona_plan_node -> map_frame: " << map_frame );
  ROS_INFO_STREAM("rona_plan_node -> robot_frame: " << robot_frame );
  ROS_INFO_STREAM("rona_plan_node -> robot_radius: " << robot_radius );
  ROS_INFO_STREAM("rona_plan_node -> dt_radius: " << dt_radius );
  ROS_INFO_STREAM("rona_plan_node -> free_robot_pos_factor: " << free_robot_pos_factor );
  ROS_INFO_STREAM("rona_plan_node -> map_scale: " << map_scale );
  ROS_INFO_STREAM("rona_plan_node -> continuous_map_operation: " << std::boolalpha << continuous_map_operation );

  _map_frame = map_frame;
  _robot_frame = robot_frame;

  _robot_radius = robot_radius;
  _dt_radius = dt_radius;

  if (std::abs(free_robot_pos_factor) > 1)
    free_robot_pos_factor = 1;
  _free_robot_pos_factor = std::abs(free_robot_pos_factor);

  _map_scale = map_scale;
  _continuous_map_operation = continuous_map_operation;

  _map_cnt = 0;
  _map_op_cnt = 0;

  _replanRdy = false;

  //init publisher
  //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
  _pubPath      = _nh.advertise<nav_msgs::Path>("rona/plan/path", 2);
  _pubGrid      = _nh.advertise<nav_msgs::GridCells>("rona/plan/inflate_grid", 1, true);
  _pubGridOb    = _nh.advertise<nav_msgs::GridCells>("rona/paln/obstacle_grid", 1, true);
  _pubMapCost   = _nh.advertise<nav_msgs::OccupancyGrid>("rona/plan/cost_map", 1, true);
  _pubMapResize = _nh.advertise<nav_msgs::OccupancyGrid>("rona/plan/map_resize", 1, true);

  //inti subscriber
  //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
  _subMap = _nh.subscribe("map", 1, &RonaPlanNode::sub_mapCallback, this);
  _subTarget = _nh.subscribe("rona/plan/target", 2, &RonaPlanNode::sub_targetCallback, this);
  _subReplan = _nh.subscribe("rona/plan/replan", 2, &RonaPlanNode::sub_replanCallback, this);
  _subAddObstacle = _nh.subscribe("rona/plan/add_obstacle", 2, &RonaPlanNode::sub_addObCallback, this);
  _subRmObstacle = _nh.subscribe("rona/plan/rm_obstacle", 2, &RonaPlanNode::sub_rmObCallbach, this);

  _srv_plan_path = _nh.advertiseService("/rona/plan/path", &RonaPlanNode::srv_plan_callback, this);
  _srv_plan_path_multiple = _nh.advertiseService("/rona/plan/path_multiple", &RonaPlanNode::srv_plan_multiple_callback, this);
}

RonaPlanNode::~RonaPlanNode()
{
}

void RonaPlanNode::start(const double duration)
{
  ros::spin();
}

void RonaPlanNode::debug_save_as_img(std::string file,
      std::shared_ptr<rona::map::Grid> grid, std::vector<rona::map::Node> path)
{
   if(!grid)
   {
      ROS_ERROR("rona_plan_node -> debug_save_as_img: no valid grid given...");
      return;
   }
  //debug: draw path in cv::Mat
  cv::Mat cvmap = grid->toCvMat();

  cv::cvtColor(cvmap, cvmap, CV_GRAY2RGB);

  if(path.size())
  {
     rona::map::Pixel p = grid->toPixel(path[0].pos);
     cv::Point old;
     cv::Point curr;
     old.x = p.x;
     old.y = p.y;
     //std::cout << "PixelPath: " << std::endl;
     for(unsigned int i = 1; i < path.size(); ++i)
     {
        p = grid->toPixel(path[i].pos);
//         std::cout << p;// << " ,(" << path[i].pos.x << "," << path[i].pos.y << ")" <<std::endl;
//         ROS_INFO(", (%f, %f)", path[i].pos.x, path[i].pos.y);
        curr.x = p.x;
        curr.y = p.y;
        cv::line(cvmap, curr, old, cv::Scalar(0,255,0),2);
        old = curr;
     }

  }

  //cv::flip(cvmap, cvmap, -1); //old
  //cv::flip(cvmap, cvmap, 1);  //old
  cv::flip(cvmap, cvmap, 0);  //flip rows
  cv::imwrite(file.c_str(),cvmap);
}

std::shared_ptr<rona::map::GridMap> RonaPlanNode::doMapOperations(
      const std::weak_ptr<rona::map::GridMap> map_raw)
{
  if(map_raw.expired())
  {
     ROS_ERROR("rona_plan_node -> Given Map in doMapOperations is expired... will exit");
     exit(EXIT_FAILURE);
  }

  rona::Timer_auto_ms timer("rona_plan_node -> MapOperations: ");
  //copy new map...
  auto map = std::make_shared<rona::map::GridMap>(map_raw.lock());

  //for visual inflation
  auto grid_raw = std::make_shared<rona::map::Grid>(map->getGrid());
  rona::map::Operations::binarize(*grid_raw, 0,10,0,255);
  //this->debug_save_as_img("/tmp/grid_raw.png", grid_raw);
  //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_extended(map_raw.lock()));
  //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_extended2(map_raw.lock()));
  //std::shared_ptr<rona::map::GridMap> map(new rona::map::GridMap_reduced(map_raw.lock()));

  this->debug_save_as_img("/tmp/map_raw.png", map->getGrid());

  //draw obstacles in map;
  //with dt on ob
  //--
  for(auto& e : _obstacles)
  {
    rona::map::Operations::drawFilledPolygon(*map->getGrid(), e.second, 120);
  }
  //--

  //vis Obstacles...
  std::shared_ptr<rona::map::Grid> grid_map(new rona::map::Grid(map->getGrid()));
  rona::map::Operations::binarize(*grid_map, 0, 10, 0, 255);
  std::shared_ptr<rona::map::Grid> grid_diff_ob = rona::map::Operations::diff(*grid_raw, *grid_map);
  _pubGridOb.publish(grid_diff_ob->toGridCells(200,255,_map_frame));

  ///@note dont inflate unknown area
  rona::map::Operations::inflateCirc(*map->getGrid(), 10, 127, _robot_radius);
  rona::map::Operations::binarize(*map->getGrid(), 0, 10, 0, 255);

  //for visual inflation
  std::shared_ptr<rona::map::Grid> grid_diff = rona::map::Operations::diff(*grid_raw, *map->getGrid());
  this->debug_save_as_img("/tmp/inflated.png", map->getGrid());
  this->debug_save_as_img("/tmp/grid_diff.png", grid_diff);
  _pubGrid.publish(grid_diff->toGridCells(200,255,_map_frame));
  
  //this->debug_save_as_img("/tmp/map.png", map->getGrid());

  //dt
  std::shared_ptr<rona::map::Grid> costmap(new rona::map::Grid(map->getGrid()));
  rona::map::Operations::distnaceTransformCirc(*costmap, _dt_radius, 255);
  
  this->debug_save_as_img("/tmp/map_dt.png", costmap);
  _pubMapCost.publish(costmap->toRosOccGrid());
  //_pubMap.publish(map->getGrid()->toRosOccGrid());
  map->addCostMap("dt_map", costmap);

  return map;
}

bool RonaPlanNode::freeRobot(std::weak_ptr<rona::map::GridMap> map, const rona::map::Point2D robot_pos)
{ 
  if(map.expired())
  {
    ROS_INFO("rona_plan_node -> Map for freeRobot is expired do not free Robot...");
    return false;
  }
  auto smap = map.lock();

  //prove if curr pos is occupied...
  if(smap->isOccupied(smap->getGrid()->toPixel(robot_pos)))
  {
    //free RobotPose if robot new target is out of range
    if(_free_robot_pos_factor > 0.001)
      rona::map::Operations::drawFilledCircle(*smap->getGrid(), robot_pos, _robot_radius * _free_robot_pos_factor, 0);
    return true;
  }
  return false;
}

rona::map::Path RonaPlanNode::computePath(
      const std::weak_ptr<rona::map::GridMap> map,
      const rona::map::Point2D start, rona::map::Point2D end)
{
  if(map.expired())
  {
    ROS_WARN("RonaPlanNode-> computePath() map is expired");
  }
  auto smap = map.lock();
  rona::Timer_auto_ms timer("rona_plan_node -> Planning: ");
  //do planning
  _planner.setMap(smap);

  rona::map::Node n_start;
  rona::map::Node n_end;

  n_start.id = smap->getGrid()->toIdx(start);
  n_start.pos = start;
  n_end.id = smap->getGrid()->toIdx(end);
  n_end.pos = end;

  if(smap->isOccupied(n_end.id))
  {
    return rona::map::Path();
  }


  rona::map::Path path = _planner.computePath(n_start, n_end);
  return path;
}

void RonaPlanNode::sub_mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  if(_map_cnt == 0)
  {
    ROS_INFO("rona_plan_node -> Got Map");

    std::cout << "map.w:   " << msg.info.width << std::endl;
    std::cout << "map.h:   " << msg.info.height << std::endl;
    std::cout << "map.res: " << msg.info.resolution << std::endl;
    std::cout << "map.x:   " << msg.info.origin.position.x << std::endl;
    std::cout << "map.y:   " << msg.info.origin.position.y << std::endl;
  }

  _map_raw = std::make_shared<rona::map::GridMap>(msg);
  _map = std::make_shared<rona::map::GridMap>(msg);
  _map->resize(_map_scale);
  _pubMapResize.publish(_map->getGrid()->toRosOccGrid());
  _map_cnt++;
  if(_continuous_map_operation)
  {
    _map_op = this->doMapOperations(_map);
    _map_op_cnt = _map_cnt;
  }

  // auto reszie2 = rona::map::Grid::resize(map->getGrid(), rona::map::Grid::RESIZE_2X2);
  // this->debug_save_as_img("/tmp/2x2.png", reszie2);
  // auto reszie4 = rona::map::Grid::resize(map->getGrid(), rona::map::Grid::RESIZE_4X4);
  // this->debug_save_as_img("/tmp/4x4.png", reszie4);
  // auto reszie8 = rona::map::Grid::resize(map->getGrid(), rona::map::Grid::RESIZE_8X8);
  // this->debug_save_as_img("/tmp/8x8.png", reszie8);
  //_planner.setMap(_map);
}

void RonaPlanNode::sub_targetCallback(const geometry_msgs::PoseStamped& msg)
{
  if(!_map_cnt)
  {
     ROS_WARN("No Map given until now.... ignore Target");
     return;
  }
  rona::Timer_auto_ms timer("rona_plan_node -> Overall sub_targetCallback: ");
  ROS_INFO("rona_plan_node -> sub_targetCallback() got target");

  //get robot pose
  rona::map::Point2D start = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);
  rona::map::Point2D end = rona::Utility::toPoint2D(msg);
  _lastTarget = end;
  _lastTarget_orientation = msg.pose.orientation;

  std::cout << "start: " << start << std::endl;
  std::cout << "end: " << end << std::endl;
  std::cout << "startPixel: " << _map->getGrid()->toPixel(start) << std::endl;
  std::cout << "endPixel: " <<_map->getGrid()->toPixel(end) << std::endl;
  std::cout << "cellSize: " << _map->getGrid()->getCellSize() << std::endl;
  std::cout << "--- end: " << _map->getGrid()->toPoint2D(_map->getGrid()->toPixel(start)) << std::endl;
  std::cout << "--- start: " << _map->getGrid()->toPoint2D(_map->getGrid()->toPixel(end)) << std::endl;
  if(_map_cnt != _map_op_cnt)
  {
    _map_op = this->doMapOperations(_map);
    _map_op_cnt = _map_cnt;
  }
  
  //to prevent having freed robot space in next plan but maybe costy :)
  auto plan_map = std::make_shared<rona::map::GridMap>(_map_op);
  this->freeRobot(plan_map, start);
  
  rona::map::Path path = this->computePath(plan_map, start, end);

  if(!path.size())
  {
     ROS_INFO("rona_plan_node -> --- NO Path found");
  }
  else
  {
     //enable replan if first path was found
     _replanRdy = true;
  }
  ROS_INFO("rona_plan_node -> Path found: num nodes: %d", (int)path.size());

  //std::cout << path << std::endl;

  //pub path;
  nav_msgs::Path ros_path = rona::Utility::toRosPath(path);
  //set target orientation.
  ros_path.poses[ros_path.poses.size()-1].pose.orientation = msg.pose.orientation;
  _pubPath.publish(ros_path);

  //debug
  this->debug_save_as_img("/tmp/map_path.png", _map->getGrid(), path);
}

void RonaPlanNode::sub_replanCallback(const std_msgs::Bool& msg)
{
  if(!msg.data)
  {
    ROS_WARN("rona_plan_node -> msg false... do nothing");
    return;
  }
  if(!_replanRdy)
  {
    ROS_WARN("rona_plan_node -> Replanning not rdy yet.... no Initital Plan planed... do nothing");
    return;
  }
  rona::Timer_auto_ms timer("rona_plan_node -> Overall sub_replanCallback: ");
  ROS_INFO("rona_plan_node -> sub_replanCallback() got called");

  //get robot pose
  rona::map::Point2D start = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);
  rona::map::Point2D end = _lastTarget;

  std::cout << "start: " << start << std::endl;
  std::cout << "end: " << end << std::endl;
  //std::cout << "startPixel: " << _map->getGrid()->toPixel(start) << std::endl;
  //std::cout << "endPixel: " <<_map->getGrid()->toPixel(end) << std::endl;

  
  if(_map_cnt != _map_op_cnt)
  {
    _map_op = this->doMapOperations(_map);
    _map_op_cnt = _map_cnt;
  }

  //to prevent having freed robot space in next plan but maybe costy :)
  auto plan_map = std::make_shared<rona::map::GridMap>(_map_op);
  this->freeRobot(plan_map, start);
  
  rona::map::Path path = this->computePath(plan_map, start, end);

   if(!path.size())
   {
      ROS_INFO("rona_plan_node -> --- NO Path found");
   }
   ROS_INFO("rona_plan_node -> Path found: num nodes: %d", (int)path.size());

   //pub path;
   nav_msgs::Path ros_path = rona::Utility::toRosPath(path);
   //set target orientation.
   ros_path.poses[ros_path.poses.size()-1].pose.orientation = _lastTarget_orientation;
   _pubPath.publish(ros_path);
}

void RonaPlanNode::sub_addObCallback(const rona_msgs::Obstacle& msg)
{
  try {
    _obstacles.at(msg.identifier) = rona::Utility::toPolygon(msg.polygon);
  } catch (std::out_of_range& e) {
    _obstacles.insert(std::make_pair(msg.identifier, rona::Utility::toPolygon(msg.polygon)));
  }
}

void RonaPlanNode::sub_rmObCallbach(const std_msgs::String& msg)
{
  _obstacles.erase(msg.data);
}

bool RonaPlanNode::srv_plan_callback(rona_msgs::PlanPathRequest& req, rona_msgs::PlanPathResponse& res)
{
  if(!_map_cnt)
  {
     ROS_WARN("rona_plan_node -> No Map given until now.... ignore Target");
     return false;
  }
  rona::Timer_auto_ms timer("rona_plan_node -> Overall srv_targetCallback: ");
  ROS_INFO("rona_plan_node -> srv_plan_callback() got target");

  //get robot pose
  rona::map::Point2D start = rona::Utility::toPoint2D(req.origin);
  rona::map::Point2D end   = rona::Utility::toPoint2D(req.target);

  std::cout << "start: " << start << std::endl;
  std::cout << "end: " << end << std::endl;


  auto used_map = _map_op;
  if(_map_scale != req.scale)
  {
    used_map = std::make_shared<rona::map::GridMap>(_map_raw);
    used_map->resize(req.scale);
    used_map = this->doMapOperations(used_map);
  }
  else if(_map_cnt != _map_op_cnt)
  {
    _map_op = this->doMapOperations(_map);
    _map_op_cnt = _map_cnt;
    used_map = _map_op;
  }
  std::cout << "startPixel: " << used_map->getGrid()->toPixel(start) << std::endl;
  std::cout << "endPixel: " << used_map->getGrid()->toPixel(end) << std::endl;
  //to prevent having freed robot space in next plan but maybe costy :)
  auto plan_map = std::make_shared<rona::map::GridMap>(used_map);
  this->freeRobot(plan_map, start);
  
  rona::map::Path path = this->computePath(plan_map, start, end);
  this->debug_save_as_img("/tmp/srv.png", plan_map->getGrid(), path);

  if(!path.size())
  {
     ROS_INFO("rona_plan_node -> --- NO Path found");
  }
  ROS_INFO("rona_plan_node -> Path found: num nodes: %d", (int)path.size());


  //std::cout << path << std::endl;
  
  //pub path;
  nav_msgs::Path ros_path = rona::Utility::toRosPath(path);
  //set target orientation.
  
  ros_path.poses[ros_path.poses.size()-1].pose.orientation = req.target.orientation;
  res.path = ros_path;
  res.length = rona::map::Operations::computePathLength(path);

  return true;
}

bool RonaPlanNode::srv_plan_multiple_callback(rona_msgs::PlanPathMultipleRequest& req, rona_msgs::PlanPathMultipleResponse& res)
{
  if(!_map_cnt)
  {
     ROS_WARN("rona_plan_node -> No Map given until now.... ignore Target");
     return false;
  }
  rona::Timer_auto_ms timer("rona_plan_node -> Overall srv_targetCallback: ");
  ROS_INFO("rona_plan_node -> srv_targetCallback() got target");

  auto robot_pos = rona::Utility::getTransformPoint2D(_tf_listnener,_map_frame, _robot_frame);

  std::shared_ptr<rona::map::GridMap> used_map;
  if(_map_scale != req.scale)
  {
    used_map = std::make_shared<rona::map::GridMap>(_map_raw);
    used_map->resize(req.scale);
    used_map = this->doMapOperations(used_map);
  }
  else if(_map_cnt != _map_op_cnt)
  {
    _map_op = this->doMapOperations(_map);
    _map_op_cnt = _map_cnt;
    used_map = _map_op;
  }


  unsigned int origin_idx = 0;
  for (unsigned int i = 0; i < req.target.size(); i++)
  {
    rona::map::Point2D start = robot_pos;
    if(i < req.origin.size() && !req.origin.empty())
    {
      origin_idx = i;
      start = rona::Utility::toPoint2D(req.origin[origin_idx]);
    }
    //get robot pose
    rona::map::Point2D end   = rona::Utility::toPoint2D(req.target[i]);
    
    std::cout << " -- " << std::endl;
    std::cout << "start: " << start << std::endl;
    std::cout << "end: " << end << std::endl;
    std::cout << "startPixel: " << used_map->getGrid()->toPixel(start) << std::endl;
    std::cout << "endPixel: " <<used_map->getGrid()->toPixel(end) << std::endl;
    std::cout << "cellSize: " << used_map->getGrid()->getCellSize() << std::endl;

    //to prevent having freed robot space in next plan but maybe costy :)
    auto plan_map = std::make_shared<rona::map::GridMap>(used_map);
    this->freeRobot(plan_map, start);
  
    rona::map::Path path = this->computePath(plan_map, start, end);

    if(!path.size())
    {
       ROS_INFO("rona_plan_node -> --- NO Path found");
    }
    ROS_INFO("rona_plan_node -> Path found: num nodes: %d", (int)path.size());

    //std::cout << path << std::endl;
    auto ros_path = rona::Utility::toRosPath(path);
    //set target orientation.
    ros_path.poses[ros_path.poses.size()-1].pose.orientation = req.target[i].orientation;
    res.path.push_back(ros_path);
    res.length.push_back(rona::map::Operations::computePathLength(path));
  }

   return true;
}

//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_plan_node");
    ros::NodeHandle nh("~");

    RonaPlanNode node;
    node.start(0.1);

}


