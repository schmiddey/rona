
#include "FrontierNode.h"

FrontierNode::FrontierNode()
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string map_frame;
    std::string robot_frame;
//    double      double_val;
//    int         int_val;
//    bool        bool_val;
//
//    privNh.param(         "string_val" ,    string_val,   std::string("string"));
//    privNh.param<double>( "double_val" ,    double_val,   100.0);
//    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
//    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

    privNh.param(         "map_frame"  ,    map_frame,   std::string("map"));
    privNh.param(         "robot_frame" ,  robot_frame,   std::string("base_footprint"));

    rona::frontier::FinderConfig config;
    privNh.param<double>("robot_radius",               config.robot_radius,               0.6);
    privNh.param<double>("min_dist_between_frontiers", config.min_dist_between_frontiers, 1.0);
    privNh.param<double>("max_search_radius",          config.max_search_radius,          10.0);

    _frontierFinder.setConfig(config);
    _frontierController.setTFFrameIds(map_frame, robot_frame);

    //init publisher
    //_pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pubFrontier     = _nh.advertise<geometry_msgs::PoseArray>("rona/frontiers",  1);
    _pubFronteirGrid = _nh.advertise<nav_msgs::GridCells>("rona/frontier_grid", 1);
    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _subMap= _nh.subscribe("map", 1, &FrontierNode::sub_map_callback, this);
    _subNodeCtrl = _nh.subscribe("rona/frontier/node_ctrl", 1, &FrontierNode::sub_node_ctrl_callback, this);

    //dyn reconfig
    dynamic_reconfigure::Server<rona_frontier::FrontierConfig>::CallbackType f;
    f = boost::bind(&FrontierNode::dynreconfig_callback, this, _1, _2);
    _drServer.setCallback(f);

    _mode = frontier::STOP;
}

FrontierNode::~FrontierNode()
{
}

void FrontierNode::start(double duration)
{
   //create timer
   //_loopTimer = _nh.createTimer(ros::Duration(duration), &FrontierNode::loop_callback, this);
   this->run();
}

void FrontierNode::run()
{
   ros::spin();
}

void FrontierNode::findFrontiers(void)
{
  if(_frontierFinder.isInitialized())
  {
     _frontierFinder.calculateFrontiers();

     ROS_INFO("----------- Frontier Finder --------------- NUM_FRONTIER: %d", (int)_frontierFinder.getFrontiers().size());
     //clear frontiers if no found... was error : always send frontiers from last call if in current none was found...
     _frontiers.clear();
     //_frontiers = _frontierFinder->getFrontiers();

     if(_frontierFinder.getFrontiers().size())
     {
        // look for best frontier
        _frontierController.setWeightedFrontiers(_frontierFinder.getWeightedFrontiers());
        _frontierController.findBestFrontier();
        _frontierController.getWeightedFrontiers();
        _frontiers = _frontierController.getWeightedFrontiers();
        //this->publishFrontiers();
     }

  }
  else
  {
     std::cout << "not initialized" << std::endl;
     ROS_DEBUG_STREAM("FrontierFinder not initialized");
  }
}

void FrontierNode::publishFrontiers(void)
{
  // publish frontiers for rviz and further calculation
  static unsigned int seq = 0;
  geometry_msgs::PoseArray frontiers;
  frontiers.header.stamp    = ros::Time::now();
  frontiers.header.seq      = seq++;
  frontiers.header.frame_id = "map";

  // fill message
  for(unsigned int i = 0; i < _frontiers.size(); ++i)
     frontiers.poses.push_back(_frontiers[i].frontier);

  // publish message
  _pubFrontier.publish(frontiers);
  _pubFronteirGrid.publish(_frontierFinder.getFrontierLayer());
}


void FrontierNode::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!
}


void FrontierNode::sub_node_ctrl_callback(const rona_msgs::NodeCtrl& msg)
{
  ROS_INFO("rona fontier: NodeControll: %s", msg.cmd_str.c_str());
  if(msg.cmd == msg.START)
  {
     _mode = frontier::RUN;
  }
  else if(msg.cmd == msg.SINGLESHOT)
  {
     _mode = frontier::SINGLESHOT;
  }
  else if(msg.cmd == msg.STOP)
  {
     _mode = frontier::STOP;
  }
}



void FrontierNode::sub_map_callback(const nav_msgs::OccupancyGrid& map)
{
  ROS_DEBUG_STREAM("received new map. ");
  _frontierFinder.setMap(map);

  if(_mode == frontier::RUN)
  {
     this->findFrontiers();
     this->publishFrontiers();
  }
  else if(_mode == frontier::SINGLESHOT)
  {
     this->findFrontiers();
     this->publishFrontiers();
     _mode = frontier::STOP;
  }
}




void FrontierNode::dynreconfig_callback(rona_frontier::FrontierConfig& config, uint32_t level)
{
  rona::FrontierControllerConfig cfg;
  cfg.euclideanDistanceFactor = config.dist_factor;
  cfg.orientationFactor       = config.orientation_factor;
  cfg.sizeFactor              = config.size_factor;
  cfg.maxEuclideanDistance    = config.max_dist_threshold;

  ROS_INFO_STREAM("changed configuration: "                              << std::endl <<
                  "distance factor :   "  << cfg.euclideanDistanceFactor << std::endl <<
                  "size factor:        "  << cfg.orientationFactor       << std::endl <<
                  "orientation factor: "  << cfg.sizeFactor              << std::endl <<
                  "max_dist_thresh:    "  << cfg.maxEuclideanDistance << std::endl);


  _frontierController.setConfig(cfg);
}


// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_frontier_node");
    ros::NodeHandle nh("~");

    FrontierNode node;
    node.start();

}


