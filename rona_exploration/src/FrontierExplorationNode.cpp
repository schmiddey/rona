/*
 * FrontierExplorationNode.cpp
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#include "FrontierExplorationNode.h"

#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseArray.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <rona_lib/ExplorationConfig.h>



namespace autonohm {

// initialization of singleton pattern
FrontierExplorationNode* FrontierExplorationNode::_instance = 0;


FrontierExplorationNode* FrontierExplorationNode::getInstance(void)
{
   if(!_instance) _instance = new FrontierExplorationNode;
   return _instance;
}

void FrontierExplorationNode::run(void)
{
   ros::Rate looprate(_rate);
   while(ros::ok()) {
      ros::spinOnce();
      looprate.sleep();
   }
}

bool FrontierExplorationNode::isInitialized(void)
{
   return _is_initialized;
}


void FrontierExplorationNode::setDynamicConfig(FrontierControllerConfig c)
{
   _frontierController->setConfig(c);
}


FrontierExplorationNode::FrontierExplorationNode(void) :
      _frontierFinder(NULL)
,     _is_initialized(false)
,     _rate(5.0)
{
   ros::NodeHandle private_nh("~");

   frontier::FinderConfig config;
   private_nh.param<double>("robot_radius",               config.robot_radius,               0.6);
   private_nh.param<double>("min_dist_between_frontiers", config.min_dist_between_frontiers, 1.0);
   private_nh.param<double>("max_search_radius",          config.max_search_radius,          10.0);

   _frontierFinder     = new frontier::Finder(config);
   _frontierController = new FrontierController;

   std::string map_topic;
   std::string frontier_topic;
   std::string base_topic;
   private_nh.param("map_topic",             map_topic,      std::string("/map"));
   private_nh.param("base_footprint_topic",  base_topic,     std::string("laser"));
   private_nh.param("frontier_topic",        frontier_topic, std::string("frontiers"));

   _frontierController->setTFFrameIds(map_topic, base_topic);

   // Publishers
   _frontier_pub      = _nh.advertise<geometry_msgs::PoseArray>(frontier_topic,  1);

   // Subscriber
   _map_sub           = _nh.subscribe(map_topic, 1, &FrontierExplorationNode::mapCallback, this);
   _sub_map_pub       = _nh.advertise<nav_msgs::OccupancyGrid>("sub_map",  1);
   _frontier_grid_pub = _nh.advertise<nav_msgs::GridCells>("frontier_grid", 1);

   // Service Server
   _best_target_service       = private_nh.advertiseService("get_target", &FrontierExplorationNode::getFrontierServiceCB, this);
   _transmitt_targets_service = _nh.advertise<rona_msgs::NodeCtrl>("frontier/node_control", 1);

   _viz.setNodeHandle(_nh);

   _is_initialized = true;

   //set runMode
   _mode = frontier::STOP;

   // start node
   this->run();
}


FrontierExplorationNode::~FrontierExplorationNode(void)
{
   if(_instance) {
      delete _instance;
      _instance = 0;
   }

   delete _frontierFinder;
   delete _frontierController;
}


void FrontierExplorationNode::findFrontiers(void)
{
   if(_frontierFinder->isInitialized())
   {
      _frontierFinder->calculateFrontiers();

      ROS_INFO("----------- Frontier Finder --------------- NUM_FRONTIER: %d", _frontierFinder->getFrontiers().size());

      //_frontiers = _frontierFinder->getFrontiers();

      if(_frontierFinder->getFrontiers().size())
      {
         // look for best frontier
         _frontierController->setWeightedFrontiers(_frontierFinder->getWeightedFrontiers());
         _frontierController->findBestFrontier();
         _frontierController->getWeightedFrontiers();
         _frontiers = _frontierController->getWeightedFrontiers();
         //this->publishFrontiers();
      }
   }
   else
   {
      std::cout << "not initialized" << std::endl;
      ROS_DEBUG_STREAM("FrontierFinder not initialized");
   }
}


void FrontierExplorationNode::publishFrontiers(void)
{
   // publish frontiers for rviz and further calculation
   static unsigned int seq = 0;
   geometry_msgs::PoseArray frontierMarkers;
   frontierMarkers.header.stamp    = ros::Time::now();
   frontierMarkers.header.seq      = seq++;
   frontierMarkers.header.frame_id = "map";

   // fill message
   for(unsigned int i = 0; i < _frontiers.size(); ++i)
      frontierMarkers.poses.push_back(_frontiers[i].frontier);

   // publish message
   _frontier_pub.publish(frontierMarkers);
   _frontier_grid_pub.publish(_frontierFinder->getFrontierLayer());

   // visualization
   _viz.setFrontiers(        _frontierFinder->getFrontiers());
   _viz.setBestFrontier(     _frontierController->getBestFrontier());
   _viz.setWeightedFrontiers(_frontierController->getWeightedFrontiers());
   _viz.publish();
}


void FrontierExplorationNode::mapCallback(const nav_msgs::OccupancyGrid& map)
{
   ROS_DEBUG_STREAM("received new map. ");
   _frontierFinder->setMap(map);

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

bool FrontierExplorationNode::getFrontierServiceCB(rona_msgs::GetFrontierTarget::Request& req,
      rona_msgs::GetFrontierTarget::Response& res)
{
   ROS_DEBUG_STREAM("service call: returning new frontier to: " << _frontierController->getBestFrontier());
   res.target = _frontierController->getBestFrontier();
   return true;
}

void FrontierExplorationNode::publishMarkers(void)
{
}

bool FrontierExplorationNode::callback_srv_transmittTargets(rona_msgs::NodeCtrl& req){

   if(req.cmd == req.START)
   {
      _mode = frontier::RUN;
   }
   else if(req.cmd == req.SINGLESHOT)
   {
      _mode = frontier::SINGLESHOT;
   }
   else if(req.cmd == req.STOP)
   {
      _mode = frontier::STOP;
   }
//   else
//   {
//      res.accepted = false;
//   }
//   ROS_INFO("ohm_frontier ->  callback NodeControll serivce: accept: %s", (res.accepted) ? "true" : "false");

   return true;
}

} /* namespace autonohm */


//void callback(ExplorationConfig &config, uint32_t level)
//{
//   std::cout << __PRETTY_FUNCTION__ << std::endl;
//
////   if(!autonohm::FrontierExplorationNode::getInstance()->isInitialized())
////      return;
//
//   autonohm::FrontierControllerConfig cfg;
//   cfg.euclideanDistanceFactor = 0; //config.dist_factor;
//   cfg.orientationFactor       = config.orientation_factor;
//   cfg.sizeFactor              = 8.0; //config.size_factor;
//
//
//   ROS_INFO_STREAM("changed configuration: "                              << std::endl <<
//                   "distance factor :   "  << cfg.euclideanDistanceFactor << std::endl <<
//                   "size factor:        "  << cfg.orientationFactor       << std::endl <<
//                   "orientation factor: "  << cfg.sizeFactor              << std::endl);
//
//
//   autonohm::FrontierExplorationNode::getInstance()->setDynamicConfig(cfg);
//}

/*
 * Main program
 */
int main(int argc,char **argv)
{
   ros::init(argc, argv, "frontier_exploration_node");

//   // configuration for dynamic reconfigure
   dynamic_reconfigure::Server<ohm_frontier_exploration::ExplorationConfig> server;
   dynamic_reconfigure::Server<ohm_frontier_exploration::ExplorationConfig>::CallbackType f;
//
//   f = boost::bind(&callback, _1, _2);
//   server.setCallback(f);

   autonohm::FrontierExplorationNode::getInstance()->run();
}
