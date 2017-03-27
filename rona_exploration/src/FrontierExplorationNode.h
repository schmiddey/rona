/*
 * FrontierExplorationNode.h
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_
#define OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
//#include <costmap_2d/costmap_2d_ros.h>

#include <rona_lib/exploration/Frontier.h>
#include <rona_lib/exploration/FrontierFinder.h>
#include <rona_lib/exploration/Visualization.h>
#include <rona_lib/exploration/FrontierController.h>

#include <rona_msgs/GetFrontierTarget.h>

//trigger service
#include <rona_msgs/NodeCtrl.h>


/**
 * @namespace autonohm
 */
namespace autonohm
{

namespace frontier{
enum enumMode{
   RUN = 0,
   SINGLESHOT,
   STOP
};
}

/**
 *
 */
class FrontierExplorationNode
{
public:
   /**
    * Function to get instance of singleton
    * @return
    */
   static FrontierExplorationNode* getInstance(void);

   /**
    * Default destructor
    */
   virtual ~FrontierExplorationNode(void);

   /**
    * Function to spin ros node
    */
   void run(void);

   bool isInitialized(void);

   void setDynamicConfig(FrontierControllerConfig c);

   void setLoopRate(const double& looprate) { _rate = looprate; }

private:
   /**
    * Private constructor for singleton pattern
    */
   FrontierExplorationNode(void);
   /**
    * Private copy constructor for singleton pattern
    * @param
    */
   FrontierExplorationNode(FrontierExplorationNode &) { }

   /**
    * Function to search for frontiers
    */
   void findFrontiers(void);
   /**
    * Function to publish frontiers
    */
   void publishFrontiers(void);

   void publishMarkers(void);

   // CALLBACK FUNCTIONS
   /**
    * Callback function for map
    * @param map
    */
   void mapCallback(const nav_msgs::OccupancyGrid& map);

   /**
    * Service callback to receive next frontier for navigation
    * @param req
    * @param res
    * @return
    */
   bool getFrontierServiceCB(rona_msgs::GetFrontierTarget::Request&  req,
         rona_msgs::GetFrontierTarget::Response& res);
   /**
    * Service callback to return all frontiers
    * @param req
    * @param res
    * @return
    */
   bool getAllFrontierServiceCB(rona_msgs::GetFrontierTarget::Request&  req,
         rona_msgs::GetFrontierTarget::Response& res);

   /**
    * Service to trigger the transmission of the frontiers
    * @param req
    * @param res
    * @return
    */
   bool callback_srv_transmittTargets(rona_msgs::NodeCtrl::Request&  req,
         rona_msgs::NodeCtrl::Response& res);

   // MEMBERS
   static FrontierExplorationNode* _instance;

   ros::NodeHandle                  _nh;

   ros::Subscriber                  _map_sub;

   ros::Publisher                   _sub_map_pub;
   ros::Publisher                   _frontier_pub;
   ros::Publisher                   _frontier_grid_pub;
   ros::Publisher                   _maker_pub;

   ros::ServiceServer               _best_target_service;
   ros::ServiceServer               _all_targets_service;
   ros::ServiceServer               _transmitt_targets_service;

   std::vector<WeightedFrontier>    _frontiers;

   frontier::Finder*                _frontierFinder;
   frontier::Visualization          _viz;
   FrontierController*              _frontierController;

   frontier::enumMode               _mode;

   bool                             _is_initialized;        //!< flag to check if node is initialized
   double                           _rate;                  //!< looprate to spin ros node
};

} /* namespace autonohm */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_ */
