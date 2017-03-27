/*
 * Visualization.h
 *
 *  Created on: 28.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_VISUALIZATION_H_
#define OHM_FRONTIER_EXPLORATION_SRC_VISUALIZATION_H_

#include <visualization_msgs/MarkerArray.h>
#include "Frontier.h"

/**
 * @namespace autonohm
 */
namespace autonohm {
/**
 * @namespace frontier
 */
namespace frontier {

/**
 * @class   Visualization
 * @author  Christian Pfitzner
 * @date    2015-01-28
 *
 * @brief   Visualization for frontier based exploration
 */
class Visualization
{
public:
   /**
    * Default constructor
    */
   Visualization(void);
   /**
    * Default destructor
    */
   virtual ~Visualization(void);

   // SETTERS
   void setNodeHandle(ros::NodeHandle nh)                              { _nh = nh; }
   /**
    * Function to set frontiers for visualization
    * @param frontiers
    */
   void setFrontiers(std::vector<Frontier> frontiers)                  { _frontiers    = frontiers; }
   /**
    * Function to set weighted frontiers
    * @param wFrontiers
    */
   void setWeightedFrontiers(std::vector<WeightedFrontier> wFrontiers) { _wFrontiers   = wFrontiers; }
   /**
    * Function to set best frontier for next exploration goal
    * @param best
    */
   void setBestFrontier(Frontier best)                                 { _bestFrontier = best; }

   // PROCESS
   /**
    * Function to publish visualization to rviz as markers
    */
   void publish(void);
private:

   void publishFrontierMarker(void);
   void publishWeightedFrontierMarker(void);
   void publishBestFrontier(void);

   unsigned int                  _idx;

   ros::NodeHandle               _nh;

   ros::Publisher                _frontier_maker_pub;

   std::vector<Frontier>         _frontiers;                //!< all frontiers
   std::vector<WeightedFrontier> _wFrontiers;               //!< weighted frontiers
   Frontier                      _bestFrontier;

   visualization_msgs::MarkerArray _markers;
};

} /* namespace autonohm */
} /* namespace frontier */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_VISUALIZATION_H_ */
