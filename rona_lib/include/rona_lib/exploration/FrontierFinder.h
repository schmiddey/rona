/*
 * Finder.h
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_Finder_H_
#define OHM_FRONTIER_EXPLORATION_SRC_Finder_H_

// ros includes
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GridCells.h"

#include "Frontier.h"

// std includes
#include <ostream>
/**
 * @namespace rona
 */
namespace rona {

namespace frontier {

/**
 * @struct  FinderConfig
 * @author  Christian Pfitzner
 * @date    2015-01-26
 *
 * @brief   Parameters for configuration of
 */
struct FinderConfig
{
   double robot_radius;                    //!< size of robot as minimum frontier size

   double min_dist_between_frontiers;      //!< minimum distance between two frontiers

   double max_search_radius;               //!< search radius around robot for frontier search can be used to save computation cost

};

//friend std::ostream& operator<<(std::ostream &output, const FinderConfig &c)
//{
//       output<< "p_robot_radius"             << c.robot_radius               << std::endl;
////                << "p_min_dist_frontiers"       << c.min_dist_between_frontiers << std::endl;
////                << "p_max_search_radius"        << c.max_search_radius          << std::endl;
//return output;
//}


/**
 * @enum CELL_STATE
 */
enum CELL_STATE {
   UNKNOWN  = -1,       //!< UNKNOWN
   FREE     =  0,       //!< FREE
   OCCUPIED =  100     //!< OCCUPIED
};





/**
 * @class   Finder
 * @author  Christian Pfitzner
 * @date    2014-01-26
 *
 * @brief   Frontier based finding on publication from Yamauchi 1998
 */
class Finder
{
public:
   /**
    * Default constructor
    */
   Finder(void);
   /**
    * Constructor with config initialization
    * @param config
    */
   Finder(FinderConfig config);
   /**
    * Default destructor
    */
   virtual ~Finder(void);


   // SETTERS
   /**
    * Function to set map for frontier estimation
    * @param map
    */
   void setMap(const nav_msgs::OccupancyGrid& map);
   /**
    * Function to set configuration
    * @param config
    */
   void setConfig(FinderConfig config);


   // GETTERS
   /**
    * Function to get frontiers
    * @return
    */
   std::vector<Frontier> getFrontiers(void)                 { return _frontiers; }

   /**
    * Function to return weighted frontiers
    * @return
    */
   std::vector<WeightedFrontier> getWeightedFrontiers(void) { return _frontiers_weighted; }

   /**
    * Function to get frontier layer for debugging
    * @return
    */
   nav_msgs::GridCells getFrontierLayer(void)               { return _frontier_layer; }


   /**
    * Function to start calculation
    */
   void calculateFrontiers(void);
   /**
    * Function to check if frontier search is initialized
    * @return
    */
   bool isInitialized(void)   { return _initialized; }

private:
   /**
    * Function to convert index to coordinate
    * @param idx           index of cell
    * @param width         width of whole map
    * @param originX       origin of map (x coordinate)
    * @param originY       origin of map (y coordinate)
    * @return
    */
   geometry_msgs::Point getPointFromIndex(unsigned int idx, unsigned int width,
                                          float originX = 0.0f, float originY = 0.0f,
                                          float resolution = 1.0);

   /**
    * Function to optimize frontier, which could be in unknown terrain.
    * This is necessary, because the path planner.
    * @param frontier
    */
   void optimizeFrontierIfInUnknown(Frontier& frontier);




   bool                             _initialized;

   nav_msgs::OccupancyGrid          _map;                //!< map for exploration
   FinderConfig                     _config;             //!< config for exploration
   std::vector<Frontier>            _frontiers;          //!< container for found frontiers
   std::vector<WeightedFrontier>    _frontiers_weighted; //!< weighted frontiers

   nav_msgs::GridCells              _frontier_layer;     //!< layer for debugging
};

} /* namespace frontier */

} /* namespace rona */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_Finder_H_ */
