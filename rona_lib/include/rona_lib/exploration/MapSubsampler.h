/*
 * MapSubsampler.h
 *
 *  Created on: 27.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_MAPSUBSAMPLER_H_
#define OHM_FRONTIER_EXPLORATION_SRC_MAPSUBSAMPLER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


namespace rona {


class MapSubsampler
{
public:
   /**
    * Default destructor
    */
   MapSubsampler(void);
   /**
    * Default destructor
    */
   virtual ~MapSubsampler(void);

   // SETTERS
   void setInput(nav_msgs::OccupancyGrid map);

   // GETTERS
   /**
    * Function to get costmap
    * @return
    */
   nav_msgs::OccupancyGrid getSubsampledMap(void) const    { return _sub; }

   // PROCESSING
   /**
    * Function to start conversion
    */
   void convert(void);

private:
   nav_msgs::OccupancyGrid    _map;          //!< input map for converter
   nav_msgs::OccupancyGrid    _sub;          //!< output costmap
};

} /* namespace rona */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_MAPSUBSAMPLER_H_ */
