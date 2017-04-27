/*
 * Frontier.h
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_FRONTIER_H_
#define OHM_FRONTIER_EXPLORATION_SRC_FRONTIER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/LinearMath/Vector3.h>

/**
 * @namespace rona
 */
namespace rona {

/**
 * @class   Frontier
 * @author  Christian Pfitzner
 * @date    2015-01-27
 */
class Frontier : public geometry_msgs::Pose
{
public:
//   Frontier(void)
//   : geometry_msgs::Pose()
//   {
//
//   }
};

/**
 * @struct  WeightedFrontier
 */
struct WeightedFrontier
{
   Frontier       frontier;
   float          size;
   float          weight;

   bool operator<(const WeightedFrontier& f) const {
      return weight > f.weight;
   }
};

/**
 * @struct  FrontierPoint
 */
struct FrontierPoint {
   int idx;
   tf::Vector3 orientation;
};

} /* namespace rona */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_FRONTIER_H_ */
