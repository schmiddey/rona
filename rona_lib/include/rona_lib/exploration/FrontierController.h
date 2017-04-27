/*
 * FrontierController.h
 *
 *  Created on: 28.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_FRONTIERCONTROLLER_H_
#define OHM_FRONTIER_EXPLORATION_SRC_FRONTIERCONTROLLER_H_

// ros includes
#include <ros/ros.h>

//
#include "Frontier.h"

/**
 * @namespace  rona
 */
namespace rona {

/**
 * @struct  FrontierControllerConfig
 * @author  Christian Pfitzner
 * @date    2015-01-27
 */
struct FrontierControllerConfig
{
   float sizeFactor;                   //!< factor to be multiplied with frontier size
   float euclideanDistanceFactor;      //!< factor to be multiplied with euclidean distance to robot's pose
   float orientationFactor;            //!< factor to be multiplied with orientation to robot's pose

   float maxEuclideanDistance;         //!< maximum value for distance to travel to next
};




/**
 * @class   FrontierController
 * @author  Christian Pfitzner
 * @date    2015-01-27
 *
 * @brief   Class to choose best suitable frontier depending on
 *          size of the frontier
 */
class FrontierController
{
public:
   /**
    * Default constructor
    */
   FrontierController(void);
   /**
    * Default destructor
    */
   virtual ~FrontierController(void);


   // SETTERS
   /**
    * Function to set all fund frontiers
    * @param wf
    */
   void setWeightedFrontiers(const std::vector<WeightedFrontier>& wf);
   /**
    * Function to set configuration to frontier controller
    * @param config
    */
   void setConfig(FrontierControllerConfig config) { _config = config; }

   void setTFFrameIds(std::string map_frame_id, std::string base_footprint_frame_id) {
      _map_topic            = map_frame_id;
      _base_footprint_topic = base_footprint_frame_id;
   }


   // GETTERS
   /**
    * Function to return best frontier
    * @return
    */
   Frontier getBestFrontier(void) const                   { return _bestFrontier; }
   std::vector<WeightedFrontier> getWeightedFrontiers(void) const { return _wf; }


   // PROCESSING
   /**
    * Function to start processing
    */
   void findBestFrontier(void);


private:
   Frontier                      _bestFrontier;       //!< best solution for all frontiers depending on weight
   std::vector<WeightedFrontier> _wf;                 //!< all weighted frontiers

   FrontierControllerConfig      _config;

   std::string                   _map_topic;
   std::string                   _base_footprint_topic;

};

};

#endif /* OHM_FRONTIER_EXPLORATION_SRC_FRONTIERCONTROLLER_H_ */
