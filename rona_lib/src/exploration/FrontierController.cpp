/*
 * FrontierController.cpp
 *
 *  Created on: 28.01.2015
 *      Author: chris
 */

#include "FrontierController.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace autonohm {

FrontierController::FrontierController(void)
{
   // default setting for only size
   _config.euclideanDistanceFactor = 2;
   _config.orientationFactor       = 0;
   _config.sizeFactor              = 0.1;

   _config.maxEuclideanDistance    = 12.0;
}

FrontierController::~FrontierController(void)
{

}


void FrontierController::setWeightedFrontiers(const std::vector<WeightedFrontier>& wf)
{
   _wf = wf;
}

void FrontierController::findBestFrontier(void)
{
   /*
    * get position of robot to map coordinate system
    */
   tf::TransformListener listener;
   tf::StampedTransform transform;

   try {
      listener.waitForTransform(_map_topic,            _base_footprint_topic, ros::Time(0), ros::Duration(1.0) );
      listener.lookupTransform( _base_footprint_topic, _base_footprint_topic, ros::Time(0), transform);
   }
   catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());

   }

   const double x = transform.getOrigin().x();
   const double y = transform.getOrigin().y();

   // calculate weight for all frontiers
   for(std::vector<WeightedFrontier>::iterator it=_wf.begin() ; it!=_wf.end() ; ++it)
   {
      const float diffX          = it->frontier.position.x - x;
      const float diffY          = it->frontier.position.y - y;

      float dist           = sqrt(diffX*diffX + diffY*diffY);

      const float ori            = 0;

      const float sizeWeight     = it->size   * _config.sizeFactor;
      const float euclDistWeight = (1.0/dist) * _config.euclideanDistanceFactor;
      const float oriWeight      = ori        * _config.orientationFactor;

      // sum up all weights
      float weight         = sizeWeight + euclDistWeight + oriWeight;
      if(dist > _config.maxEuclideanDistance) weight = 1000;

      // set weight to object
      it->weight = weight;
   }


   // sort frontiers for their weight
   std::sort(_wf.begin(), _wf.end());

   // set best frontier with lowest weight
   _bestFrontier = _wf.back().frontier;
}


};
