/*
 * FrontierController.cpp
 *
 *  Created on: 28.01.2015
 *      Author: chris
 */

#include <rona_lib/exploration/FrontierController.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace rona {

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
      listener.lookupTransform( _map_topic, _base_footprint_topic, ros::Time(0), transform);
   }
   catch (tf::TransformException& ex) {
      ROS_ERROR("%s",ex.what());

   }

   const double x = 0.0;//transform.getOrigin().x();
   const double y = 0.0;//transform.getOrigin().y();

   ROS_INFO(" frontier ------------ x: %f, y: %f", x, y);

   // calculate weight for all frontiers
   for(std::vector<WeightedFrontier>::iterator it=_wf.begin() ; it!=_wf.end() ; ++it)
   {
      const float diffX          = it->frontier.position.x - x;
      const float diffY          = it->frontier.position.y - y;

      float dist           = _config.maxEuclideanDistance - sqrt(diffX*diffX + diffY*diffY);

      if(dist < 0.0)
        dist = 0.0;

      const float ori            = 0;

      const float sizeWeight     = it->size   * _config.sizeFactor;
      const float euclDistWeight = dist       * _config.euclideanDistanceFactor;
      const float oriWeight      = ori        * _config.orientationFactor;

//      ROS_INFO("fontier ....... -> dist: %f", dist);
//
      ROS_INFO_STREAM("changed configuration: "                              << std::endl <<
                      "distance factor :   "  << _config.euclideanDistanceFactor << std::endl <<
                      "size factor:        "  << _config.orientationFactor       << std::endl <<
                      "orientation factor: "  << _config.sizeFactor              << std::endl);

      // sum up all weights
      float weight         = sizeWeight + euclDistWeight + oriWeight;

      if(dist < 0.0001) weight = 1000;

      ROS_INFO("frontie : weight: %f", weight);

      // set weight to object
      it->weight = weight;
   }


   // sort frontiers for their weight
   std::sort(_wf.begin(), _wf.end());

   // set best frontier with lowest weight
   _bestFrontier = _wf.back().frontier;
}


};
