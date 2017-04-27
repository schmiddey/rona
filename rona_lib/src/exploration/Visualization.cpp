/*
 * Visualization.cpp
 *
 *  Created on: 28.01.2015
 *      Author: chris
 */

#include <rona_lib/exploration/Visualization.h>

namespace rona {

namespace frontier {

Visualization::Visualization(void)
{
   ros::NodeHandle privat_nh("~");
   _idx = 0;

   _frontier_maker_pub = _nh.advertise<visualization_msgs::MarkerArray>("frontier_marker_array", 100);
}

Visualization::~Visualization(void)
{

}

void Visualization::publish(void)
{
   _markers.markers.clear();
   _idx = 0;

   if(_frontiers.size() > 0) {
      this->publishFrontierMarker();
   }
   if(_wFrontiers.size() > 0) {
      this->publishWeightedFrontierMarker();
   }

   if(_bestFrontier.position.x != 0 &&
      _bestFrontier.position.y != 0) {
      this->publishBestFrontier();
   }

   if(_markers.markers.size() > 0) {
      _frontier_maker_pub.publish(_markers);
   }

}


// PRIVATE
void Visualization::publishFrontierMarker(void)
{
   unsigned int idx = 0;

   for(std::vector<Frontier>::const_iterator it=_frontiers.begin() ; it != _frontiers.end() ; ++it)
   {
      visualization_msgs::MarkerArray frontier_markers;
      visualization_msgs::Marker m;
      m.header.frame_id = "/map";
      m.header.stamp    = ros::Time::now();
      m.ns              = "frontiers";
      m.id              = idx++;

      m.type            = visualization_msgs::Marker::CYLINDER;
      m.action          = visualization_msgs::Marker::ADD;
      m.lifetime        = ros::Duration(5.0f);

      // set color
      m.color.r         = 0.0f;
      m.color.g         = 1.0f;
      m.color.b         = 0.0f;
      m.color.a         = 0.5f;

      // set size
      m.scale.x         = 0.1f;
      m.scale.y         = 0.1f;
      m.scale.z         = 0.5f;

      m.pose.position   =  it->position;
      m.pose.position.z += m.scale.z / 2.0f;

      _markers.markers.push_back(m);
   }

}


void Visualization::publishWeightedFrontierMarker(void)
{
   unsigned int idx = 0;

   for(std::vector<WeightedFrontier>::const_iterator it=_wFrontiers.begin() ; it != _wFrontiers.end() ; ++it)
   {
      visualization_msgs::Marker m;
      m.header.frame_id  = "/map";
      m.header.stamp     = ros::Time::now();
      m.ns               = "weight_frontiers";
      m.id               = idx++;

      m.type             = visualization_msgs::Marker::CYLINDER;
      m.action           = visualization_msgs::Marker::ADD;
      m.lifetime         = ros::Duration(5.0f);

      // set color
      m.color.r          = 1.0f;
      m.color.g          = 0.0f;
      m.color.b          = 0.0f;
      m.color.a          = 0.5f;

      // set size
      m.scale.x          = 0.2f;
      m.scale.y          = 0.2f;
      m.scale.z          = it->weight;

      m.pose.position    = it->frontier.position;
      m.pose.position.z += m.scale.z / 2.0f;

      _markers.markers.push_back(m);
   }
}


void Visualization::publishBestFrontier(void)
{
   unsigned int idx = 0;
   visualization_msgs::Marker m;
   m.header.frame_id = "/map";
   m.header.stamp    = ros::Time::now();
   m.ns              = "best_frontier";
   m.id              = idx++;

   m.type            = visualization_msgs::Marker::ARROW;
   m.action          = visualization_msgs::Marker::MODIFY;
   m.lifetime        = ros::Duration(5.0f);

   // set color
   m.color.r         = 0.0f;
   m.color.g         = 0.0f;
   m.color.b         = 1.0f;
   m.color.a         = 1.0f;

   // set size
   m.scale.x         = 1.0f;
   m.scale.y         = 0.2f;
   m.scale.z         = 0.2f;

   m.pose.position    = _bestFrontier.position;
   m.pose.orientation = _bestFrontier.orientation;

   _markers.markers.push_back(m);
}

} /* namespace frontier */
} /* namespace rona */
