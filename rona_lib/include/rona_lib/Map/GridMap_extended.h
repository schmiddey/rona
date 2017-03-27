/*
 * GridMap.h
 *
 *  Created on: 09.12.2015
 *      Author: m1ch1
 */

#ifndef GRIDMAP_EXTENDED_H_
#define GRIDMAP_EXTENDED_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <cstdint>
#include <cmath>


#include <rona_lib/Map/Map.h>
#include <rona_lib/Map/Grid.h>
#include <rona_lib/Map/GridMap.h>

#ifdef USE_ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#endif

namespace rona
{
namespace map
{

/**
 * @class GridMap
 *
 * @brief Class for PathPlanner containing a grid. Plus functions for Graph representation
 */
class GridMap_extended: public GridMap
{
public:
   using GridMap::GridMap; //uses constructors from baseclass

   virtual std::vector<Node> getChildNodes(unsigned int id)
   {
      if(!_rdy)
      {
         this->init_extended();
         _rdy = true;
      }
      std::vector<Node> nodes;
      Pixel p = _grid->toPixel(id);

      std::vector<Pixel> tmp_vec(8+16);

      tmp_vec[0]  = (Pixel(p.x-1, p.y-1));
      tmp_vec[1]  = (Pixel(p.x,   p.y-1));
      tmp_vec[2]  = (Pixel(p.x+1, p.y-1));
      tmp_vec[3]  = (Pixel(p.x-1, p.y  ));
      tmp_vec[4]  = (Pixel(p.x+1, p.y  ));
      tmp_vec[5]  = (Pixel(p.x-1, p.y+1));
      tmp_vec[6]  = (Pixel(p.x,   p.y+1));
      tmp_vec[7]  = (Pixel(p.x+1, p.y+1));
      tmp_vec[8]  = (Pixel(p.x-2, p.y-2));
      tmp_vec[9]  = (Pixel(p.x-1, p.y-2));
      tmp_vec[10] = (Pixel(p.x,   p.y-2));
      tmp_vec[11] = (Pixel(p.x-1, p.y-2));
      tmp_vec[12] = (Pixel(p.x-2, p.y-2));
      tmp_vec[13] = (Pixel(p.x-2, p.y-1));
      tmp_vec[14] = (Pixel(p.x+2, p.y-1));
      tmp_vec[15] = (Pixel(p.x-2, p.y  ));
      tmp_vec[16] = (Pixel(p.x+2, p.y  ));
      tmp_vec[17] = (Pixel(p.x-2, p.y+1));
      tmp_vec[18] = (Pixel(p.x+2, p.y+1));
      tmp_vec[19] = (Pixel(p.x+2, p.y+2));
      tmp_vec[20] = (Pixel(p.x+1, p.y+2));
      tmp_vec[21] = (Pixel(p.x,   p.y+2));
      tmp_vec[22] = (Pixel(p.x-1, p.y+2));
      tmp_vec[23] = (Pixel(p.x-2, p.y+2));



      for(unsigned int i=0; i<tmp_vec.size(); ++i)
      {
         //prove border cases
         if((int)tmp_vec[i].x >= 0 && (int)tmp_vec[i].y >= 0 && tmp_vec[i].x <= (_grid->getWidth() - 1) && tmp_vec[i].y <= (_grid->getHeight() - 1))
         {
            unsigned int succ_id = _grid->toIdx(tmp_vec[i]);
            //prove occupied
            if(!this->isOccupied(succ_id))
            {//Free
               Node node;
               node.id = succ_id;
               node.pos = _grid->toPoint2D(tmp_vec[i]);

               // only add one edge from gridMap for distance and cost (edge from parent id)
               Edge edge;
               edge.id = id; //parent id
               edge.distance = _parentDist_extended[i];
               edge.cost = this->computeCost(succ_id);

               //insert edge
               node.edges_map.insert(std::make_pair(edge.id, edge));
               nodes.push_back(node);
            }
         }
      }

      return nodes;
   }

private:
   std::vector<double> _parentDist_extended;
   bool _rdy = false;
   void init_extended()
   {
      _occupiedMinValue = 10;
      _occupiedMaxValue = 255;

      _parentDist_extended.resize(8+16);

      _parentDist_extended[0]  = PIXEL_LONG_COST  * _grid->getCellSize();
      _parentDist_extended[1]  = PIXEL_SHORT_COST * _grid->getCellSize();
      _parentDist_extended[2]  = PIXEL_LONG_COST  * _grid->getCellSize();
      _parentDist_extended[3]  = PIXEL_SHORT_COST * _grid->getCellSize();
      _parentDist_extended[4]  = PIXEL_SHORT_COST * _grid->getCellSize();
      _parentDist_extended[5]  = PIXEL_LONG_COST  * _grid->getCellSize();
      _parentDist_extended[6]  = PIXEL_SHORT_COST * _grid->getCellSize();
      _parentDist_extended[7]  = PIXEL_LONG_COST  * _grid->getCellSize();

      _parentDist_extended[8]  = PIXEL_LONG_LONG_COST    * _grid->getCellSize();
      _parentDist_extended[9]  = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[10] = PIXEL_SHORT_SHORT_COST  * _grid->getCellSize();
      _parentDist_extended[11] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[12] = PIXEL_LONG_LONG_COST    * _grid->getCellSize();
      _parentDist_extended[13] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[14] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[15] = PIXEL_SHORT_SHORT_COST  * _grid->getCellSize();
      _parentDist_extended[16] = PIXEL_SHORT_SHORT_COST  * _grid->getCellSize();
      _parentDist_extended[17] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[18] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();

      _parentDist_extended[19] = PIXEL_LONG_LONG_COST    * _grid->getCellSize();
      _parentDist_extended[20] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[21] = PIXEL_SHORT_SHORT_COST  * _grid->getCellSize();
      _parentDist_extended[22] = PIXEL_LONG_SHORT_COST   * _grid->getCellSize();
      _parentDist_extended[23] = PIXEL_LONG_LONG_COST    * _grid->getCellSize();
   }

   const double PIXEL_SHORT_COST  = 1;
   const double PIXEL_LONG_COST   = ::sqrt(PIXEL_SHORT_COST*PIXEL_SHORT_COST + PIXEL_SHORT_COST*PIXEL_SHORT_COST);
   const double PIXEL_LONG_LONG_COST = PIXEL_LONG_COST * 2;
   const double PIXEL_LONG_SHORT_COST = PIXEL_LONG_COST + PIXEL_SHORT_COST;
   const double PIXEL_SHORT_SHORT_COST = PIXEL_SHORT_COST * 2;
};


} /* namespace map */
} /* namespace rona */

#endif /* GRIDMAP_H_ */
