/*
 * MapSubsampler.cpp
 *
 *  Created on: 27.01.2015
 *      Author: chris
 */

#include "MapSubsampler.h"

#include <math.h>

namespace autonohm {

double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

MapSubsampler::MapSubsampler(void)
{

}


MapSubsampler::~MapSubsampler(void)
{
   // nothing to do
}


void MapSubsampler::setInput(nav_msgs::OccupancyGrid map)
{
   _map = map;
}


void MapSubsampler::convert(void)
{
   const float factor = 2.0f;

   // copy header info
   _sub                 = _map;
   _sub.info.resolution = _map.info.resolution * factor;
   _sub.info.width      = _map.info.width      / factor;
   _sub.info.height     = _map.info.height     / factor;

//   _sub.info.origin.position.x   = _map.info.origin.position.x / factor;
//   _sub.info.origin.position.y   = _map.info.origin.position.y / factor;

   _sub.data.resize(_sub.info.width*_sub.info.height);

   // check for unknown space
   for(   unsigned int h=0 ; h<_sub.info.height ;  h++) {
      for(unsigned int w=0 ; w<_map.info.width  ;  w++)
      {
         const unsigned int sub_idx = h*_map.info.height + w;

         const unsigned int col = sub_idx % _sub.info.height;
         const unsigned int row = std::floor(sub_idx / _sub.info.height);

         const unsigned int map_idx = row*factor*_map.info.height + col*factor;

         int values[3] = {-1, 0, 100};

         for(unsigned int i=0 ; i<3 ; i++)
         {
            for(unsigned int f=0 ; f<factor ; f++)
            {
               if(_map.data[map_idx+f]                  == values[i])   _sub.data[sub_idx] = values[i];
               if(_map.data[map_idx+_map.info.height+f] == values[i])   _sub.data[sub_idx] = values[i];
            }
         }

      }
   }
}

} /* namespace autonohm */
