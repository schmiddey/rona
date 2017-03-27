/*
 * Map.h
 *
 *  Created on: 09.12.2015
 *      Author: m1ch1
 */

#ifndef MAP_H_
#define MAP_H_


#include <iostream>
#include <vector>
#include <map>

#include <rona_lib/Map/map_types.h>

namespace rona
{
namespace map
{





class Map
{
public:
   /**
    * @brief
    *
    * @param id
    * @return
    */
   virtual std::vector<Node> getChildNodes(unsigned int id) = 0;

   /**
    * @brief
    *
    * @param file
    */
   virtual void serialize(std::string file) = 0;

   /**
    * @brief
    *
    * @param file
    */
   virtual void load(std::string file) = 0;

   virtual unsigned int getSize() = 0;
};

} /* namespace map */
} /* namespace rona */

#endif /* MAP_H_ */
