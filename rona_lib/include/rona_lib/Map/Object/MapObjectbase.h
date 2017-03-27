/*
 * MapObjectbase.h
 *
 *  Created on: 11.10.2015
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOBJECTS_MAPOBJECTBASE_H_
#define ALGORITHM_PATHFIND_MAP_MAPOBJECTS_MAPOBJECTBASE_H_


#include <iostream>
#include <memory>

#include "../Grid.h"
#include "../Operations.h"

namespace rona
{

namespace map
{

enum class Object{
   FREE = 0,   //non object
   ROBOT_OBJECT,
   PATH_OBJECT
};

class MapObject_base
{
public:
   MapObject_base()
   {}
   virtual ~MapObject_base()
   {}

   virtual void draw(std::weak_ptr<rona::map::Grid> grid) = 0;

   virtual rona::map::Object whatObject() = 0;

};

} /* namespace map */
} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOBJECTS_MAPOBJECTBASE_H_ */
