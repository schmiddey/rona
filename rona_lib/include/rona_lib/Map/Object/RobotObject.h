/*
 * RobotObject.h
 *
 *  Created on: 11.10.2015
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOBJECTS_ROBOTOBJECT_H_
#define ALGORITHM_PATHFIND_MAP_MAPOBJECTS_ROBOTOBJECT_H_

#include "MapObjectbase.h"

namespace rona
{
namespace map
{

class RobotObject: public MapObject_base
{
public:
   RobotObject(rona::map::Point2D pos, double robot_radius) :
      MapObject_base(),
      _POS(pos),
      _ROBOT_RADIUS(robot_radius)
   {}
   virtual ~RobotObject()
   {}
   virtual void draw(std::weak_ptr<rona::map::Grid> grid)
   {
      if(grid.expired())
      {
         std::cerr << "RobotObject -> draw() given grid is expired!!!" << std::endl;
         return;
      }
      rona::map::Operations::drawFilledCircle(grid, _POS, _ROBOT_RADIUS, 255);
   }

   virtual rona::map::Object whatObject()
   {
      return Object::ROBOT_OBJECT;
   }

private:
   const rona::map::Point2D _POS;
   const double _ROBOT_RADIUS;
};

} /* namespace map */
} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOBJECTS_ROBOTOBJECT_H_ */
