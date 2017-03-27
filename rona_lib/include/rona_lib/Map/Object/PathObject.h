/*
 * PathObject.h
 *
 *  Created on: 11.10.2015
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOBJECTS_PATHOBJECT_H_
#define ALGORITHM_PATHFIND_MAP_MAPOBJECTS_PATHOBJECT_H_

#include "MapObjectbase.h"


namespace rona
{
namespace map
{

class PathObject: public MapObject_base
{
public:
   PathObject(rona::map::Path path, double robot_radius) :
      MapObject_base(),
      _INFL_PATH_FAK(2),
      _ROBOT_POS_R_FAK(4)
   {
      _path = path;
      _robot_radius = robot_radius;
   }
   virtual ~PathObject()
   {}

   void setRobotPos(const rona::map::Point2D robot_pos)
   {
      _robot_pos = robot_pos;
   }

   virtual void draw(std::weak_ptr<rona::map::Grid> grid)
   {
      if(grid.expired())
      {
         std::cerr << "PathObject -> draw() given grid is expired!!!" << std::endl;
         return;
      }
      auto sgrid = grid.lock();
      auto& data = sgrid->getData();
      //draw path with 200 value...
      for(unsigned int i=0; i<_path.size(); ++i)
      {
         data[sgrid->toIdx(_path[i].pos)] = 200;
      }

      //inflate all pixel with 200 value
      rona::map::Operations::inflateCirc(sgrid, 200, 200, _robot_radius * _INFL_PATH_FAK);
      //set robot pos as free value
      rona::map::Operations::overdrawFilledCircle(sgrid, _robot_pos, _robot_radius*_ROBOT_POS_R_FAK, 0, 200, 200);
//      cv::Mat tmpmap = currPlanMap->toCvMat();
//      std::stringstream ss;
//      ss << "/tmp/replan" << _robot_id << ".png";
//      cv::imwrite(ss.str().c_str(), tmpmap);

      //binarize for planner
      rona::map::Operations::binarize(sgrid, 0, 1, 0, 255);

   }

   virtual rona::map::Object whatObject()
   {
      return Object::PATH_OBJECT;
   }

private:
   rona::map::Path _path;

   rona::map::Point2D _robot_pos;

   double _robot_radius;

   const double _INFL_PATH_FAK;
   const double _ROBOT_POS_R_FAK;
};

} /* namespace map */
} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOBJECTS_PATHOBJECT_H_ */
