/*
 * ConflictAnalyser.h
 *
 *  Created on: 16.09.2015
 *      Author: m1ch1
 */

#ifndef ALGORITHM_MULTI_ROBOT_CONFLICTANALYSER_H_
#define ALGORITHM_MULTI_ROBOT_CONFLICTANALYSER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <limits>

//#define USE_OPENCV

#include "motion_types.h"
#include "../Map/GridMap.h"
#include "../Map/Operations.h"


namespace rona
{
namespace motion
{

typedef struct
{
   bool isValid;
   std::vector<unsigned int> path_ids;
   std::vector<rona::map::Point2D> poses;

}Conflict;



class ConflictAnalyser
{
public:
   /**
    * @brief constructor
    *
    * @param empty_map       allocated map (empty)
    * @param robotradius     robot radius (used for ervery robot)
    * @param time_margin     time margin between conflicts are detectet  t_ == in range( t +- (time_margin/2) )
    */
   ConflictAnalyser(std::weak_ptr<rona::map::Grid> empty_map, double robotradius, double time_margin, double maxLength)
   {
      if(empty_map.expired())
      {
         std::cerr << "no valid map given will exit" << std::endl;
         exit(EXIT_FAILURE);
      }
      _map = empty_map.lock();
      _robotradius = robotradius;
      _timeMargin  = time_margin;
      _maxLength   = maxLength;

      std::cout << "------------------------------" << std::endl;
      std::cout << "-- Collission Analyser -------" << std::endl;
      std::cout << "rr        : " << _robotradius << std::endl;
      std::cout << "timeMargin: " << _timeMargin << std::endl;
      std::cout << "maxLength : " << _maxLength << std::endl;
      std::cout << "------------------------------" << std::endl;
   }
   virtual ~ConflictAnalyser() { }


   //void updatePath(apps::Path path);

   /**
    *
    * @param motrion_own    computed Motion with abs_time, this_robot
    * @param path_own       path, this robot
    * @param motrion_ext    computed Motion with abs_time, partner_robot
    * @param path_ext       path form partner_robot
    * @return
    */
   rona::motion::Conflict analyseConflict(std::vector<double> motrion_own,
                                  const rona::map::Path& path_own,
                                  unsigned int curr_idx_own,
                                  std::vector<double> motrion_ext,
                                  const rona::map::Path& path_ext,
                                  unsigned int curr_idx_ext)
   {
      /*
       * if a collission occur .... (time and pos) then all collisstions between the paths are returned
       *  - also the conflicts wich are no time conflicts
       *
       *  maybe... vorest nur die mit time und pos colli zurück
       *
       */

      Conflict conf;
      conf.isValid = false;

      //if one path is epmty
      if(!path_own.size() || !path_ext.size())
      {
         return conf;
      }

      std::vector<uint8_t>& map_data = _map->getData();

      //clean Map

      for(unsigned int i=0; i<map_data.size(); ++i)
      {
         map_data[i] = 0;
      }


      //draw Path in it    //inflatec path must be ext_path... for better collitionpoint detection
      double length = 0;
      rona::map::Point2D old_point = path_ext[curr_idx_ext].pos;
      for(unsigned int i=curr_idx_ext; i<path_ext.size(); ++i)
      {
         if(i > curr_idx_ext)
         {
            length += rona::map::Operations::computeDistance(old_point, path_ext[i].pos);
         }
         map_data[_map->toIdx(path_ext[i].pos)] = 255;

         if(length > _maxLength)
            break;

         old_point = path_ext[i].pos;
      }


      //inflate path with 2* robot_radius
      rona::map::Operations::inflateCirc(_map, 1, 255, _robotradius * 2);

      std::vector<unsigned int> collissions;    ///< idxs in path_own

      //prove if path collidate with expt path + inflation
      length = 0;
      old_point = path_own[curr_idx_own].pos;
      for(unsigned int i=curr_idx_own; i<path_own.size(); ++i)
      {
         if(i > curr_idx_own)
         {
            length += rona::map::Operations::computeDistance(old_point, path_own[i].pos);
         }
         if(map_data[_map->toIdx(path_own[i].pos)] > 0)
         {//path is in save area vom origin path
            collissions.push_back(i);
            map_data[_map->toIdx(path_own[i].pos)] = 1;
         }
         if(length > _maxLength)
            break;
         old_point = path_own[i].pos;
      }

      //cv::Mat cvmap = _map->toCvMat();
      //cv::imwrite("/tmp/multi_robot_path.png", cvmap);


      //prove Time collition

      for(unsigned int i=0; i<collissions.size(); ++i)
      {
         //find corresponding path elements origin path (corresponding means nearest)
         unsigned int tmp_id = this->findNearestPathElement(path_ext, path_own[collissions[i]].pos);

         //prove time is in margin
         double t_min = motrion_ext[tmp_id] - (_timeMargin );
         double t_max = motrion_ext[tmp_id] + (_timeMargin );

         if(motrion_own[collissions[i]] > t_min && motrion_own[collissions[i]] < t_max)
         {//is in time margin -> conflict
            conf.isValid = true;
            conf.path_ids.push_back(collissions[i]);
            conf.poses.push_back(path_own[collissions[i]].pos);
         }
      }
      return conf;
   }

   unsigned int findNearestPathElement(rona::map::Path to_find_id, rona::map::Point2D p)
   {
      unsigned int id = 0;

      unsigned int minDist = std::numeric_limits<unsigned int>::max();

      /**
       * @note just for testing brute force shuld be not used in future
       * (maybe increasing circle or just prove points which ware in a definded circle)
       */
      for(unsigned int i=0; i<to_find_id.size(); ++i)
      {
         double tmp_dist = rona::map::Operations::computeDistance(p, to_find_id[i].pos);
         if(tmp_dist < minDist)
         {
            id = i;
            minDist = tmp_dist;
         }
      }

      return id;
   }


private:
   std::shared_ptr<rona::map::Grid> _map;

   double _robotradius;
   double _timeMargin;
   double _maxLength;
};

}
} /* namespace apps */


#endif /* ALGORITHM_MULTI_ROBOT_CONFLICTANALYSER_H_ */
