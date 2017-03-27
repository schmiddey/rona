/*
 * PathAnalyser_base.cpp
 *
 *  Created on: 06.11.2014
 *      Author: m1ch1
 */


#include "PathAnalyser_base.h"

using namespace Eigen;

namespace analyser{

PathAnalyser_base::PathAnalyser_base() :
      _currentGoal_index(0),
      _dist_to_current_goal(0),
      _path_lenth(0),
      _path_lenth_rest(0),
      _reached_final_goal(false)
{
   _do_end_rotate = true;
}


void PathAnalyser_base::setPath(std::vector<analyser::pose> path)
{
    _path = std::vector<analyser::pose>(path);
    _currentGoal_index = 0;
    _reached_final_goal = false;

    //compute full lenth of path
    double length = 0;
    if(_path.size() > 1)
    {
       for (unsigned int i = 1; i < _path.size(); ++i)
       {
          length += (_path[i-1].position - _path[i].position).norm();
       }
       _path_lenth = _path_lenth_rest = length;
    }
}


analyser::info PathAnalyser_base::getInfo()
{
   analyser::info info;
   info.path_length = _path_lenth;
   info.path_length_remaining = _path_lenth_rest + _dist_to_current_goal;
   info.num_goals = _path.size();
   info.current_goal_id = _currentGoal_index;
   info.reached_goal = _reached_final_goal;
   return info;
}

void PathAnalyser_base::nextGoal()
{
   if(_currentGoal_index < _path.size() - 1)
      _currentGoal_index++;

   //compute rest path
   if(_path.size() > 1)
      _path_lenth_rest -= (_path[_currentGoal_index-1].position - _path[_currentGoal_index].position).norm();
}

} // namespace analyser
