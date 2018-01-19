/*
 * MecanumAnalyser.cpp
 *
 *  Created on: 19.01.2018
 *      Author: m1ch1
 */

#include "MecanumAnalyser.h"

namespace analyser{

MecanumAnalyser::MecanumAnalyser()
{
  _target_radius       = 0.2;
  _target_radius_final = 0.1; //todo


  _curr_target_raius   = _target_radius;
}

MecanumAnalyser::~MecanumAnalyser()
{
  // nothing to do
}

analyser::diff_scale MecanumAnalyser::analyse(const analyser::pose& current_pose)
{
  analyser::diff_scale diff_scale;

  if(_path.empty())
  {
    this->setReachedFinalGoal(false);
    return diff_scale;
  }

  if(this->isReachedFinalGoal())
  {
    return diff_scale;
  }


  Vector3d ori = current_pose.orientation;
  Vector3d pos = current_pose.position;

  Vector3d p = this->currentGoal() - pos;
  p.z() = 0;

  //take next goal, maybe skip multiple goals if movement was great enough
  //todo what to do with last goal...
  while(p.norm() < _curr_target_raius)
  {
    this->nextGoal();
    p = this->currentGoal() - pos;
    p.z() = 0;

    //todo rdy stuff...

    if(this->isLastGoal())
      break;
  }

  //todo do_endtorate yes or no...


  int direction_angular = this->getDirection(this->currentGoal().orientation, ori);

  //get scalfactor angular
  double diff_max = M_PI_2;
  double tmp_diff = ::acos(static_cast<double>(ori.dot(p) / (ori.norm() * p.norm())));
  //reached last pose if arrived last goal
//  if(_reachedLastPose && std::abs(tmp_diff) < _ang_reached_range)
//  {
//    diff_scale.angular = 0;
//    this->local_reachedFinalGoal(true);
//  }
//  else
  diff_scale.angular = (tmp_diff / diff_max) * direction_angular;  //scale between -1..1


  //linear scale...











  return diff_scale;
}

} //namespace analyser
