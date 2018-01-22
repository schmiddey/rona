/*
 * MecanumAnalyser.cpp
 *
 *  Created on: 19.01.2018
 *      Author: m1ch1
 */

#include "MecanumAnalyser.h"

namespace analyser{

MecanumAnalyser::MecanumAnalyser(const double target_radius, const double target_radius_final) :
    PathAnalyser_base()
{
  _target_radius       = target_radius;
  _target_radius_final = target_radius_final;


  _curr_target_raius   = _target_radius;
}

MecanumAnalyser::~MecanumAnalyser()
{
  // nothing to do
}

analyser::diff_scale MecanumAnalyser::analyse(const analyser::pose& current_pose)
{
  analyser::diff_scale diff_scale;

  std::cout << "---" << std::endl;

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

  Vector3d p = this->currentGoal().position - pos;
  p.z() = 0;

  //take next goal, maybe skip multiple goals if movement was great enough
  //todo what to do with last goal...
  while(p.norm() < _curr_target_raius)
  {
    this->nextGoal();
    p = this->currentGoal().position - pos;
    p.z() = 0;

    //todo rdy stuff...

    if(this->isLastGoal())
      break;
  }

  int direction_angular = this->getDirection(this->currentGoal().orientation, ori);
  int direction_linear  = this->getDirection(p, ori);
  Vector3d p_ang = this->currentGoal().orientation;

  //get scalfactor angular
  double diff_max = M_PI_2;
  double diff_ang = ::acos(static_cast<double>(ori.dot(p_ang) / (ori.norm() * p_ang.norm())));
  double diff_lin = ::acos(static_cast<double>(ori.dot(p) / (ori.norm() * p.norm())));
  //reached last pose if arrived last goal
//  if(_reachedLastPose && std::abs(tmp_diff) < _ang_reached_range)
//  {
//    diff_scale.angular = 0;
//    this->local_reachedFinalGoal(true);
//  }
//  else
  diff_scale.angular = (diff_ang / diff_max) * direction_angular;  //scale between -1..1

  //linear scale...

  //p is xy diff to next goal...
  Vector2d p_2d(p.x(), p.y());

  std::cout << "diff_lin: " << diff_lin << std::endl;

  std::cout << "p_2d: " << p_2d << std::endl;

  //roate p by ori...
  Rotation2D<double> rot_2d(diff_lin * direction_linear);

  p_2d = rot_2d * p_2d;

  std::cout << "p_2d_rot: " << p_2d << std::endl;

  //scale to +-1
  double max_lin = ( std::max(std::abs(p_2d.x()), std::abs(p_2d.y()) ) );

  diff_scale.linear_x = p_2d.x() / max_lin;
  diff_scale.linear_y = p_2d.y() / max_lin;

  std::cout << "lin_x: " << diff_scale.linear_x << std::endl;
  std::cout << "lin_y: " << diff_scale.linear_y << std::endl;
  std::cout << "ang  : " << diff_scale.angular  << std::endl;


  return diff_scale;
}

} //namespace analyser
