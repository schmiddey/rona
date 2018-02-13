/*
 * MecanumAnalyser.cpp
 *
 *  Created on: 19.01.2018
 *      Author: m1ch1
 */

#include "MecanumAnalyser.h"

namespace analyser{

MecanumAnalyser::MecanumAnalyser(const cfg::AnalyserBase_config& cfg) :
    PathAnalyser_base()
{
  _cfg = cfg;

  _curr_target_raius   = _cfg.target_radius;
  _is_end_approach = false;
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

    if(this->isLastGoal())
    {
      break;
    }
  }

  Vector3d e(1,0,0);

  int direction_angular = this->getDirection(this->currentGoal().orientation, ori);
  int direction_linear  = this->getDirection(e, ori);
  Vector3d p_ang = this->currentGoal().orientation;

  //get scalfactor angular
  double diff_max = M_PI;
  double diff_ang = ::acos(static_cast<double>(ori.dot(p_ang) / (ori.norm() * p_ang.norm())));
  double diff_lin = ::acos(static_cast<double>(ori.dot(e) / (ori.norm() * e.norm())));


  //linear scale...

  //p is xy diff to next goal...
  Vector2d p_2d(p.x(), p.y());

  //debug output
  {
    static int cnt = 0;
    if((++cnt % 50) == 0)
    {
      std::cout << "AngOffset: " << ((diff_ang * 180) / M_PI) * 1000 << " milli°" << std::endl;
      std::cout << "Offset: (" << p.x() << ", " << p.y() << ")" << std::endl;
    }
  }


//  std::cout << "diff ang: " << diff_ang << ", cfg_ang_reached: " << _cfg.ang_reached_range << std::endl;

  if(p_2d.norm() < _cfg.target_radius_final && diff_ang < _cfg.ang_reached_range)
  {
    this->setReachedFinalGoal(true);
  }

  //roate p by ori...
  Rotation2D<double> rot_2d(diff_lin * direction_linear);

  p_2d = rot_2d * p_2d;


  double tmp_angular_scaled = (diff_ang / diff_max) * direction_angular;  //scale between -1..1

  diff_scale.angular = tmp_angular_scaled;

  tmp_angular_scaled = std::abs(tmp_angular_scaled) * _cfg.wait_for_rotation;

  if(tmp_angular_scaled > 1.0)
    tmp_angular_scaled = 1.0;

  tmp_angular_scaled = 1.0 - tmp_angular_scaled;

  //scale length to 1 or to lin_end_approach...
  if((this->getPathLengthRest() + p.norm()) < _cfg.lin_end_approach)
  {
    p_2d /= _cfg.lin_end_approach;
  }
  else
  {
    p_2d /= p_2d.norm();
  }

  p_2d *= tmp_angular_scaled;

  //note: min vel value is done by RonaMove class...
  diff_scale.linear_x = p_2d.x();
  diff_scale.linear_y = p_2d.y();

  return diff_scale;
}

} //namespace analyser
