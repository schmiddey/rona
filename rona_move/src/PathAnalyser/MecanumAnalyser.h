/*
 * MecanumAnalyser.h
 *
 *  Created on: 19.01.2018
 *      Author: m1ch1
 */

#ifndef MECANUMANALYSER_H_
#define MECANUMANALYSER_H_

#include <iostream>

#include "PathAnalyser_base.h"


namespace analyser{




/**
 * @todo add reached final goal...
 * @todo add compute curve fcn for scaling lin vel... scalable... (drive slowlier when hard turn is coming...)
 * @todo add scale value for how important is current rotation
 */
class MecanumAnalyser : public PathAnalyser_base
{
public:
  MecanumAnalyser(const cfg::AnalyserBase_config& cfg);
  virtual ~MecanumAnalyser();

  virtual analyser::diff_scale analyse(const analyser::pose& current_pose);

  virtual void setTargetRadius(const double r, const double r_final)
  {
    _cfg.target_radius         = r;
    _cfg.target_radius_final   = r_final;
    // _target_radius = r;
    // _target_radius_last = r_final;
  }

private: //functions


private: //data fields

  cfg::AnalyserBase_config _cfg;

  double _curr_target_raius;

  bool _is_end_approach;
};

} //namespace analyser

#endif /* MECANUMANALYSER_H_ */
