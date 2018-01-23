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
 * @todo add min vel
 * @todo add reached final goal...
 * @todo add compute curve fcn for scaling lin vel... scalable...
 * @todo add scale value for how important is current rotation
 */
class MecanumAnalyser : public PathAnalyser_base
{
public:
  MecanumAnalyser(const double target_radius, const double target_radius_final, double end_approach = 0.5);
  virtual ~MecanumAnalyser();

  virtual analyser::diff_scale analyse(const analyser::pose& current_pose);

private: //functions


private: //data fields
  double _target_radius;
  double _target_radius_final;
  double _end_approach_length;

  double _curr_target_raius;

  bool _is_end_approach;
};

} //namespace analyser

#endif /* MECANUMANALYSER_H_ */
