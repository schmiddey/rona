/*
 * MecanumAnalyser.h
 *
 *  Created on: 19.01.2018
 *      Author: m1ch1
 */

#ifndef MECANUMANALYSER_H_
#define MECANUMANALYSER_H_

#include "PathAnalyser_base.h"

namespace analyser{

class MecanumAnalyser : public PathAnalyser_base
{
public:
  MecanumAnalyser();
  virtual ~MecanumAnalyser();

  virtual analyser::diff_scale analyse(const analyser::pose& current_pose);

private: //functions


private: //data fields
  double _target_radius;
  double _target_radius_final;
  double _curr_target_raius;

};

} //namespace analyser

#endif /* MECANUMANALYSER_H_ */
