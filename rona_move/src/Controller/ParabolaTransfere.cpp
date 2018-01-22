/*
 * ParabolaTransfere.cpp
 *
 *  Created on: 04.11.2014
 *      Author: m1ch1
 */

#include "ParabolaTransfere.h"
#include <string>
#include <iostream>

namespace controller
{

ParabolaTransfere::ParabolaTransfere(
      double max_vel_lin,
      double max_vel_ang,
      double parabola_lin_fac,
      double parabola_ang_fac)
{
   _max_vel_lin      = max_vel_lin;
   _max_vel_ang      = max_vel_ang;
   _parabola_lin_fac = parabola_lin_fac;
   _parabola_ang_fac = parabola_ang_fac;
}

ParabolaTransfere::~ParabolaTransfere()
{
}
controller::velocity ParabolaTransfere::control(double linear_x, double linear_y, double angular)
{
   controller::velocity vel;
   double ang = angular * _parabola_ang_fac;
   double sng = ang > 0 ? 1 : -1;


   vel.angular = (std::abs(ang) < _max_vel_ang ? ang : _max_vel_ang * sng);//this->parabola(_parabola_ang_fac, angular,_max_vel_ang);
   vel.linear_x = this->parabola(_parabola_lin_fac, linear_x, _max_vel_lin);
   vel.linear_y = this->parabola(_parabola_lin_fac, linear_y, _max_vel_lin);

   return vel;
}

double ParabolaTransfere::parabola(double scale, double value, double max_value)
{
   max_value = std::abs(max_value);
   int sng = value > 0 ? 1 : -1;
   double tmp = 0;
   tmp = scale * value;
   return std::abs(tmp) > max_value ? (max_value * sng) : tmp;
}

} /* namespace controller */
