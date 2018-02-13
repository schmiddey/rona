/*
 * LinPwrTransfere.h
 *
 *  Created on: 13.02.2018
 *      Author: m1ch1
 */

#ifndef LINPWRTRANSFERE_H_
#define LINPWRTRANSFERE_H_


#include "Controller_base.h"
#include <cmath>
#include <string>
#include <boost/lexical_cast.hpp>

namespace controller
{


/**
 * @bug if parabola_lin_fac <= 1 then max speed is never reached... maybe scale first with max vel ... and lower ramp/parabola salcing
 */
class LinPwrTransfere: public controller::Controller_base
{
public:
  LinPwrTransfere(
         double max_vel_lin,
         double max_vel_ang,
         double pwr_ratio_lin_fac,    //value between 0..1 -> 0 lin -> 0.5 lin + pwr  -> 1.0 only pwr
         double pwr_ratio_ang_fac) :  //value between 0..1 -> 0 lin -> 0.5 lin + pwr  -> 1.0 only pwr
    _max_vel_lin      (max_vel_lin      ),
    _max_vel_ang      (max_vel_ang      ),
    _pwr_ratio_lin_fac(pwr_ratio_lin_fac),
    _pwr_ratio_ang_fac(pwr_ratio_ang_fac)

  {
    //prove max values...
    _max_vel_ang = std::abs(_max_vel_ang);
    _max_vel_lin = std::abs(_max_vel_lin);
    _pwr_ratio_lin_fac = std::abs(_pwr_ratio_lin_fac);
    _pwr_ratio_ang_fac = std::abs(_pwr_ratio_ang_fac);

    this->fix_max(_pwr_ratio_lin_fac, 1.0);
    this->fix_max(_pwr_ratio_ang_fac, 1.0);
  }

   virtual controller::velocity control(double linear_x, double linear_y, double angular)
   {
     controller::velocity vel;

     vel.angular  = this->transfere_fcn(_pwr_ratio_ang_fac, angular , _max_vel_ang);
     vel.linear_x = this->transfere_fcn(_pwr_ratio_lin_fac, linear_x, _max_vel_lin);
     vel.linear_y = this->transfere_fcn(_pwr_ratio_lin_fac, linear_y, _max_vel_lin);

     return vel;
   }

private: //functions
   inline double transfere_fcn(const double scale, const double value, const double max_val)
   {
     double ret = (1 - scale) * std::abs(value) + ((1-(1-scale)) * std::pow(std::abs(value), _pwr));

     int sgn = (value > 0) - (value < 0);

     ret = std::abs(ret);

     ret *= max_val;

     this->fix_max(ret, max_val);

     return ret * (double)sgn;
   }

   inline void fix_max(double& value, const double max_val)
   {
     if(value > max_val)
       value = max_val;
   }

private: //dataelements
   double _max_vel_lin      ;
   double _max_vel_ang      ;
   double _pwr_ratio_lin_fac;
   double _pwr_ratio_ang_fac;

   int _pwr = 10;
};

} /* namespace controller */


#endif /* LINPWRTRANSFERE_H_ */
